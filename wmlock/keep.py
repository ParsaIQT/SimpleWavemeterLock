"""Watching how a laser's mode is, while it runs locked, so that it can be kept off the edge of its mode before it
hops (the engine starts the keeping: modehop.Recenter.keep).

Why it can be seen coming. An ECDL does not hop out of the blue as its diode's modes drift past its external cavity's
(lasermodel.py): shortly before the mode it lases on gives way, the one that will take over begins to lase as well,
with a growing part of the power (the map's lasers show it on about half their points next to a hop; the simulator's:
ECDL.components, LaserModel.look). Two things show that second mode while the laser is still on target:

- with an SHG stage, the intra-cavity signal of its SHG cavity, locked: the cavity resonates with the main mode
  alone, so the signal falls by the part of the power the second mode takes (what the upkeep, and easing off the top
  of its mode after a relock, go by: modehop.py);
- the wavemeter's interference patterns (its 'spectrum'): the second mode's fringes add to the first's, a little
  out of step, and the fringes fade (Fringes: how deep they are in each part of the pattern, over what they were,
  which neither the exposure nor a drift of the wavemeter's own changes).

Level keeps such a signal: what it is as it should be (its baseline: a high quantile of its blocks' medians over the
last hour, so that a dip of minutes does not drag it down, while a signal that rose is followed within a fifth of
that), what it is now, its noise, and so how far it has dipped. ModeWatch has one for each signal of a channel and says when it is
time to act (alarm), not more often than its last tries allow: a dip the current did not mend (the signal low for
another reason: its TA weaker, the SHG cavity's alignment) is taken for how the signal is from then on (rebase), and
looked into again only after a wait that doubles each time.

Nothing here touches the laser. numpy, if it is there, for the patterns (without it: the amplitudes' contrast).
"""
from __future__ import annotations

import math
from collections import deque

try:
    import numpy as np
except ImportError:  # (the patterns are then not looked at: Fringes.seen returns None)
    np = None


def _median(xs) -> float:
    xs = sorted(xs)
    n = len(xs)
    if not n:
        return math.nan
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


class Level:
    """A signal that is at its highest while the laser runs on one mode, as it is watched.

    feed(t, x) takes each reading of it (while the laser runs as it should). Readings are taken BLOCK_S at a time
    (a block: their median, and their scatter); its baseline is the BASE_Q quantile of the blocks of the last BASE_S
    (prior, until there are BASE_MIN of them: what it was learned to be before, if that is known), its level now the
    median of the last NOW blocks, its noise the blocks' scatter (of a block's median). dip(): how far below its
    baseline it is now, as a fraction, and how many of its noise that is. sharp(): the same by its last SHARP_N
    readings alone, each of them (a deep dip is seen in seconds, without waiting for whole blocks)."""

    BLOCK_S = 10.0   # s of readings taken together
    BLOCK_MIN = 5    # ...at least this many of them
    BASE_S = 3600.0  # s: its baseline is taken over the blocks of this long
    BASE_Q = 0.8     # ...this quantile of their medians
    BASE_MIN = 6     # blocks before it has a baseline of its own
    NOW = 2          # blocks its level now is taken from
    SHARP_N = 6      # readings a sharp dip is seen by (each of them)

    def __init__(self, prior: float = 0.0):
        self.prior = prior      # its level as it should be, as learned before (0: not known)
        self.blocks = deque()   # (t, median, scatter of a reading, n)
        self.recent = deque(maxlen=self.SHARP_N)  # its last readings
        self._t0 = None
        self._xs = []
        self._stale = 0         # blocks still to come before its last ones say how it is now (mended)

    def reset(self, prior: float | None = None):
        """Afresh (the laser relocked, its setpoint moved: what it was is no longer what it should be)."""
        self.blocks.clear()
        self.recent.clear()
        self._t0, self._xs, self._stale = None, [], 0
        if prior is not None:
            self.prior = prior

    def mended(self, level: float | None = None):
        """The laser was just kept (its current moved, the signal back up): what was read before does not say how
        it is now (no dip until NOW blocks of new readings are in). level: what the signal is, back up, where its
        current is now: that is what it should be from here on (it goes a little with the current too, as any
        laser's power does: what it was an hour ago, at another current, is not the measure), until there are
        blocks enough of its own again."""
        self.recent.clear()
        self._t0, self._xs, self._stale = None, [], self.NOW
        if level is not None and level > 0:
            self.blocks.clear()
            self.prior = level

    def rebase(self):
        """What it is now is what it should be from now on (a dip the current did not mend: not its mode's
        doing)."""
        now = self.now()
        self.reset(now if now is not None else self.prior)

    def feed(self, t: float, x: float):
        if not (isinstance(x, (int, float)) and math.isfinite(x)):
            return
        if self._t0 is None:
            self._t0 = t
        self._xs.append(float(x))
        self.recent.append(float(x))
        if t - self._t0 >= self.BLOCK_S and len(self._xs) >= self.BLOCK_MIN:
            m = _median(self._xs)
            mad = 1.4826 * _median([abs(v - m) for v in self._xs])
            self.blocks.append((t, m, mad, len(self._xs)))
            self._t0, self._xs, self._stale = None, [], max(self._stale - 1, 0)
            while self.blocks and t - self.blocks[0][0] > self.BASE_S:
                self.blocks.popleft()

    def base(self) -> float | None:
        """Its level as it should be (None: not known yet)."""
        blocks = list(self.blocks)  # (as they are now: the window reads this while the engine goes on feeding it)
        if len(blocks) < self.BASE_MIN:
            return self.prior if self.prior > 0 else None
        ms = sorted(b[1] for b in blocks)
        return max(ms[min(int(self.BASE_Q * len(ms)), len(ms) - 1)], 0.0) or None

    def now(self) -> float | None:
        """Its level now: the median of its last NOW blocks (None: not that many yet)."""
        blocks = list(self.blocks)
        if len(blocks) < self.NOW:
            return None
        return _median([b[1] for b in blocks[-self.NOW:]])

    def noise(self) -> float:
        """The scatter of one reading of it (the median of its blocks'; 0: not known)."""
        blocks = list(self.blocks)
        return _median([b[2] for b in blocks]) if blocks else 0.0

    def dip(self) -> tuple:
        """(how far below its baseline it is now, as a fraction of that; in units of its level's noise): (0, 0) if
        there is no telling. Dipped means each of its last NOW blocks is below (not one odd block)."""
        base, now = self.base(), self.now()
        if base is None or now is None or base <= 0 or self._stale:
            return 0.0, 0.0
        last = list(self.blocks)[-self.NOW:]
        if not last:
            return 0.0, 0.0
        high = max(b[1] for b in last)  # (the least dipped of them)
        n = _median([b[3] for b in last])
        sd = max(self.noise() / math.sqrt(max(n, 1.0)), 1e-4 * base)  # (of a block's median)
        return (base - high) / base, (base - high) / sd

    def sharp(self) -> tuple:
        """As dip(), by its last SHARP_N readings alone: (how far below its baseline the highest of them is, as a
        fraction; in units of a reading's noise): (0, 0) if there is no telling."""
        base, recent = self.base(), list(self.recent)
        if base is None or base <= 0 or len(recent) < self.SHARP_N:
            return 0.0, 0.0
        high = max(recent)
        return (base - high) / base, (base - high) / max(self.noise(), 1e-4 * base)

    def trend(self) -> float:
        """Per minute, as a fraction of its baseline: how its last blocks' medians go (a least-squares line over the
        last six; 0: no telling)."""
        base = self.base()
        bs = list(self.blocks)[-6:]
        if base is None or base <= 0 or len(bs) < 4:
            return 0.0
        mt, mx = sum(b[0] for b in bs) / len(bs), sum(b[1] for b in bs) / len(bs)
        sxx = sum((b[0] - mt) ** 2 for b in bs)
        return 60.0 * sum((b[0] - mt) * (b[1] - mx) for b in bs) / sxx / base if sxx > 0 else 0.0


class Fringes:
    """How much of its fringes is left: the wavemeter's interference pattern of a laser, as a measure of how
    single-mode it is.

    A pattern is the sum of each mode's fringes; all have the same period on the CCD (to parts in a million), each
    its own place. One mode: fringes as deep as the interferometer makes them. A second mode's fringes add out of
    step, and the fringes fade (by twice its part of the power, in an interferometer where the two are half a fringe
    apart; not at all in one where they are whole fringes apart: so each interferometer by itself). The pattern is
    looked at in parts (a quarter of it each, half overlapping: the wavemeter's interferometers lie side by side on
    its CCD), and in each how deep its fringes are: the rms of what varies along it faster than its envelope
    (periods of 4 px to a fourth of the part, under a Hann window), over its mean. That is the same whatever the
    exposure and the light level, and (to a percent) wherever the fringes are along the CCD (the laser tuned, the
    wavemeter warming). seen(pattern) returns the least of the parts' depths, each over what it was when first seen
    (the average of the first AVERAGE patterns; 1: as it was; 0.9: fringes somewhere have faded by a tenth); None if
    there is no telling (no numpy, no fringes to speak of, or a pattern of another size: learned afresh)."""

    MIN_PART = 0.25  # a part counts if its fringes were this deep at least, of the deepest part's
    AVERAGE = 20     # patterns its first look is averaged over
    able = np is not None  # (without numpy, patterns are not looked at)

    def __init__(self):
        self.parts = None   # (which parts count, how deep each one's fringes were when first seen)
        self._first = []    # the first patterns' depths, until AVERAGE are in
        self._n = 0

    def reset(self):
        self.parts, self._first = None, []

    @staticmethod
    def depths(pattern):
        """How deep the fringes are in each part of the pattern (None: not a pattern)."""
        try:
            y = np.asarray(pattern, dtype=float)
        except (TypeError, ValueError):
            return None
        n = y.size
        if y.ndim != 1 or n < 64:
            return None
        size = min(n, max(256, 1 << int(math.log2(n / 4))))
        w = np.hanning(size)
        lo, hi = max(2, size // 128), size // 4
        out = []
        for a in range(0, n - size + 1, size // 2):
            part = y[a:a + size]
            mean = float((w * part).sum() / w.sum())
            if mean <= 0:
                out.append(0.0)
                continue
            power = np.abs(np.fft.rfft(w * (part - mean))) ** 2
            out.append(math.sqrt(float(power[lo:hi].sum())) / (mean * float(w.sum())))
        return np.array(out)

    def seen(self, pattern) -> float | None:
        if np is None or pattern is None:
            return None
        d = self.depths(pattern)
        if d is None:
            return None
        if len(pattern) != self._n:
            self.reset()
        if self.parts is None:
            self._n = len(pattern)
            self._first.append(d)
            if len(self._first) < self.AVERAGE:
                return None
            mean = np.mean(self._first, axis=0)
            self._first = []
            count = mean >= max(self.MIN_PART * float(mean.max()), 1e-3)
            if not count.any():
                return None
            self.parts = (count, mean[count])
            return 1.0
        count, first = self.parts
        return float(np.min(d[count] / first))


class ModeWatch:
    """A channel's laser watched for the edge of its mode: its signals (a Level each: 'shg', the SHG cavity's
    intra-cavity signal locked; 'fringes', what is left of its fringes on the wavemeter), and when to act.

    alarm(now, dip): the signal that has dipped by `dip` (a fraction of its baseline) or more, and by ALARM_SD of its
    noise (over its last blocks: some twenty seconds; or SHARP times as deep on each of its last few readings: a
    few seconds), as {signal, base, level, noise, dip, trend, way: the way its current went that mended it last,
    again: it dipped the last time too, and was back up by itself when looked at}; None: none has, or it is not
    yet time (acted on a moment ago; or the last tries did not mend it: `wait` s, doubled each time, WAIT_MAX at
    most). The engine then keeps the laser (modehop.Recenter.keep) and says how it went: kept(now, mended)."""

    ALARM_SD = 5.0     # a dip counts if it is this many times its level's noise
    SHARP = 3.0        # a dip this many times as deep as asked is not waited out: its last readings say so
    GAP = 15.0         # s after a keeping that mended it before the next
    WAIT = 300.0       # s after one that did not: doubled each time in a row...
    WAIT_MAX = 7200.0  # ...up to this

    def __init__(self):
        self.signals = {"shg": Level(), "fringes": Level()}
        self.fringes = Fringes()
        self.next = 0.0     # no alarm before then
        self.wait = 0.0     # s the last try that did not mend it has it wait (0: the last one did)
        self.idle = 0.0     # s the last look that found it back up by itself has it wait (0: the last one moved it)
        self.way = 0        # the way the current went that mended it last (+1 up, -1 down; 0: not known)
        self.kept_n = 0     # times it was kept (mended)
        self.last = None    # what the last alarm said

    def reset(self, shg_prior: float | None = None):
        """Afresh (it was relocked, recovered, its setpoint moved)."""
        self.signals["shg"].reset(shg_prior)
        self.signals["fringes"].reset()
        self.fringes.reset()

    def feed(self, signal: str, t: float, x: float):
        self.signals[signal].feed(t, x)

    def pattern(self, t: float, pattern, amps=None):
        """A reading's interference pattern (None: its amplitudes alone, [min, max, ...]: their contrast)."""
        q = self.fringes.seen(pattern)
        if q is None and pattern is None and amps and len(amps) >= 2 and amps[0] + amps[1] > 0:
            q = (amps[1] - amps[0]) / (amps[1] + amps[0])
        if q is not None:
            self.signals["fringes"].feed(t, q)
        return q

    def state(self) -> dict:
        """Each signal as it is: {signal: (dip as a fraction, in units of its noise, per minute)}, those with a
        baseline."""
        out = {}
        for name, level in self.signals.items():
            if level.base() is not None and level.now() is not None:
                out[name] = (*level.dip(), level.trend())
        return out

    def alarm(self, now: float, dip: float):
        if now < self.next:
            return None
        worst = None
        for name, level in self.signals.items():
            d, sd = level.dip()
            now_ = level.now()
            if not (d >= dip and sd >= self.ALARM_SD):
                d, sd = level.sharp()
                if not (d >= self.SHARP * dip and sd >= self.ALARM_SD):
                    continue
                now_ = max(level.recent)
            if worst is None or d > worst["dip"]:
                worst = {"signal": name, "base": level.base(), "level": now_, "noise": level.noise(),
                         "dip": d, "trend": level.trend(), "way": self.way, "again": self.idle > 0}
        self.last = worst or self.last
        return worst

    def kept(self, now: float, mended: bool, way: int = 0, rebase: bool = False, level: float | None = None):
        """How the keeping the alarm started went: mended (the current brought the signal back up; way: which way it
        went; 0: it was back up by itself, nothing moved: a signal that dips now and then is looked at less and less
        often, GAP doubled each time up to WAIT; level: what the signal is there, back up: what it should be at
        that current, from now on), or not (rebase: the current made no difference either way: the signal as it is
        now is how it is)."""
        if mended and not way:
            self.idle = min(2 * self.idle, self.WAIT) if self.idle else self.GAP
            self.wait, self.next = 0.0, now + self.idle
        elif mended:
            self.wait, self.idle, self.next, self.way = 0.0, 0.0, now + self.GAP, way
            self.kept_n += 1
            for name, signal in self.signals.items():  # (what they read before is not how it is now)
                signal.mended(level if self.last is not None and self.last["signal"] == name else None)
        else:
            self.wait = min(2 * self.wait, self.WAIT_MAX) if self.wait else self.WAIT
            self.next = now + self.wait
        if rebase and self.last is not None:
            self.signals[self.last["signal"]].rebase()
