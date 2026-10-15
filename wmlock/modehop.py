"""Mode-hop recovery: put a laser that has hopped back on its target frequency, on a mode that
will hold, by moving its diode current as well as its piezo, within safe limits; and, for a laser
its DLC pro locks to a cavity (optionally an SHG laser), relock it the way you would by hand.

Why the current. An external-cavity diode laser lases on the external-cavity mode that is best
aligned with its selection filter: the grating together with the diode chip's own modes. The
piezo moves the mode and, more slowly, the filter; the current moves the filter quickly (the
chip modes shift by GHz per mA) and the mode only slowly. Temperature drifts slide the filter
until a neighbouring mode wins: a mode hop. Pulling the frequency back with the piezo alone,
as a lock would, can leave the laser at the edge of its mode, where it runs on two modes at
once and hops again (or cannot get back at all). Moving the current re-centers the filter on
the mode, and a re-centered mode holds for a long time.

Lasing. A laser can stop lasing altogether when its current drops too far, and setting the
current back does not make it lase again: it needs a good deal more current first. With a TA,
its seed power tells (HopParams.seed): below seed_min of its usual (and below the TA's own
power-min) it is not lasing, and then, at once (without seed for long, the TA switches itself
off): the current up to nominal + jump, then up to nominal + jump_max until it lases, and slowly
back down (panic). It is asked at the start, and after every move of current or piezo (on the
way down, every STEP_MAX); if it stopped, wherever that was, the recovery panics, never lets
the current that low again, and starts afresh. The current stays within nominal - i_below ...
nominal + i_window otherwise.

Two ways to a mode on target (HopParams.method):
  walk  as by hand (the default with an SHG stage). Single mode first: the SHG cavity scanned,
        its sweep looked at, the current trimmed a step at a time (up first) while the sweep gets
        no worse, until it is single-mode. Within `reach` of the target (else hop to a mode that
        is, with the current, the way that brings it nearer), walk the piezo onto the target in
        walk_step steps, looking at the frequency (a hop: afresh), whether it lases and the SHG
        sweep after each, and trimming the current as soon as the mode gets worse than it was
        (after a hop, a few steps on the way it was going, out of the new mode's edge).
        On target and single-mode there: done. A laser that is fine is left as it is: nothing
        moves but the piezo onto the target (not at all if it is on target), never the current,
        and the piezo tuning, once known, is not measured again.
  map   sweep the current over the window (below), and choose.

Which way the current comes matters (the map). A mode, once lasing, holds on past the point
where it would not have started (hysteresis): sweeping up, the laser hops onto a mode later than
it hops off it sweeping down. Where a mode lases sweeping up can be quite different from where it
lases sweeping down, and the same current gives one mode or another depending on where the
current came from. So the map keeps the two directions apart, and the laser is brought to its
operating point the way the map found its mode there.

The procedure (Recenter.run, a generator: see "Driving it" below)
  lasing?    read the seed power; not lasing: panic before anything else
  take over  (a laser locked by its DLC pro, or with an SHG stage) read how it is set up, turn
             its cavity lock off (and the SHG cavity's lock), and stop the scan if that starts
  walk       (above), or:
  near       if the laser is within reach of its target, tune the piezo onto it and check the
             margins (below). Fine: done; the cavity lock had just jumped to another peak, say.
  map        at fixed piezo, ramp the current to the top of its window and down to the bottom
             (so the laser starts on the mode a down-sweep leaves it on), then sweep it up to
             the top and back down, one reading per step. The readings fall into straight
             segments, one per mode, separated by hops; for each mode, where it lases sweeping
             up and where sweeping down. With an SHG stage its cavity is scanned meanwhile, and
             wherever the intra-cavity sweep shows the mode with a second one (cavity.py) is
             cut out of its ranges.
  choose     for each mode and direction, a range where it lases sweeping that way (found in
             either direction: that is enough). Where to operate in it: the middle of where the
             mode holds once lasing, between where it hops off sweeping up and where sweeping
             down (seen one way only: between where it hops on and off), but at least a little
             past where it hops on and short of where it hops off (an edge that is only the
             window's is taken to go on a little). The best is wide that way and needs little
             piezo to reach the target.
  park       go round by the far end of the window to the end the map's sweep that way started
             from, then sweep the current that way, a reading a step, and stop at the operating
             point: on the mode the map found there, else map again. The piezo tuning is
             measured there (the first time).
  tune       piezo onto the target, at fixed current, watching for hops.
  check      step the current by +-margin around the operating point: the frequency must
             stay on one straight line and come back, i.e. no hop within the margin (and with
             an SHG stage, single mode at both ends and in the middle). Otherwise, or after a
             big piezo move (the piezo moves the modes too), map again here, up to three maps.
  hand back  lock the SHG cavity (its intra-cavity signal must come up, as high as usual: if not,
             the master is not single-mode, however its sweep looked, and the walk trims the
             current to where the sweep's main peaks are biggest); then the cavity lock
             (relock.py): with lockpoints, start the scan, pick the TEM00 peak where the
             wavemeter puts the target among the DLC pro's lockpoint candidates, null the
             error signal's DC level with the FALC pro's input offset, select the peak, lock,
             and make sure the wavemeter and the transmission agree; else just re-enable it.
             A wrong peak: try again.
The current never leaves its window (up to i_top only to make a laser lase again; nor exceeds
the controller's own current limit), never changes by more than STEP_MAX per write, and the
piezo stays within its limits; Runner also refuses any other write but the few named in
WRITABLE. On failure, current, offset and everything else it changed go back where they were
(the current down slowly, for a laser that may stop lasing), except that a cavity lock it
turned off stays off, and a laser that stopped lasing is put back where it lased again.

Currents. The procedure writes, and works in, the set current (dl:cc:current-set); the nominal
current and everything shown are the measured one (current-act), which differs from it by a
fixed amount here (act_offset, measured when the recovery starts).

Driving it. run() yields requests: Move(current, offset) (then the next fresh reading is
sent in), Read(), Get(names) (their values come back), Set(name, value, restore),
Exec(name, args) (its output, or the controller's refusal, comes back), Wait(seconds) and
Confirm(text) (step by step: the user says go on). Readings are (t, e): monotonic seconds and
the deviation from the target in MHz (NaN when the wavemeter had no valid reading). It returns
a Result, or raises Failed. Runner does the driving for the engine; the tests drive it
synchronously against a simulated laser and controller.
"""
from __future__ import annotations

import logging
import math
import re
from collections import deque
from dataclasses import asdict, dataclass, field, fields

from . import cavity
from .dlcpro import DecopError
from .relock import CavitySteps

log = logging.getLogger(__name__)

STEP_MAX = 0.2  # mA: the most the current changes in one write; larger moves are ramped
CURRENT = "dl:cc:current-set"
OFFSET = "scan:offset"


@dataclass
class HopParams:
    enabled: bool = False     # watch the laser and recover it after a mode hop
    cavity: bool = False      # its DLC pro locks it to a cavity: take that lock over to recover
    shg: bool = False         # it has an SHG stage: check the mode in its cavity, relock it
    method: str = "auto"      # how it finds a mode on target: walk (a single-mode mode, then the piezo onto
    #                           the target), map (sweep the current), auto (walk with an SHG stage, else map)
    relock: str = "auto"      # the DLC pro's lock: auto | lockpoint (pick the peak) | plain (re-enable)
    falc: bool = True         # null the error signal's DC with the FALC pro's input offset first
    falc_max: float = 0.05    # V: the most the input offset may move from where it was
    window: float = 150.0     # MHz: the right cavity peak is within this of where the wavemeter puts it
    peak_tol: float = 50.0    # MHz: locked to its cavity, but further than this from the target: the wrong peak
    lock_min: float = 0.5     # its cavity transmission averaging below this fraction of the way from its zero to its
    #                           level locked (both learned): the lock keeps losing it
    confirm: bool = False     # step by step: ask before each step
    i_nom: float = 0.0        # mA, the laser's normal current (0: read it when enabled)
    i_window: float = 2.0     # mA: the current stays below i_nom + this, always...
    i_below: float = 1.0      # ...and above i_nom - this (lower, a laser may stop lasing)
    v_window: float = 25.0    # V: how far a recovery may move the scan offset
    reach: float = 8.0        # GHz: a single-mode mode this near the target is walked onto it with the piezo
    walk_step: float = 0.5    # V: the piezo's steps while walking, the mode looked at after each
    step: float = 0.1         # mA between readings while mapping, and of a trim
    margin: float = 0.5       # mA: wanted distance from a hop, both ways (the map checks it)
    tolerance: float = 100.0  # MHz: further than this from the target is off target (locked to a cavity: peak_tol)...
    grace: float = 2.0        # s: ...for this long (and two readings) means a hop
    max_per_hour: int = 4     # recoveries; more than that and something else is wrong
    seed: str = "auto"        # what tells it lases: auto (its TA's seed photodiode, if any) | a parameter | off
    seed_min: float = 0.5     # lasing: the seed power above this fraction of its usual (and the TA's power-min)
    jump: float = 2.0         # mA: not lasing, the current jumps to i_nom + this, then up to...
    jump_max: float = 5.0     # ...i_nom + this until it lases,
    ramp: float = 0.5         # mA/s: then comes back down to i_nom this slowly
    # learned by recoveries
    kv: float = 0.0           # MHz/V: piezo tuning
    spacing: float = 0.0      # MHz between modes
    peak_y: float = 0.0       # V: height of the right cavity peak on the lock input, in the scan
    lp_kind: str = ""         # type of the lockpoint chosen by hand, as the DLC pro marks candidates
    lp_code: int = -1         # the type number select-lockpoint takes for it (-1: not known yet)
    shg_area: float = 0.0     # the SHG sweep of the laser single-mode (cavity.ModeRef)
    shg_side: float = 0.0
    shg_n: int = 0
    falc_sign: int = 0        # +-1: how the input offset shows in the error signal (0: not known)
    seed_ok: float = 0.0      # mW: its seed power, lasing as usual
    trans_locked: float = 0.0  # V: the cavity transmission (its lock detection's input), locked as usual...
    trans_zero: float = 0.0   # V: ...and off resonance (its DC offset), measured while the master scans
    shg_lock: float = 0.0     # V: the SHG cavity's intra-cavity signal, locked, the master single-mode

    LEARNED = ("kv", "spacing", "peak_y", "lp_kind", "lp_code", "shg_area", "shg_side", "shg_n", "falc_sign",
               "seed_ok", "trans_locked", "trans_zero", "shg_lock")

    @classmethod
    def from_dict(cls, d) -> "HopParams":
        out = cls()
        for f in fields(cls):
            if d and f.name in d:
                kind = {"bool": bool, "int": int, "str": str}.get(str(f.type), float)
                try:
                    setattr(out, f.name, kind(d[f.name]))
                except (TypeError, ValueError):
                    pass
        return out

    def to_dict(self) -> dict:
        return asdict(self)

    def forget(self) -> "HopParams":
        """The same settings, with nothing learned."""
        d = self.to_dict()
        d.update({k: getattr(HopParams, k) for k in self.LEARNED})
        return HopParams.from_dict(d)

    @property
    def shg_ref(self) -> cavity.ModeRef:
        return cavity.ModeRef(self.shg_area, self.shg_side, self.shg_n)

    def off_target(self) -> float:
        """MHz: further than this from the target is off target (locked to a cavity: on the wrong peak; the
        wavemeter is never that far off)."""
        return self.peak_tol if self.cavity else self.tolerance

    def how(self, shg: bool | None = None) -> str:
        """The method it uses: 'walk' or 'map'."""
        if self.method in ("walk", "map"):
            return self.method
        return "walk" if (self.shg if shg is None else shg) else "map"

    @property
    def seed_names(self) -> tuple:
        """The parameters that may show the seed power, in order; () if it is not to be watched."""
        name = self.seed.strip()
        if name.lower() in ("", "off", "none"):
            return ()
        return SEED_NAMES if name.lower() == "auto" else (name,)

    def seed_least(self, usual: float, floor: float = 0.0) -> float:
        """mW: less seed power than this, it does not lase (0: no telling)."""
        return max(self.seed_min * usual if usual > 0 else 0.0, floor if floor > 0 else 0.0)


SEED_NAMES = ("amp:pd:seed:power", "amp:seed-limits:power")  # a TA's seed photodiode (firmware 1.7+, any)
SEED_FLOOR = "amp:seed-limits:power-min"  # the TA switches itself off below this, after a delay
TA_ON = "amp:cc:enabled"
TA_EMISSION = "amp:cc:emission"

# ------------------------------------------------------------------- requests
@dataclass
class Move:
    """Set the current (mA) and/or the scan offset (V); then send in the next fresh reading."""
    current: float | None = None
    offset: float | None = None
    read: bool = True


@dataclass
class Read:
    """Send in the next reading."""


@dataclass
class Get:
    """Read parameters (names below laserN:, or absolute: falc1:..., io:...); their values come
    back, None for any the controller does not have."""
    names: tuple


@dataclass
class Set:
    """Write a parameter. restore: what to put back if the recovery fails (None: leave it)."""
    name: str
    value: object
    restore: object = None


@dataclass
class Exec:
    """Run a command; its output comes back, or the controller's refusal (a DecopError)."""
    name: str
    args: tuple = ()


@dataclass
class Wait:
    """Let time pass: `seconds` from now, or from the last write (after_write: so that a scope
    trace read next was acquired after it)."""
    seconds: float
    after_write: bool = False


@dataclass
class Confirm:
    """Step by step: wait until the user says go on (sent on only with HopParams.confirm)."""
    text: str


# what a recovery may write besides current and offset, and how
WRITABLE = {
    "dl:lock:lock-enabled": bool, "dl:lock:lock-without-lockpoint": bool, "scan:enabled": bool,
    "nlo:shg:lock:lock-enabled": bool, "nlo:shg:scan:enabled": bool,
    "scope:channel1:signal": int, "scope:channel2:signal": int,
    "nlo:shg:scope:channel1:signal": int, "nlo:shg:scope:channel2:signal": int,
}
FALC_OFFSET = re.compile(r"falc[1-4]:input:offset$")  # within falc_max of where it was
COMMANDS = {"dl:lock:select-lockpoint": 3, "dl:lock:find-candidates": 0}  # name: number of arguments
ABSOLUTE = ("falc", "io:", "pdh", "system", "fw-ver", "serial-number")  # names not below laserN:


def full_name(laser: int, name: str) -> str:
    return name if name.startswith(ABSOLUTE) else f"laser{laser}:{name}"


def perform(client, laser: int, op):
    """Carry out one of Runner's operations with a DLCPro client; returns its result."""
    kind = op[0]
    if kind == "set":
        client.set(full_name(laser, op[1]), op[2])
        return None
    if kind == "get":
        out = []
        for name in op[1]:
            try:
                out.append(client.get_value(full_name(laser, name)))
            except DecopError:
                out.append(None)
        return out
    if kind == "exec":
        try:
            return client.exec(full_name(laser, op[1]), *op[2])
        except DecopError as e:
            return e
    raise ValueError(f"unknown operation {op!r}")


class Failed(Exception):
    def __init__(self, message: str, keep: bool = False):
        super().__init__(message)
        self.keep = keep  # leave current and offset where they are (else put them back)


class NotLasing(Exception):
    """The laser stopped lasing (its seed power, or its SHG sweep, says so), at this set current (None: it
    has not been seen lasing in this recovery, so where it stopped is not known)."""

    def __init__(self, current: float | None):
        super().__init__("not lasing" + (f" at {current:.3f} mA" if current is not None else ""))
        self.current = current


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


@dataclass
class Result:
    current: float   # mA, the new operating point
    offset: float    # V
    kv: float        # MHz/V measured, 0 if not
    spacing: float   # MHz between modes measured, 0 if not
    maps: int
    readings: int
    summary: str
    learned: dict = field(default_factory=dict)  # more HopParams fields learned
    record: dict = field(default_factory=dict)   # what it saw and did, for a look afterwards
    lock_e: float = math.nan  # MHz from the target where the wavemeter puts its cavity lock, relocked (NaN: none)


# ------------------------------------------------------------------- the map
@dataclass
class Point:
    i: float   # mA
    e: float   # MHz from the target, NaN: no valid reading
    leg: int   # sweep leg: 1 up, 2 down (3: parking)
    up: bool   # the current was rising
    bad: bool = False    # the SHG sweep there showed a second mode
    rough: bool = False  # the reading is off its mode's line by far more than the noise


def _merge(ranges, gap):
    out = []
    for a, b in sorted(ranges):
        if out and a <= out[-1][1] + gap:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def _intersect(xs, ys):
    out = []
    for a, b in xs:
        for c, d in ys:
            lo, hi = max(a, c), min(b, d)
            if hi >= lo:
                out.append((lo, hi))
    return out


def _subtract(xs, cuts):
    """Ranges xs without the ranges cuts (a range no cut touches is kept as it is, even a point)."""
    out = list(xs)
    for c, d in cuts:
        nxt = []
        for a, b in out:
            if d < a or c > b:
                nxt.append((a, b))
                continue
            if a < c:
                nxt.append((a, c))
            if d < b:
                nxt.append((d, b))
        out = nxt
    return out


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    if not n:
        return math.nan
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


@dataclass
class Mode:
    """One external-cavity mode in a map. Its current ranges (but `seen`) leave out wherever it
    showed with a second mode: by the SHG sweep, or by readings scattered off its line."""
    e_ref: float   # MHz at i_ref
    slope: float   # MHz/mA, common to all modes
    i_ref: float
    up: list = field(default_factory=list)      # current ranges where it lased, sweeping up
    down: list = field(default_factory=list)    # ... and sweeping down
    seen: list = field(default_factory=list)    # (up, lo, hi): where it lased, all of it
    hold: list = field(default_factory=list)    # either way: once lasing, it holds there
    stable: list = field(default_factory=list)  # both ways: no other mode possible there
    bad: list = field(default_factory=list)     # where it showed with a second mode (SHG sweep, rough readings)
    n: int = 0

    def at(self, i: float) -> float:
        return self.e_ref + self.slope * (i - self.i_ref)


HOP_MIN = 150.0  # MHz: steps smaller than this are never taken for a hop
ROUGH_MIN = 20.0  # MHz: a reading off its mode's line by less than this is never taken for two modes


@dataclass
class MapResult:
    modes: list
    threshold: float = HOP_MIN  # MHz: a bigger step between readings is a hop
    slope: float = 0.0          # MHz/mA within a mode
    spacing: float = 0.0        # MHz between neighbouring modes, 0 if fewer than two seen
    noise: float = 0.0          # MHz rms of single readings about the modes' lines


def analyse(points, step: float) -> MapResult:
    """The modes in a current sweep.

    Within a leg, consecutive readings that differ by more than the threshold are on
    different modes; the threshold is well above the typical step-to-step change. Segments
    are grouped into modes by their frequency extrapolated to a common current with the
    common slope. Wherever a point of a mode is bad (two modes at once, by the SHG sweep) is
    cut out of that mode's ranges, whichever leg it was seen on; and so is a point rough, its
    reading off the line through its neighbours by far more than the readings' noise: running on
    two modes, the laser shows the wavemeter now one, now the other (or a mixture)."""
    legs = {}
    for p in points:
        legs.setdefault(p.leg, []).append(p)
    pairs = []
    for pts in legs.values():
        for a, b in zip(pts, pts[1:]):
            if math.isfinite(a.e) and math.isfinite(b.e) and b.i != a.i:
                pairs.append((a, b))
    if not pairs:
        return MapResult([])
    threshold = max(HOP_MIN, 6.0 * _median([abs(b.e - a.e) for a, b in pairs]))
    slope = _median([(b.e - a.e) / (b.i - a.i) for a, b in pairs if abs(b.e - a.e) <= threshold])
    slope = 0.0 if not math.isfinite(slope) else slope

    segments = []  # (up, [points])
    for pts in legs.values():
        seg = []
        for p in pts:
            if not math.isfinite(p.e) or (seg and abs(p.e - seg[-1].e) > threshold):
                if len(seg) >= 2:
                    segments.append(seg)
                seg = []
            if math.isfinite(p.e):
                seg.append(p)
        if len(seg) >= 2:
            segments.append(seg)
    if not segments:
        return MapResult([], threshold, slope)
    # noise from second differences within segments (the straight line drops out): rms * sqrt(6)
    d2 = [abs(s[k + 1].e - 2 * s[k].e + s[k - 1].e) for s in segments for k in range(1, len(s) - 1)]
    noise = _median(d2) / 0.6745 / math.sqrt(6) if d2 else 0.0
    rough = max(ROUGH_MIN, 5.0 * math.sqrt(6) * noise)
    for s in segments:
        for k in range(1, len(s) - 1):
            if abs(s[k + 1].e - 2 * s[k].e + s[k - 1].e) > rough:
                s[k].rough = True
    i_ref = _median([p.i for p in points])
    tagged = sorted(((sum(p.e - slope * (p.i - i_ref) for p in s) / len(s), s) for s in segments),
                    key=lambda x: x[0])
    groups = []
    for e_ref, s in tagged:
        if groups and e_ref - groups[-1][0][-1] <= threshold / 2:
            groups[-1][0].append(e_ref)
            groups[-1][1].append(s)
        else:
            groups.append(([e_ref], [s]))
    modes = []
    gap = 1.5 * step + 1e-9
    for refs, segs in groups:
        n = sum(len(s) for s in segs)
        m = Mode(sum(r * len(s) for r, s in zip(refs, segs)) / n, slope, i_ref, n=n)
        up, down = [], []
        for s in segs:
            (up if s[0].up else down).append((min(p.i for p in s), max(p.i for p in s)))
        m.bad = _merge([(p.i - step / 2, p.i + step / 2) for s in segs for p in s if p.bad or p.rough], 1e-9)
        m.seen = [(True, a, z) for a, z in _merge(up, gap)] + [(False, a, z) for a, z in _merge(down, gap)]
        m.up, m.down = _subtract(_merge(up, gap), m.bad), _subtract(_merge(down, gap), m.bad)
        m.hold = _subtract(_merge(up + down, gap), m.bad)
        m.stable = _intersect(m.up, m.down)
        modes.append(m)
    gaps = [b.e_ref - a.e_ref for a, b in zip(modes, modes[1:])]
    return MapResult(modes, threshold, slope, _median(gaps) if gaps else 0.0, noise)


@dataclass
class Band:
    """A current range where a mode lases sweeping one way, and the operating current chosen in
    it, to be approached sweeping that way."""
    mode: Mode
    lo: float         # mA, as mapped
    hi: float
    v: float          # scan offset it was mapped at
    up: bool = True   # sweeping up (else down) it lases on lo..hi: park sweeping that way
    hlo: float = 0.0  # mA, the edges of where it holds either way (an edge that is only the
    hhi: float = 0.0  # window's pushed out a little): where it hops off, down and up
    op: float = 0.0   # mA, where to operate
    score: float = 0.0

    @property
    def width(self) -> float:
        return self.hi - self.lo

    @property
    def center(self) -> float:
        return 0.5 * (self.hlo + self.hhi)

    @property
    def margin(self) -> float:
        """mA from the operating point to where it hops off, the nearer way."""
        return min(self.op - self.hlo, self.hhi - self.op)

    @property
    def entry(self) -> float:
        """Where it hops on (or the window's edge it lased from), sweeping its way."""
        return self.lo if self.up else self.hi

    @property
    def way(self) -> str:
        return "up" if self.up else "down"

    @property
    def e(self) -> float:
        """Deviation from the target at the operating current, MHz."""
        return self.mode.at(self.op)


def _steps(a: float, b: float, step: float):
    """Points from a (excluded) to b (included), evenly spaced, at most `step` apart."""
    n = max(1, math.ceil(abs(b - a) / step - 1e-9))
    return [a + (b - a) * k / n for k in range(1, n + 1)]


# ------------------------------------------------------------------ procedure
class Recenter(CavitySteps):
    MAX_MAPS = 3
    REMAP_V = 2.5          # V: after tuning the piezo further than this, map again before checking
    MAX_TUNE = 12
    RELOCK_TRIES = 3
    MIN_MARGIN = 0.15      # mA: the least distance from a hop worth having
    SHIFT = 0.15           # mA/V: how far a mode's current range may move per volt of piezo (rather more)
    KV_GUESS = 300.0       # MHz/V, before anything is known
    KV_PLAUSIBLE = (20.0, 3000.0)
    VERIFY_S = 10.0        # s for the cavity lock to show the right peak
    VERIFY_N = 5           # readings in a row on target that prove it
    ARRIVE_S = 5.0         # s at most for the current to get where a quick ramp sent it
    # not lasing
    PANICS = 3             # times it may stop lasing in one recovery
    PANIC_WAIT = 0.5       # s after a jump of the current, before asking the seed power whether it lases
    STOP_MARGIN = 0.5      # mA: stopped lasing at some current, it is kept this much above it from then on
    # the walk
    WALK_ROUNDS = 6        # starts afresh (after a hop on the way, say), at most
    HEAL_STEPS = 25        # current trims each way, at most
    HEAL_EXTRA = 2         # trims on, once single-mode, for good measure
    WORSE = 0.05           # an SHG sweep this much worse (badness) than the best is getting worse

    def __init__(self, p: HopParams, current: float, offset: float, i_lo: float, i_hi: float,
                 v_lo: float, v_hi: float, kv: float = 0.0, cavity: bool = False, quick: bool = True,
                 shg: bool | None = None, act_offset: float = 0.0, i_top: float | None = None):
        """Currents are set currents (what is written); act_offset is the measured current minus
        the set current, for showing them as measured, and for the nominal current, which is.
        quick: if the laser is close to its target, first try the piezo alone (after a cavity
        lock caught the wrong peak, say); else always map, which finds the best mode (the walk
        is always as quick as it can be).
        cavity: its DLC pro has it locked to a cavity (take that over, relock it after).
        shg: it has an SHG stage (by default p.shg).
        i_top: the most current a panic (not lasing) may jump to; the rest stays within i_lo..i_hi."""
        self.p = p
        self.quick = quick
        self.I0, self.V0 = current, offset  # where the laser was when this began (where it lases, after a panic)
        self.I_start = current
        self.I, self.V = current, offset    # where it is now
        self.i_lo, self.i_hi, self.v_lo, self.v_hi = i_lo, i_hi, v_lo, v_hi
        self.i_top = max(i_hi if i_top is None else i_top, i_hi)
        self.act_offset = act_offset
        self.nominal = p.i_nom - act_offset if p.i_nom else current  # as a set current
        self.edge = None                    # 'lo' or 'hi': swept there from the other end, not moved since
        self.modes = []                     # of the last map
        self.e0 = math.nan                  # MHz from the target when it began
        self.home = None                    # sweeping up (True) or down to I0 gives the mode it began on
        self.final_act = None               # mA measured at the end
        guess = kv or p.kv
        self.kv = guess or self.KV_GUESS    # MHz/V, sign included
        self.kv_known = bool(guess)
        self.kv_measured = False
        self._kv_try = None                 # a first measurement, waiting for confirmation
        self.cavity = cavity
        self.spacing = p.spacing
        self.threshold = max(HOP_MIN, 0.4 * self.spacing) if self.spacing else 1000.0
        self.noise = 3.0                    # MHz rms of a reading, until a map measures it
        self.slope = 0.0                    # MHz/mA within a mode, once mapped
        self.t = math.nan
        self.e = math.nan
        self.stage = "starting"
        self.maps = 0
        self.readings = 0
        self.notes = []
        self.learned = {}                   # HopParams fields to keep, beyond kv and spacing
        self.record = {"notes": [], "maps": [], "sweeps": []}
        self.method = p.how(shg)
        # whether it lases
        self.seed_name = ""                 # the parameter that shows the seed power ('': not watched)
        self.seed = math.nan                # mW, as last read
        self.seed_usual = p.seed_ok         # mW, lasing as usual
        self.seed_floor = 0.0               # mW: its TA's power-min
        self.lasing = None                  # True, False, or None: no telling
        self.seen_lasing = False            # it has been seen lasing in this recovery (a reading, an SHG sweep)
        self.ta_on = None                   # its TA was on when this began
        self.panics = 0
        # the walk
        self.trim_way = 1                   # the way a current trim last made the mode single-mode
        self.trims = 0
        self.healed_hop = False             # it hopped while being trimmed single-mode
        self._hop = False                   # the last trim was a hop
        self.last_look = (None, "")         # the last SHG sweep's verdict
        self._looked = None                 # (current, offset) it was taken at
        self._slopes = deque(maxlen=9)
        CavitySteps.__init__(self, p.shg if shg is None else shg)

    # ---------------------------------------------------------------- helpers
    def mA(self, i: float) -> float:
        """A set current as measured."""
        return i + self.act_offset

    def note(self, text: str):
        self.notes.append(text)
        self.record["notes"].append([round(self.t, 2) if math.isfinite(self.t) else None, text])
        log.info("recenter: %s", text)

    def _got(self, reading):
        self.t, self.e = reading
        self.readings += 1
        self.seen_lasing |= math.isfinite(self.e)
        return self.e

    def stopped(self) -> NotLasing:
        """It does not lase: where it stopped, if it has been seen lasing in this recovery."""
        self.lasing = False
        return NotLasing(self.I if self.seen_lasing else None)

    def read(self):
        return self._got((yield Read()))

    def move(self, current=None, offset=None, read=True, check=True, top=False):
        """Set current and/or offset (within their windows; top: the current up to i_top), then (read)
        a fresh reading. check: with the seed power watched, ask it first whether the laser still
        lases (on the way down, every STEP_MAX); and if the wavemeter has no valid reading of it, ask
        the SHG sweep (alive). Raises NotLasing if it does not lase."""
        prev = self.I
        if current is not None:
            current = min(max(current, self.i_lo), self.i_top if top else self.i_hi)
            self.I = current
        if offset is not None:
            offset = min(max(offset, self.v_lo), self.v_hi)
            self.V = offset
        self.edge = None
        if check and self.seed_name:
            if current is not None and current < prev - STEP_MAX - 1e-9:  # where it stops lasing, if it does
                for i in _steps(prev, current, STEP_MAX)[:-1]:
                    yield Move(i, None, False)
                    if (yield from self.lasing_now()) is False:
                        self.I = i
                        raise NotLasing(i)
            yield Move(current, offset, False)
            if (yield from self.lasing_now()) is False:
                raise NotLasing(self.I)
            e = (yield from self.read()) if read else None
        else:
            reading = yield Move(current, offset, read)
            e = self._got(reading) if read else None
        if read and check and not math.isfinite(e) and (yield from self.alive()) is False:
            raise self.stopped()  # the wavemeter lost it: it stopped lasing, by the SHG sweep (or seed)
        if read and self.method == "walk":
            self.record.setdefault("walk", []).append(
                [round(self.V, 4), round(self.mA(self.I), 4), round(e, 1) if math.isfinite(e) else None, None])
        return e

    def come_to(self, up: bool):
        """To the bottom of the window by way of its top (up: ready to sweep up), or to the top by
        way of the bottom: the laser is then on the mode a full sweep leaves it on there, as at
        the start of the map's sweep that way. No readings: the ramps are quick."""
        want, there, other = ("lo", self.i_lo, self.i_hi) if up else ("hi", self.i_hi, self.i_lo)
        if self.edge == want:
            return
        if self.edge != ("hi" if up else "lo"):
            yield from self.move(current=other, read=False)
        yield from self.move(current=there, read=False)
        yield from self.arrive()
        self.edge = want

    def arrive(self):
        """After a quick ramp, until the measured current stops changing (if the controller limits
        its slew rate, it lags behind what is set), at most ARRIVE_S."""
        last = None
        for _ in range(int(self.ARRIVE_S / 0.1)):
            (act,) = yield Get(("dl:cc:current-act",))
            if not isinstance(act, (int, float)) or isinstance(act, bool) or not math.isfinite(act):
                return
            if last is not None and abs(act - last) <= 0.005:
                return
            last = act
            yield Wait(0.1)

    def ask(self, text: str):
        """Step by step: wait for the user before doing what `text` says."""
        if self.p.confirm:
            yield Confirm(text)

    # -------------------------------------------------------- is it lasing?
    def seed_setup(self):
        """Which parameter shows the seed power (by default the TA's seed photodiode, if it has a TA),
        how much of it means lasing (seed_min of its usual, learned; no less than the TA's own
        power-min), and whether it lases now. Its emission off, there is nothing to do."""
        names = self.p.seed_names
        if not names:
            return
        values = yield Get(tuple(names) + (SEED_FLOOR, TA_ON, TA_EMISSION, "emission"))
        seeds, (floor, ta, ta_emits, emission) = values[:len(names)], values[len(names):]
        if emission is False:
            raise Failed("its emission is off")
        name = next((n for n, v in zip(names, seeds) if _num(v)), "")
        if not name:
            if self.p.seed.strip().lower() != "auto":
                self.note(f"cannot read {names[0]}: not watching whether it lases")
            return
        self.seed_name, self.seed = name, float(seeds[names.index(name)])
        self.seed_floor = float(floor) if _num(floor) and floor > 0 else 0.0
        self.ta_on = (ta and ta_emits is not False) if isinstance(ta, bool) else None
        self.lasing = self.lases(self.seed)
        least = self.p.seed_least(self.seed_usual, self.seed_floor)
        self.note(f"seed power {self.seed:.2f} mW" + (f" (it lases above {least:.2f} mW)" if least else "")
                  + (": NOT LASING" if self.lasing is False else ""))

    def lases(self, seed: float):
        """True, False, or None: no telling (yet)."""
        least = self.p.seed_least(self.seed_usual, self.seed_floor)
        return seed >= least if least > 0 else None

    def lasing_now(self):
        """Read the seed power: True lasing, False not, None no telling."""
        if not self.seed_name:
            return None
        (s,) = yield Get((self.seed_name,))
        if not _num(s):
            return None
        self.seed = float(s)
        self.lasing = self.lases(self.seed)
        return self.lasing

    def tells_lasing(self) -> bool:
        """Can it tell whether the laser lases (by its seed power, or its SHG sweep)?"""
        return bool(self.seed_name) or bool(self.shg_ok)

    def alive(self):
        """Does it lase? With its SHG cavity scanned (and its TA on), by the sweep: resonances, or flat
        (not lasing, a laser sends broadband light, which a seed photodiode takes for lasing and no
        cavity resonates with); else, or if a flat sweep may be its scan's doing, by the seed power.
        True, False, or None (no telling)."""
        if self.shg_ok and self.shg_scanning:
            on, emits = yield Get((TA_ON, TA_EMISSION))
            if on is not False and emits is not False:
                m = yield from self.shg_sweep(check=False)
                if m is not None and self.sweep_lases(m):
                    self.lasing = True
                    return True
                if m is not None and self.flat_is_dark():
                    self.lasing = False
                    return False
        return (yield from self.lasing_now())

    def flat_is_dark(self) -> bool:
        """Does a flat SHG sweep mean it does not lase (rather than a scan that misses the resonances)?
        Yes if the sweep has shown resonances (in this recovery, or locked at the start, or as learned),
        or if the wavemeter has no valid reading of it either."""
        return self.sweep_tells or not math.isfinite(self.e)

    def dark(self):
        """The SHG sweep is flat where it should not be: no narrow-band light reaches the SHG cavity.
        Its TA off: say so (Failed); else the laser does not lase: NotLasing."""
        on, emits = yield Get((TA_ON, TA_EMISSION))
        if on is False or emits is False:
            raise Failed(f"its TA is off (no light reaches the SHG cavity): switch it on again (the laser was left at "
                         f"{self.mA(self.I):.2f} mA)", keep=True)
        self.note(f"at {self.mA(self.I):.2f} mA the SHG sweep is flat: it does not lase")
        raise self.stopped()

    def panic(self, why: str, where: float | None = None):
        """It does not lase. At once (nobody is asked): the cavity lock off (it would fight), the
        current up to nominal + jump, then a mA at a time up to nominal + jump_max until it lases;
        then slowly (ramp mA/s) back down to the nominal current, watching that it keeps lasing. If
        it stops again on the way down, it is never let that low again: up once more, and down to a
        little above that. where: the current it stopped lasing at, if known: never that low again
        either. Afterwards a failure leaves it where this leaves it, lasing, rather than where it
        began. Raises Failed if it does not lase even at nominal + jump_max (the current is put
        back), keeps stopping, lases only above the window, or its TA switched itself off (those
        three leave it lasing)."""
        p = self.p
        self.panics += 1
        self.note(f"not lasing ({why})" + (f": seed power {self.seed:.2f} mW" if self.seed_name else "")
                  + ", current up at once")
        if self.cavity and not self.unlocked:  # its lock would fight the jump
            yield Set("dl:lock:lock-enabled", False, restore=False)
            self.unlocked = True
        self.home = None
        floor = None if where is None else where + self.STOP_MARGIN
        top = min(self.nominal + p.jump_max, self.i_top)
        ups, up = [], p.jump
        while up < p.jump_max - 1e-9:
            ups.append(up)
            up += 1.0
        ups.append(p.jump_max)
        for rounds in range(1, self.PANICS + 2):
            self.stage = "not lasing: current up"
            for up in ups:
                yield from self.move(current=max(min(self.nominal + up, top), self.I), read=False, check=False,
                                     top=True)
                yield Wait(self.PANIC_WAIT)
                if (yield from self.alive()) is not False:  # (the SHG sweep shows resonances again, or the seed)
                    break
            else:
                raise Failed(f"not lasing even at {self.mA(self.I):.2f} mA ("
                             + (f"seed power {self.seed:.2f} mW" if self.seed_name and self.lasing is False and not
                                self.shg_scanning else "the SHG sweep flat") + "): look at the laser")
            self.note(f"lasing at {self.mA(self.I):.2f} mA, " + ("the SHG sweep shows it again" if self.shg_scanning
                                                                  else f"seed power {self.seed:.2f} mW"))
            if floor is not None:
                self.raise_floor(floor)
            if rounds > self.PANICS:
                raise Failed(f"it keeps stopping lasing on the way down: left lasing at {self.mA(self.I):.2f} mA: "
                             f"look at the laser", keep=True)
            self.stage = "lasing again: current slowly back down"
            back = min(max(self.nominal, self.i_lo), self.i_hi)
            stopped = None
            for i in _steps(self.I, back, STEP_MAX) if self.I > back + 1e-9 else []:
                yield from self.move(current=i, read=False, check=False, top=True)
                yield Wait(STEP_MAX / max(p.ramp, 0.01))
                if (yield from self.alive()) is False:
                    stopped = i
                    break
            if stopped is None:
                break
            self.note(f"stopped lasing again at {self.mA(stopped):.2f} mA, on the way down")
            floor = stopped + self.STOP_MARGIN
        self.I0 = self.I
        self.edge = None
        if self.ta_on:
            on, emits = yield Get((TA_ON, TA_EMISSION))
            if on is False or emits is False:
                raise Failed(f"its TA switched itself off while the seed was low: switch it on again (the "
                             f"laser lases again, at {self.mA(self.I):.2f} mA)", keep=True)
        if self.panics > self.PANICS:
            raise Failed(f"it stopped lasing {self.panics} times: left lasing at {self.mA(self.I):.2f} mA: "
                         f"look at the laser", keep=True)
        self.note(f"lasing again at {self.mA(self.I):.2f} mA" + ("" if self.shg_scanning else
                                                                 f", seed power {self.seed:.2f} mW"))
        yield from self.read()

    def raise_floor(self, lo: float):
        """Never below lo (set mA) from now on: it stopped lasing a little lower. Raises Failed if that
        leaves no current in the window (it is left lasing where it is)."""
        if lo > self.i_hi + 1e-9:
            raise Failed(f"it stops lasing below {self.mA(lo - self.STOP_MARGIN):.2f} mA, too near the top of "
                         f"the window ({self.mA(self.i_hi):.2f} mA): left lasing at {self.mA(self.I):.2f} mA: "
                         f"look at the laser", keep=True)
        if lo > self.i_lo:
            self.i_lo = lo
            self.note(f"from now on, no lower than {self.mA(lo):.2f} mA")

    def tol(self) -> float:
        """How close the piezo must bring it: well inside a cavity lock's capture range."""
        return min(5.0, self.p.tolerance / 10)

    def near(self) -> float:
        """Closer than this to the target, try the piezo alone first."""
        return max(0.3 * self.spacing if self.spacing else 1500.0, 2 * self.p.tolerance)

    # ------------------------------------------------------------------ steps
    def run(self):
        yield from self.seed_setup()
        if self.lasing is False:  # first of all: without seed, its TA may switch itself off
            yield from self.panic("when the recovery began")
        if self.cavity or self.shg:
            yield from self.take_over()
        self.e0 = yield from self.read()
        if self.seed_name and not self.seed_usual and self.lasing is not False and math.isfinite(self.e0):
            self.seed_usual = self.seed  # the wavemeter reads it: lasing, as usual for now
        relocks = 0
        while True:
            try:
                yield from (self.walk() if self.method == "walk" else self.find())
                if not self.cavity and not self.shg:
                    return (yield from self.finish())
                if (yield from self.hand_back()):
                    return (yield from self.finish())
                relocks += 1
                if relocks >= self.RELOCK_TRIES:
                    raise Failed(f"relocking failed {self.RELOCK_TRIES} times, last: {self.relock_trouble}",
                                 keep=True)
                yield from self.take_back()
            except NotLasing as stop:
                yield from self.panic(f"at {self.mA(stop.current):.2f} mA, {self.stage}" if stop.current is not None
                                      else f"{self.stage}: not seen lasing in this recovery", stop.current)
                if self.cavity or self.shg:
                    yield from self.take_back()  # whatever it had locked again: off again

    def finish(self):
        names = ("dl:cc:current-act",) + ((self.seed_name,) if self.seed_name else ())
        act, *seed = yield Get(names)
        if _num(act):
            self.final_act = float(act)
        if seed and _num(seed[0]) and self.lasing is not False and seed[0] > 0:
            self.learned["seed_ok"] = round(float(seed[0]), 3)
        return self.result()

    def find(self):
        """Put the laser on target, on a mode with margins: the quick way if it is near, else map."""
        if self.shg_ok:
            yield from self.shg_scan(True)  # its sweep shows how single-mode the laser is, all along
        if self.quick and math.isfinite(self.e) and abs(self.e) <= self.near():
            yield from self.ask(f"The laser is {self.e:+.0f} MHz from its target: tune the piezo onto it")
            if ((yield from self.tune()) == "ok" and (yield from self.check(self.p.margin))
                    and (yield from self.mode_ok())):
                return
        lo, hi = self.mA(self.i_lo), self.mA(self.i_hi)
        for _ in range(self.MAX_MAPS):
            bands = yield from self.map()
            if not bands:
                yield from self.go_home()
                raise Failed(f"no mode within {lo:.2f} … {hi:.2f} mA, sweeping up or down, "
                             + ("that the piezo can bring onto the target" if self.kv_measured else "at all")
                             + self.nearest())
            while bands:
                band = bands.pop(0)
                if not (yield from self.park(band)):
                    break                 # not where the map said: map again
                if not self.kv_measured:
                    yield from self.measure_kv()
                    if self.kv_measured:  # the piezo's reach is known now: weigh the others again
                        bands = self.weigh(bands)
                outcome = yield from self.tune(band)  # (out of reach, the piezo goes as far as it may: map there)
                if outcome == "limit":
                    self.note(f"the mode at {self.mA(band.op):.2f} mA is out of the piezo's reach")
                    continue              # next candidate
                if outcome != "ok":
                    break                 # it hopped while tuning: map again, here
                if abs(self.V - band.v) > self.REMAP_V and self.maps < self.MAX_MAPS:
                    break                 # the piezo moved the modes too: map again, here
                if ((yield from self.check(max(self.MIN_MARGIN, min(self.p.margin, 0.8 * band.margin))))
                        and (yield from self.mode_ok())):
                    return
                break                     # too close to a hop: map again, here
        yield from self.go_home()
        raise Failed(f"no mode that holds reaches the target within {lo:.2f} … {hi:.2f} mA" + self.nearest())

    def go_home(self):
        """Before giving up: back to the current it began at, the way that gives the mode it was on
        there, if the first map shows one (with hysteresis, setting the current back is not enough)."""
        if self.home is None:
            return
        way = "up" if self.home else "down"
        yield from self.ask(f"Give up: back to {self.mA(self.I0):.2f} mA sweeping {way}, onto the mode it was on")
        self.stage = "putting it back on its mode"
        if abs(self.V - self.V0) > 1e-6:
            yield from self.move(offset=self.V0, read=False)
        yield from self.come_to(self.home)
        yield from self.move(current=self.I0, read=False)
        e = yield from self.read()
        self.note(f"back at {self.mA(self.I0):.2f} mA sweeping {way}: {e:+.0f} MHz from the target "
                  f"(it began {self.e0:+.0f} MHz from it)")

    def nearest(self) -> str:
        """' (the nearest mode was ... from it)' for a failure message, after the last map ('' if none)."""
        e = min((abs(m.at(i)) for m in self.modes for i in (self.i_lo, self.i_hi)), default=math.nan)
        return f" (the nearest mode was {e / 1e3:.1f} GHz from it)" if math.isfinite(e) and e >= 1e3 else ""

    def map(self):
        """Sweep the current up over the window, then down, at fixed piezo: the modes, and where
        each lases sweeping up and sweeping down. Returns the candidate bands, best first."""
        self.maps += 1
        v, lo, hi, step = self.V, self.i_lo, self.i_hi, self.p.step
        n = 1 + 2 * len(_steps(lo, hi, step))
        yield from self.ask(f"Map the modes: by way of {self.mA(hi):.2f} mA go to {self.mA(lo):.2f} mA, then sweep "
                            f"the current up to {self.mA(hi):.2f} mA and back down in {step:g} mA steps "
                            f"({n} readings) at {v:.2f} V" + (", scanning the SHG cavity" if self.shg_ok else ""))
        self.stage = f"mapping modes ({self.maps})"
        yield from self.come_to(True)  # the up-sweep starts as the down-sweep will leave it
        points, sweeps, traces = [], [], []

        def at(i, e, leg, up):
            points.append(Point(i, e, leg, up))
            s = (yield from self.shg_sweep()) if self.shg_ok else None
            sweeps.append(s)
            traces.append(cavity.decimate(self._last_trace, 300) if s is not None else None)

        yield from at(lo, (yield from self.read()), 1, True)
        for leg, (a, b, up) in enumerate(((lo, hi, True), (hi, lo, False)), start=1):
            if leg == 2:  # the top reading starts the sweep down as well: the same laser, the same place
                points.append(Point(hi, points[-1].e, 2, False))
                sweeps.append(sweeps[-1])
                traces.append(traces[-1])
            for i in _steps(a, b, step):
                yield from at(i, (yield from self.move(current=i)), leg, up)
        self.edge = "lo"
        if any(s is not None for s in sweeps):
            ref = cavity.reference_from([s for s in sweeps if s is not None])
            if ref is not None:
                self.shg_ref = ref
            judged = [(self.judge(s), pt, s, y) for pt, s, y in zip(points, sweeps, traces) if s is not None]
            for (ok, _), pt, _, _ in judged:
                pt.bad = ok is False
            good = [j for j in judged if j[0][0]]
            bad = [j for j in judged if j[0][0] is False]
            examples = ([max(good, key=lambda j: j[2].area)] if good else []) + (
                [min(bad, key=lambda j: j[2].area)] if bad else [])  # for a look afterwards
            for (ok, text), pt, _, y in examples:
                self.record["sweeps"].append({"why": f"map {self.maps}", "i": round(self.mA(pt.i), 3), "ok": ok,
                                              "text": text, "y": y})
        res = analyse(points, step)
        modes, self.threshold, self.slope = res.modes, res.threshold, res.slope
        self.modes = modes
        if self.maps == 1 and abs(v - self.V0) < 1e-6 and math.isfinite(self.e0):
            self.home = self.homeward(modes)
        if res.spacing:
            self.spacing = res.spacing
        if res.noise:
            self.noise = res.noise
        bands = self.weigh([Band(mode, a, b, v, up) for mode in modes
                            for up, ranges in ((True, mode.up), (False, mode.down)) for a, b in ranges])
        bad, rough = sum(pt.bad for pt in points), sum(pt.rough and not pt.bad for pt in points)
        self.note(f"map {self.maps} at {v:.2f} V: {len(modes)} modes"
                  + (f", {bad} points multimode in the SHG cavity" if bad else "")
                  + (f", {rough} readings scattered (two modes?)" if rough else "")
                  + (": " if bands else "")
                  + ", ".join(f"{self.mA(b.lo):.2f}–{self.mA(b.hi):.2f} mA sweeping {b.way} ({b.e:+.0f} MHz)"
                              for b in bands[:3]))
        self.record["maps"].append({
            "v": round(v, 4), "points": [[round(self.mA(pt.i), 4), round(pt.e, 1) if math.isfinite(pt.e) else None,
                                          pt.leg, pt.bad or pt.rough] for pt in points],
            "bands": [[round(self.mA(b.lo), 3), round(self.mA(b.hi), 3), round(self.mA(b.op), 3), round(b.e, 1),
                       b.way] for b in bands[:5]]})
        return bands

    def homeward(self, modes):
        """Which way the current must come to I0 for the mode the laser began on: True (up), False
        (down), or None if the map did not see that mode there."""
        tol = max(0.5 * self.threshold, 50.0)
        for m in modes:
            if abs(m.at(self.I0) - self.e0) <= tol:
                ways = [up for up, a, z in m.seen if a - 1e-9 <= self.I0 <= z + 1e-9]
                return ways[0] if ways else None
        return None

    def weigh(self, bands):
        """The bands with their operating points (those the piezo can reach the target from, once
        its tuning is measured), best first."""
        out = [self.operating_point(b.mode, b.lo, b.hi, b.v, b.up) for b in bands]
        return sorted((b for b in out if b is not None), key=lambda b: -b.score)

    def operating_point(self, mode: Mode, lo: float, hi: float, v: float, up: bool = True):
        """Where to operate in lo..hi, where the mode lases sweeping up (or down): the middle of
        where it holds either way, but two steps (MIN_MARGIN at least) past where it hops on and
        MIN_MARGIN short of where it hops off sweeping that way, inside where it lases sweeping
        both ways if it does anywhere there, and as near as lets the piezo reach the target (once
        the tuning is measured); None if the piezo cannot reach it from anywhere there, or it is
        too close to a hop wherever in it. Seen one way only, where it holds is lo..hi itself: the
        middle of that.

        An edge at the window's edge is not a hop: the mode goes on beyond, by how much the map
        cannot tell; it is counted on for another margin's worth (unless it is where the SHG sweep
        showed a second mode)."""
        step, edge = self.p.step, 1e-6  # the map reads at both ends of the window, sweeping either way

        def cut(x):
            return any(a - step <= x <= b + step for a, b in mode.bad)

        a, z = next(((a, z) for a, z in mode.hold if a - 1e-9 <= lo and hi <= z + 1e-9), (lo, hi))
        b = Band(mode, lo, hi, v, up,
                 hlo=a - self.p.margin if a <= self.i_lo + edge and not cut(a) else a,
                 hhi=z + self.p.margin if z >= self.i_hi - edge and not cut(z) else z)
        join = max(2 * step, self.MIN_MARGIN)  # past where it hops on; short of where it hops off: MIN_MARGIN
        on = any(w == up and abs((a if up else z) - (lo if up else hi)) < 1e-9 for w, a, z in mode.seen)
        low = 0.0 if lo <= self.i_lo + edge else (join if up and on else self.MIN_MARGIN)  # (at the window's
        high = 0.0 if hi >= self.i_hi - edge else (join if on and not up else self.MIN_MARGIN)  # edge: no hop)
        lo2 = max(lo + low, b.hlo + self.MIN_MARGIN, self.i_lo)
        hi2 = min(hi - high, b.hhi - self.MIN_MARGIN, self.i_hi)
        if lo2 > hi2 + 1e-9:  # narrow: its middle
            lo2 = hi2 = min(max(0.5 * (lo + hi), self.i_lo), self.i_hi)
        if self.kv_measured:  # mode.at(i) + kv * dv = 0 with v + dv within the piezo's limits
            e1, e2 = sorted((self.kv * (v - self.v_hi), self.kv * (v - self.v_lo)))
            if abs(mode.slope) > 1e-9:
                a, z = sorted((mode.i_ref + (e1 - mode.e_ref) / mode.slope,
                               mode.i_ref + (e2 - mode.e_ref) / mode.slope))
                lo2, hi2 = max(lo2, a), min(hi2, z)
            elif not e1 <= mode.e_ref <= e2:
                return None
            if lo2 > hi2 + 1e-9:
                return None
        both = [(a, z) for a, z in mode.stable if a <= hi2 and z >= lo2 and z - a >= 2 * self.MIN_MARGIN]
        if both:  # where it lases sweeping either way: no other mode possible there, the safest; if it may
            a, z = max(both, key=lambda r: r[1] - r[0])
            a = max(lo2, a + (0.0 if a <= self.i_lo + edge else self.MIN_MARGIN))
            z = min(hi2, self.i_hi if z >= self.i_hi - edge else z - self.MIN_MARGIN)
            if a <= z + 1e-9:
                lo2, hi2 = a, z
        b.op = min(max(b.center, lo2), hi2)
        if b.margin < self.MIN_MARGIN - 1e-6:
            return None  # too close to a hop wherever in it
        b.score = (min(b.margin, 2 * self.p.margin) - self.SHIFT * abs(b.e / self.kv)
                   - 0.1 * abs(b.op - self.nominal) / max(self.p.i_window, 0.1)
                   + 0.05 * min(abs(b.op - b.entry), 2 * self.p.margin))  # well past where it hops on
        return b

    def measure_kv(self):
        """Piezo +0.5 V and back, here, in the middle of a mode where it cannot hop: how many MHz
        per volt, to choose between modes with. Each the middle of three readings: near where it
        goes multimode, the wavemeter now and then reads nonsense."""
        self.stage = "measuring the piezo tuning"
        v = self.V
        dv = 0.5 if v + 0.5 <= self.v_hi else -0.5
        e0 = yield from self.steady()
        yield from self.move(offset=v + dv)
        e1 = yield from self.steady()
        yield from self.move(offset=v)
        e2 = yield from self.steady()
        a, b = (e1 - e0) / dv, (e1 - e2) / dv
        lo, hi = self.KV_PLAUSIBLE
        if all(math.isfinite(x) for x in (a, b)) and lo <= abs(a) <= hi and 0.7 < b / a < 1.4:
            self.kv, self.kv_known, self.kv_measured = 0.5 * (a + b), True, True
            self.note(f"piezo tuning {self.kv:+.0f} MHz/V")

    def steady(self):
        """The middle of the last reading and two more (MHz, NaN if none was valid): one odd reading
        does not count."""
        es = sorted(x for x in (self.e, (yield from self.read()), (yield from self.read())) if math.isfinite(x))
        self.e = es[len(es) // 2] if es else math.nan
        return self.e

    def park(self, band: Band):
        """To the band's operating point the way the map found its mode there: from the end of the
        window the map's sweep that way started from (come round to it by way of the other end),
        sweeping that way, a reading a step. True if it is on that mode there."""
        start = self.i_lo if band.up else self.i_hi
        yield from self.ask(f"Park at {self.mA(band.op):.2f} mA, sweeping {band.way} from {self.mA(start):.2f} mA "
                            f"as the map found the mode there ({self.mA(band.lo):.2f}–{self.mA(band.hi):.2f} mA "
                            f"sweeping {band.way}, {band.e:+.0f} MHz from the target), then tune the piezo onto it")
        self.stage = f"parking: sweeping {band.way} to {self.mA(band.op):.2f} mA"
        if abs(self.V - band.v) > 1e-6:
            yield from self.move(offset=band.v, read=False)
        yield from self.come_to(band.up)
        e, pts = math.nan, []
        for i in _steps(start, band.op, self.p.step):
            e = yield from self.move(current=i)
            pts.append([round(self.mA(i), 4), round(e, 1) if math.isfinite(e) else None, 3, False])
        if self.record["maps"]:
            self.record["maps"][-1]["points"] += pts
        es = [e]
        for _ in range(2):  # the middle of three: not fooled by one odd reading
            es.append((yield from self.read()))
        es = sorted(x for x in es if math.isfinite(x))
        e, want = es[len(es) // 2] if es else math.nan, band.e
        if math.isfinite(e) and abs(e - want) <= max(0.5 * self.threshold, 50.0):
            self.note(f"parked at {self.mA(band.op):.2f} mA sweeping {band.way}: {e:+.0f} MHz from the target, "
                      f"on the mode the map found there")
            return True
        self.note(f"parked at {self.mA(band.op):.2f} mA sweeping {band.way}, but {e:+.0f} MHz from the target, not "
                  f"the map's {want:+.0f} MHz: it did not come back the same way")
        return False

    def tune(self, band: Band | None = None):
        """Piezo onto the target at fixed current -> 'ok', 'limit', 'hop', 'lost' or 'slow'.
        With the piezo at its limit, the current finishes the job if that stays inside band."""
        self.stage = "tuning the piezo"
        for _ in range(self.MAX_TUNE):
            e = self.e
            if not math.isfinite(e):
                return "lost"
            if abs(e) <= self.tol():
                return "ok"
            reach = 3.0 if self.kv_measured else 1.0
            v = min(max(self.V + max(-reach, min(reach, -e / self.kv)), self.v_lo), self.v_hi)
            dv = v - self.V
            if abs(dv) < 1e-3:
                i = self.I - e / self.slope if band is not None and abs(self.slope) > 20 else math.nan
                lo = max(band.lo + self.MIN_MARGIN, self.i_lo) if band is not None else 0.0
                hi = min(band.hi - self.MIN_MARGIN, self.i_hi) if band is not None else 0.0
                if not lo <= i <= hi:
                    return "limit"
                e2 = yield from self.move(current=i)
                if not math.isfinite(e2) or abs(e2 - e - self.slope * (i - self.I)) > self.threshold:
                    return "hop"
                continue
            e2 = yield from self.move(offset=v)
            if not math.isfinite(e2):
                return "lost"
            kv = (e2 - e) / dv
            lo, hi = self.KV_PLAUSIBLE
            if not lo <= abs(kv) <= hi and abs(e2 - e) > self.threshold:
                return "hop"  # a 'tuning' no piezo has
            predicted = self.kv * dv
            if self.kv_measured and abs(e2 - e - predicted) > max(self.threshold, 0.5 * abs(predicted)):
                return "hop"
            if abs(e2 - e) < max(10 * self.noise, 20.0):
                continue  # too small a step to learn the tuning from
            if self.kv_measured:
                if 0.5 < kv / self.kv < 2:
                    self.kv = 0.7 * self.kv + 0.3 * kv
            elif self._kv_try is not None and 0.7 < kv / self._kv_try < 1.4:
                self.kv, self.kv_known, self.kv_measured = 0.5 * (kv + self._kv_try), True, True
            elif lo <= abs(kv) <= hi:
                self.kv = self._kv_try = kv  # use it, but believe it once a second step agrees
        return "ok" if abs(self.e) <= self.tol() else "slow"

    def check(self, m: float):
        """Step the current by +-m around here: True if the frequency stays on one straight line
        and comes back (no hop within m), and with an SHG stage, single mode at both ends.
        Finishes back on target."""
        self.stage = "checking the margins"
        ic, ec = self.I, self.e
        lo, hi = max(ic - m, self.i_lo), min(ic + m, self.i_hi)
        xs, es, modes = [0.0], [ec], []
        for i in (0.5 * (ic + hi), hi, ic, 0.5 * (ic + lo), lo, ic):
            es.append((yield from self.move(current=i)))
            xs.append(i - ic)
            if self.shg_ok and i in (hi, lo) and i != ic:
                s = yield from self.shg_sweep()
                ok, text = self.judge(s) if s is not None else (None, "")
                if s is not None:
                    self.keep_sweep(s, f"check {i - ic:+.2f} mA")
                if ok is False:
                    modes.append(f"{i - ic:+.2f} mA: {text}")
        if not all(math.isfinite(x) for x in es):
            return False
        n = len(xs)
        mx, me = sum(xs) / n, sum(es) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        b = sum((x - mx) * (y - me) for x, y in zip(xs, es)) / sxx if sxx > 0 else 0.0
        worst = max(abs(y - me - b * (x - mx)) for x, y in zip(xs, es))
        if worst > max(25.0, 6.0 * self.noise, 0.15 * abs(b) * m):
            self.note(f"hop within {m:.2f} mA of {self.mA(ic):.2f} mA (off the line by {worst:.0f} MHz)")
            return False
        if modes:
            self.note(f"two modes within {m:.2f} mA of {self.mA(ic):.2f} mA ({'; '.join(modes)})")
            return False
        if abs(self.e) > self.tol():
            return (yield from self.tune()) == "ok"
        return True

    # --------------------------------------------------------------- the walk
    def walk(self):
        """As by hand (method 'walk'): make the mode single-mode first, by the SHG sweep, trimming the
        current if it is not; if it is then within reach of the target (else hop to a mode that is,
        with the current), walk the piezo slowly onto the target, looking at the mode after every
        step and trimming the current whenever it gets worse. On target and single-mode (looked at
        there): done. A laser that is fine is left as it is but for the piezo onto the target: the
        current is not moved, nor the piezo to measure its tuning if that is known. A hop on the
        way: start afresh from where it is."""
        self.stage = "looking at the mode"
        if self.shg_ok:
            yield from self.shg_scan(True)
        reach, sought, before, hops = self.p.reach * 1e3, False, None, 0
        for _ in range(self.WALK_ROUNDS):
            explore, self.doubt = self.doubt, False  # its SHG cavity would not lock as it should
            self.explored += explore
            if not (yield from self.heal(explore=explore)):
                if before is not None:  # the mode it hopped to, to be nearer: the one before after all
                    yield from self.back(before, "the mode it hopped to does not go single-mode")
                    before = None
                    continue
                raise Failed(f"no single-mode current within {self.mA(self.i_lo):.2f} … {self.mA(self.i_hi):.2f} mA "
                             f"(SHG sweep: {self.last_look[1]})")
            if not math.isfinite(self.e) and not math.isfinite((yield from self.read())):
                raise Failed("no valid readings")
            if abs(self.e) <= self.tol():  # on target
                if (yield from self.single_here()):
                    return
                continue  # not single-mode here after all: trim
            if abs(self.e) > reach and not sought:
                sought, here = True, (self.V, self.I)
                if (yield from self.seek(reach)):
                    before = here
                continue  # on another mode, or back where it began: single-mode? (else walked from there)
            if not self.kv_known:  # (known from an earlier recovery: not measured again)
                for _ in range(2):
                    if not self.kv_measured:
                        yield from self.measure_kv()
                if not self.kv_measured:
                    raise Failed("cannot measure the piezo tuning (it hops, or the readings are too noisy)")
            need = -self.e / self.kv
            if not self.v_lo - 1e-6 <= self.V + need <= self.v_hi + 1e-6:
                raise Failed(f"the target is {self.e / 1e3:+.2f} GHz away"
                             + (f" (and no mode within {self.p.reach:g} GHz of it in {self.mA(self.i_lo):.2f} … "
                                f"{self.mA(self.i_hi):.2f} mA)" if abs(self.e) > reach else "")
                             + f": {need:+.1f} V of piezo, beyond the {self.v_lo:.1f} … {self.v_hi:.1f} V it may use "
                             f"(Piezo moves at most, in Mode-hop recovery…)")
            if abs(self.e) > reach:
                self.note(f"no mode nearer: walking the piezo {self.e / 1e3:+.2f} GHz")
            outcome = yield from self.walk_piezo()
            if outcome in ("ok", "retune"):
                continue  # on target: the mode looked at there (above); or the tuning to be measured
            if outcome == "hop":
                hops += 1
                continue
            if outcome == "sick" and before is not None:
                yield from self.back(before, f"no current keeps the mode it hopped to single-mode at {self.V:.2f} V")
                before = None
                continue
            if outcome == "sick":
                raise Failed(f"multimode at {self.V:.2f} V, and no current within {self.mA(self.i_lo):.2f} … "
                             f"{self.mA(self.i_hi):.2f} mA makes it single-mode (SHG sweep: {self.last_look[1]})")
            raise Failed(f"the piezo did not bring it onto the target ({outcome}, {self.e:+.0f} MHz from it)")
        raise Failed(f"it did not settle on the target, single-mode, in {self.WALK_ROUNDS} goes"
                     + (f" (it hopped {hops} times on the way)" if hops else ""))

    def single_here(self):
        """On target: single-mode here by the SHG sweep (the last one, if it was taken right here)? Its
        look is kept, and learned. True if there is no telling."""
        if not self.shg_ok:
            return True
        if self.last_look[0] and self._looked == (self.I, self.V) and self.last_sweep is not None:
            self.adopt(self.last_sweep, self.last_look[1])
            return True
        return (yield from self.mode_ok())

    def back(self, where, why: str):
        """Piezo, then current, back to where = (V, I): to the mode it was on there, if it comes back."""
        self.note(f"{why}: back to {where[0]:.2f} V, {self.mA(where[1]):.2f} mA, and walked from there")
        yield from self.move(offset=where[0])
        yield from self.move(current=where[1])

    def jumped(self, e0: float, e1: float, di: float = 0.0, dv: float = 0.0) -> bool:
        """Did the laser hop between readings e0 and e1 (MHz), the current changed by di (mA) and the
        piezo by dv (V)? (A missing reading counts as a hop: no telling where it is.)"""
        if not math.isfinite(e1):
            return True
        if not math.isfinite(e0):
            return False
        expected = self.slope * di + (self.kv * dv if self.kv_known else 0.0)
        return abs(e1 - e0 - expected) > max(self.threshold, 0.5 * abs(expected))

    def hopped(self, e0: float, di: float = 0.0, dv: float = 0.0):
        """The last reading says it hopped (jumped): two more, and the middle one decides (on two
        modes at once, the wavemeter reads now one, now the other, sometimes nonsense)."""
        return self.jumped(e0, (yield from self.steady()), di, dv)

    def learn_slope(self, di: float, de: float):
        """MHz/mA within the mode, from a current step that was not a hop."""
        if abs(di) >= 0.05 and math.isfinite(de):
            self._slopes.append(de / di)
            self.slope = _median(self._slopes)

    def badness(self, m: cavity.ModeCheck) -> float:
        """How far from single-mode an SHG sweep looks, for telling better from worse: the main
        peaks' shortfall from a single-mode laser's (or from the biggest seen). It changes smoothly
        with the power in a second mode, where the side peaks would jump: once as big as the main
        ones, they count as main ones."""
        ref = self.shg_ref
        area = ref.area if ref is not None and ref.known else self.best_area
        return max(1.0 - m.area / area, 0.0) if area > 0 else 0.0

    def badness_now(self) -> float:
        """The badness of the last SHG sweep, as things stand (the biggest main peak seen may have grown)."""
        return self.badness(self.last_sweep) if self.last_sweep is not None else 0.0

    def look(self, why: str):
        """The SHG sweep now: (single-mode, badness); single-mode None if there is no telling."""
        m = yield from self.shg_sweep()
        if m is None:
            self.last_look = (None, "no SHG sweep")
            return None, 0.0
        ok, text = self.judge(m)
        self.last_look, self._looked = (ok, text), (self.I, self.V)
        self.keep_sweep(m, why)
        walk = self.record.get("walk")
        if walk and ok is not None:
            walk[-1][3] = ok
        return ok, self.badness(m)

    def trim(self, way: int):
        """One current trim (step mA) up (+1) or down; False if the window ends there. Readings and
        a hop are taken care of by the caller (self.e, and jumped)."""
        i = self.I + way * self.p.step
        if not self.i_lo - 1e-9 <= i <= self.i_hi + 1e-9:
            return False
        e0, i0 = self.e, self.I
        yield from self.move(current=i)
        self.trims += 1
        self._hop = self.jumped(e0, self.e, self.I - i0)
        if not self._hop:
            self.learn_slope(self.I - i0, self.e - e0)
        return True

    def heal(self, want: float | None = None, again: bool = True, explore: bool = False):
        """Single-mode by the SHG sweep, and with want, its badness no more than that (on the walk: as
        good as it was): at once if it is so, or if there is no telling. Else trim the current a step
        at a time, the way that worked last (up, to begin with: down, it may stop lasing), while the
        sweep gets no worse, then the other way; once there, on while its main peaks grow, and
        HEAL_EXTRA trims more for good measure while it stays so. Short of want, the single-mode
        current found on the way with the biggest main peaks will do. explore: it looks single-mode,
        but is not (its SHG cavity would not lock as it should): what is wanted is main peaks
        clearly bigger than where it began (by 1/AREA_MIN), or else where they are biggest. False
        if there is no single-mode current. self.healed_hop: it hopped meanwhile (look again where
        it is)."""
        self.healed_hop = False
        if not self.shg_ok:
            return True

        def area():
            return self.last_sweep.area if self.last_sweep is not None else 0.0

        def good(ok, bad):
            if explore:  # single-mode, its main peaks clearly bigger than where it began
                return ok is True and area() * cavity.AREA_MIN >= a_start
            return ok is not False and (want is None or bad <= want + 1e-9)

        a_start = math.inf
        ok, bad = yield from self.look("the mode")
        if good(ok, bad):
            return True
        way = "up" if self.trim_way > 0 else "down"
        if explore:
            yield from self.ask(f"The SHG cavity would not lock as it should ({self.relock_trouble}), though its sweep "
                                f"looks single-mode: trim the current either way, {way} first, to where the sweep's "
                                f"main peaks are biggest")
        else:
            yield from self.ask(f"The mode is {'not single' if ok is False else 'getting worse'} (SHG sweep: "
                                f"{self.last_look[1]}): trim the current, {way} first, a step at a time, until it is "
                                f"{'single-mode' if ok is False else 'as it was'}")
        self.stage = "trimming the current: single mode?"
        start, e_start, first, a_start = self.I, self.e, self.last_look[1], area() or math.inf
        fine = (area(), self.I, self.e) if ok is not False else None  # the best single-mode point on this mode
        for way in (self.trim_way, -self.trim_way):
            if abs(self.I - start) > 1e-9:  # back to where it began, and the other way from there
                yield from self.move(current=start)
                if self.jumped(e_start, self.e):
                    self.healed_hop, fine = True, None
                ok, bad = yield from self.look("the mode")
                if good(ok, bad):
                    return True
            best, worse, healthy, top = bad, 0, 0, 0.0
            for _ in range(self.HEAL_STEPS + self.HEAL_EXTRA):
                if not (yield from self.trim(way)):
                    break
                if self._hop:
                    self.healed_hop, fine = True, None
                ok, bad = yield from self.look("trimming")
                if ok is not False and (fine is None or area() > fine[0]):
                    fine = (area(), self.I, self.e)
                if good(ok, bad):
                    self.trim_way = way
                    if area() > (1 + self.WORSE / 2) * top:  # its main peaks still growing: on, and HEAL_EXTRA
                        top, healthy = area(), 1               # trims past where they stop
                    else:
                        healthy += 1
                    if healthy > self.HEAL_EXTRA:
                        break
                    continue
                if healthy:  # as wanted a trim ago: back there
                    yield from self.trim(-way)
                    self.healed_hop |= self._hop
                    break
                if self._hop:
                    best, worse = bad, 0  # another mode: judged afresh
                elif bad < best - self.WORSE / 2:
                    best, worse = bad, 0
                elif bad > best + self.WORSE:
                    worse += 1
                    if worse >= 2:
                        break
            if healthy:
                self.note((f"the SHG sweep's main peaks {area() / a_start:.2f} times as big" if explore else
                           'single mode' if want is None else 'the mode as it was')
                          + f" at {self.mA(self.I):.2f} mA, trimming {'up' if way > 0 else 'down'} from "
                          f"{self.mA(start):.2f} mA (it was {first})")
                return True
        if fine is not None:  # not as good as wanted, but single-mode there
            yield from self.move(current=fine[1])
            if not self.jumped(fine[2], self.e):
                if explore:
                    self.note(f"the SHG sweep's main peaks are biggest at {self.mA(self.I):.2f} mA, trimming "
                              f"either way from {self.mA(start):.2f} mA (where they were "
                              f"{a_start / max(fine[0], 1e-12):.2f} of that)")
                else:
                    self.note(f"single mode at {self.mA(self.I):.2f} mA, the best trimming either way from "
                              f"{self.mA(start):.2f} mA (it was {first})")
                return True
            self.healed_hop = True
            if again:
                return (yield from self.heal(again=False))
        self.note(f"no single-mode current trimming either way from {self.mA(start):.2f} mA (it was {first})")
        return False

    def seek(self, reach: float):
        """The mode is further than `reach` (MHz) from the target: hop to another with the current,
        the way that brings the laser nearer (within a mode and from one to the next, a current
        change moves it the same way: as the slope says), up to i_hi or down to i_lo, a reading a
        step. True once a mode within reach lases; else back to where it began, False."""
        if not self.slope:
            yield from self.measure_slope()
        way = 1 if -self.e * self.slope > 0 else -1
        end = self.i_hi if way > 0 else self.i_lo
        yield from self.ask(f"The laser is {self.e / 1e3:+.1f} GHz from the target, too far for the piezo: sweep the "
                            f"current {'up' if way > 0 else 'down'} towards {self.mA(end):.2f} mA, until a mode "
                            f"within {reach / 1e3:g} GHz lases")
        self.stage = f"looking for a mode within {reach / 1e3:g} GHz"
        start, e_start = self.I, self.e
        while (yield from self.trim(way)):
            if self._hop and (yield from self.hopped(e_start, self.I - start)):
                self.note(f"hopped at {self.mA(self.I):.2f} mA: {self.e / 1e3:+.2f} GHz from the target")
                if math.isfinite(self.e) and abs(self.e) <= reach:
                    yield from self.settle_in(way)
                    return True
        self.note(f"sweeping the current {'up' if way > 0 else 'down'} from {self.mA(start):.2f} to "
                  f"{self.mA(self.I):.2f} mA brought no mode within {reach / 1e3:g} GHz (it began "
                  f"{e_start / 1e3:+.1f} GHz from the target): back to {self.mA(start):.2f} mA")
        yield from self.move(current=start)
        return False

    def measure_slope(self):
        """MHz/mA within the mode: the current 0.2 mA up (or down) and back."""
        i0, e0 = self.I, self.e
        di = 0.2 if i0 + 0.2 <= self.i_hi + 1e-9 else -0.2
        e1 = yield from self.move(current=i0 + di)
        e2 = yield from self.move(current=i0)
        if all(math.isfinite(x) for x in (e0, e1, e2)) and abs(e2 - e0) < self.threshold:
            self.learn_slope(di, e1 - 0.5 * (e0 + e2))

    def settle_in(self, way: int):
        """It has just hopped onto a mode: it is at that mode's edge, where the one it came from still
        lases a little. The current on, `way` (sweeping the current, the way it was going; after a hop to
        a higher frequency, down: the current moves the selection filter faster than the mode), out of
        the edge into the mode: HEAL_EXTRA steps, and on while the SHG sweep's main peaks grow, as long
        as they do not shrink and it does not hop (then a step back)."""
        if not self.shg_ok:
            return
        self.stage = "into the mode it hopped to"
        ok, _ = yield from self.look("hopped onto")
        top = self.last_sweep.area if ok is not None and self.last_sweep is not None else 0.0
        start = self.I
        for k in range(self.HEAL_STEPS):
            if not top or not (yield from self.trim(way)):
                break
            if self._hop:
                yield from self.trim(-way)
                break
            ok, _ = yield from self.look("into the mode")
            area = self.last_sweep.area if self.last_sweep is not None else 0.0
            if ok is False or area < (1 - self.WORSE) * top:
                yield from self.trim(-way)
                break
            if area > (1 + self.WORSE / 2) * top:
                top = area  # still growing: on
            elif k + 1 >= self.HEAL_EXTRA:
                break
        if abs(self.I - start) > 1e-9:
            self.note(f"into the mode it hopped to: {self.mA(start):.2f} → {self.mA(self.I):.2f} mA")

    def walk_piezo(self):
        """The piezo onto the target in steps of walk_step V. After each: the frequency (a hop?),
        whether it lases (move does that), and the mode by the SHG sweep: worse than single-mode,
        trim the current (heal). 'ok' on target, 'hop', 'retune' (the piezo did not tune it as its
        tuning says, or the wrong way: to be measured), 'limit' (the piezo may go no further), 'sick'
        (no single-mode current here), or 'slow'."""
        step = self.p.walk_step
        e = self.e
        if abs(e) > self.tol():
            away = f"{e / 1e3:+.2f} GHz" if abs(e) >= 1e3 else f"{e:+.0f} MHz"
            dv = -e / self.kv
            yield from self.ask(f"Walk the piezo onto the target: {away} away, about "
                                f"{dv:+.{1 if abs(dv) >= 1 else 2}f} V in {step:g} V steps, looking at the mode "
                                f"after each")
        self.stage = "walking the piezo onto the target"
        v_start = self.V
        start = self.badness_now() if self.shg_ok else 0.0  # how good the mode is: it should stay so
        for _ in range(int(3 * abs(e / self.kv) / step) + 20):
            e = self.e
            if abs(e) <= self.tol():
                if abs(self.V - v_start) > 1e-6:
                    self.note(f"walked the piezo {v_start:.2f} → {self.V:.2f} V onto the target")
                return "ok"
            v = min(max(self.V + max(-step, min(step, -e / self.kv)), self.v_lo), self.v_hi)
            dv = v - self.V
            if abs(dv) < 1e-4:  # the piezo may go no further: the current finishes, if a little does it
                i = self.I - e / self.slope if abs(self.slope) > 20.0 else math.nan
                if not (self.i_lo <= i <= self.i_hi and abs(i - self.I) <= self.p.margin):
                    return "limit"
                i0 = self.I
                yield from self.move(current=i)
                if self.jumped(e, self.e, self.I - i0):
                    return "hop"
                continue
            e1 = yield from self.move(offset=v)
            if self.jumped(e, e1, dv=dv) and (yield from self.hopped(e, dv=dv)):
                if math.isfinite(self.e) and abs(self.e - e) <= self.threshold:  # it moved, but not as it should
                    return self.retune((self.e - e) / dv)
                self.note(f"hopped at {self.V:.2f} V: {self.e / 1e3:+.2f} GHz from the target"
                          if math.isfinite(self.e) else f"lost it at {self.V:.2f} V")
                if math.isfinite(self.e):
                    yield from self.settle_in(-1 if self.e > e else 1)
                return "hop"
            e1 = self.e
            if abs(e1 - e) > max(10 * self.noise, 20.0):
                kv = (e1 - e) / dv
                if 0.5 < kv / self.kv < 2:
                    self.kv = 0.8 * self.kv + 0.2 * kv
                elif kv / self.kv < 0:  # away from the target: is the tuning what it was?
                    e2 = yield from self.steady()
                    if abs(e2 - e) > max(10 * self.noise, 20.0) and (e2 - e) / dv / self.kv < 0:
                        return self.retune((e2 - e) / dv)
            if self.shg_ok and abs(e1) > self.tol():
                ok, bad = yield from self.look("walking")
                if ok is False or bad > start + 2 * self.WORSE:  # it lost quality: trim the current
                    self.stage = "walking: trimming the current"
                    if not (yield from self.heal(start + self.WORSE)):
                        return "sick"
                    if self.healed_hop:
                        return "hop"
                    self.stage = "walking the piezo onto the target"
        return "slow"

    def retune(self, kv: float) -> str:
        """The piezo tuned it kv MHz/V, not as thought: its tuning to be measured again."""
        self.note(f"the piezo tuned it {kv:+.0f} MHz/V, not {self.kv:+.0f}: measured again")
        self.kv_known = self.kv_measured = False
        return "retune"

    def verify(self):
        """After relocking the cavity: VERIFY_N readings in a row within peak_tol (further, it is on the
        wrong peak). Their middle one is where the wavemeter puts the lockpoint (self.lock_e: the
        target is re-centered on it)."""
        self.stage = "waiting for the cavity lock"
        t0, run = None, []
        while True:
            e = yield from self.read()
            t0 = self.t if t0 is None else t0
            run = run + [e] if math.isfinite(e) and abs(e) <= self.p.peak_tol else []
            if len(run) >= self.VERIFY_N:
                self.lock_e = _median(run)
                return True
            if self.t - t0 > self.VERIFY_S:
                return False

    def result(self) -> Result:
        now = self.final_act if self.final_act is not None else self.mA(self.I)
        if self.method == "walk" and not self.trims and abs(self.V - self.V0) < 0.5 and not self.panics:
            summary = f"the laser itself was fine; piezo {self.V0:.2f} → {self.V:.2f} V"
        elif self.method == "walk":
            summary = (f"walked the piezo {self.V0:.2f} → {self.V:.2f} V, current {self.mA(self.I_start):.2f} → "
                       f"{now:.2f} mA measured ({self.trims} trim{'s' if self.trims != 1 else ''}, "
                       f"{self.readings} readings)")
        elif self.maps:
            summary = (f"recentered {self.mA(self.I_start):.2f} → {now:.2f} mA measured, {self.V0:.2f} → {self.V:.2f} V"
                       f" ({self.maps} map{'s' if self.maps != 1 else ''}, {self.readings} readings)")
        else:
            summary = f"the laser itself was fine; piezo {self.V0:.2f} → {self.V:.2f} V"
        if self.panics:
            summary = (f"stopped lasing{f' {self.panics} times' if self.panics > 1 else ''}, lasing again; "
                       + summary)
        if self.cavity:
            summary = "relocked, " + summary
        if self.mode_text:
            summary += f"; {self.mode_text}"
        if self.shg_text:
            summary += f"; {self.shg_text}"
        if self.falc_text:
            summary += f"; {self.falc_text}"
        self.record.update({"i0": round(self.mA(self.I_start), 4), "v0": self.V0, "i": round(self.mA(self.I), 4),
                            "v": self.V, "act": self.final_act, "act_offset": round(self.act_offset, 4),
                            "method": self.method, "panics": self.panics, "summary": summary})
        return Result(self.I, self.V, self.kv if self.kv_measured else 0.0,
                      self.spacing if self.maps else 0.0, self.maps, self.readings, summary,
                      dict(self.learned), self.record, self.lock_e if self.cavity else math.nan)


# --------------------------------------------------------------------- driver
class Runner:
    """Carries out a Recenter's requests through `io` and feeds it the channel's readings.

    io.do(op, done, failed) performs one operation on the laser's controller and calls done(result)
    or failed(exc) afterwards:
      ('set', name, value)   result None
      ('get', names)         result [value, ...], None for a parameter the controller lacks
      ('exec', name, args)   result its output, or the DecopError if the controller refused
    Names are below laserN: unless absolute (ABSOLUTE). Whatever the procedure asks for, the current
    is kept within its window (up to i_top: a panic may go that high) and ramped in steps of at
    most STEP_MAX, the offset within its window, and nothing else is written but WRITABLE, a FALC
    pro's input offset within falc_max of where it was, and COMMANDS. After a write, SETTLE
    readings are skipped: the exposure may have straddled it. On failure or cancel, everything
    written goes back where it was (a Set's `restore`; current and offset unless the failure says
    keep), before current and offset; with the seed power watched (a laser that may stop lasing
    when its current drops), the current comes down slowly (HopParams.ramp), and if it was not
    lasing, by way of the top, as a panic would. finished(runner) is called once, when it is over."""

    SETTLE = 1
    INVALID = 3          # invalid readings in a row passed over before one is handed on
    ACK_TIMEOUT = 5.0    # s
    READ_TIMEOUT = 10.0  # s
    READ_GAP = 2.5       # s (at least) without anything new from the wavemeter: no valid reading, if it can tell why
    CONFIRM_TIMEOUT = 900.0  # s: step by step, nobody answering for this long stops it

    def __init__(self, rec: Recenter, io, now: float, timeout: float = 600.0, finished=None):
        self.rec, self.io, self.finished = rec, io, finished
        self.gen = rec.run()
        self.started = now
        self.deadline = now + timeout
        self.now = now
        self.state = "running"    # running | stopping | restoring | done | failed
        self.result = None
        self.error = ""
        self.restored = False     # on failure: current, offset and all else are back where they were
        self.i_written = rec.I    # what the controller has
        self.v_written = rec.V
        self.prompt = ""          # step by step: what it is waiting to do
        self.journal = {}         # name -> what to put back on failure, in the order first written
        self.trouble = []         # what could not be put back
        self._ops = deque()
        self._then = None         # after the ops: 'read', 'advance', 'answer' or 'restored'
        self._waiting = None      # 'ack', 'read', 'wait' or 'confirm'
        self._since = now
        self._until = now
        self._skip = self._invalid = 0
        self._heard = now         # when the last reading (valid or not) came
        self._gaps = deque(maxlen=15)  # s between readings
        self._dirty = False       # written since the last reading
        self._written_at = now    # when the last write landed
        self._token = 0
        self._keep = False
        self._send(None)

    @property
    def busy(self) -> bool:
        return self.state in ("running", "stopping", "restoring")

    @property
    def stage(self) -> str:
        return self.rec.stage if self.state == "running" else "putting things back"

    @property
    def asking(self) -> str:
        """Step by step: what it waits for the user to allow, else ''."""
        return self.prompt if self._waiting == "confirm" else ""

    # ------------------------------------------------------------- the procedure
    def _send(self, value):
        try:
            req = self.gen.send(value)
        except StopIteration as stop:
            self.result = stop.value
            self._end("done")
            return
        except Failed as f:
            self._fail(str(f), keep=f.keep)
            return
        except Exception as exc:  # a bug: stop, and put things back
            log.exception("recovery")
            self._fail(f"internal error: {exc}")
            return
        if isinstance(req, Move):
            if req.offset is not None:
                self._ops.append(("offset", float(req.offset)))
            if req.current is not None:
                self._ramp(req.current)
            self._then = "read" if req.read else "advance"
            self._next()
        elif isinstance(req, Read):
            self._expect_reading()
        elif isinstance(req, Set):
            if req.restore is not None and req.name not in self.journal:
                self.journal[req.name] = req.restore
            self._ops.append(("set", req.name, req.value))
            self._then = "advance"
            self._next()
        elif isinstance(req, Get):
            self._ops.append(("get", tuple(req.names)))
            self._then = "answer"
            self._next()
        elif isinstance(req, Exec):
            self._ops.append(("exec", req.name, tuple(req.args)))
            self._then = "answer"
            self._next()
        elif isinstance(req, Wait):
            start = self._written_at if req.after_write else self.now
            self._until = start + max(float(req.seconds), 0.0)
            if self._until <= self.now:
                self._send(None)
                return
            self._waiting = "wait"
        elif isinstance(req, Confirm):
            if not self.rec.p.confirm:
                self._send(True)
                return
            self.prompt = req.text
            self._waiting, self._since = "confirm", self.now
        else:
            self._fail(f"internal error: unknown request {req!r}")

    def _ramp(self, target: float, pace: float = 0.0):
        """Current from where the last operation leaves it to target, STEP_MAX a write; with pace, a
        pause of that many seconds after each step down."""
        r = self.rec
        target = min(max(float(target), r.i_lo), r.i_top)
        last = self._ops_current()
        for i in _steps(last, target, STEP_MAX) if abs(target - last) > 1e-9 else [target]:
            self._ops.append(("current", i))
            if pace and i < last:
                self._ops.append(("pause", pace))

    def _ops_current(self) -> float:
        for op in reversed(self._ops):
            if op[0] == "current":
                return op[1]
        return self.i_written

    def _refusal(self, op) -> str:
        """Why op must not be done ('' if it may): the last line of defence."""
        r, kind = self.rec, op[0]
        if kind == "current":
            if r.i_lo - 1e-9 <= op[1] <= r.i_top + 1e-9 and abs(op[1] - self.i_written) <= STEP_MAX + 1e-9:
                return ""
            return f"refused to set {op[1]:.3f} mA (from {self.i_written:.3f} mA)"
        if kind == "offset":
            return "" if r.v_lo - 1e-9 <= op[1] <= r.v_hi + 1e-9 else f"refused to set the offset to {op[1]:.3f}"
        if kind == "set":
            name, value = op[1], op[2]
            want = WRITABLE.get(name)
            if want is not None and type(value) is want:  # noqa: E721 (no bools for ints)
                return ""
            if FALC_OFFSET.match(name) and isinstance(value, float):
                was = self.journal.get(name)
                if isinstance(was, (int, float)) and abs(value - was) <= r.p.falc_max + 1e-9:
                    return ""
            return f"refused to set {name} to {value!r}"
        if kind == "exec":
            if COMMANDS.get(op[1]) == len(op[2]):
                return ""
            return f"refused to run {op[1]}"
        return ""

    def _next(self):
        if not self._ops:
            then, self._then = self._then, None
            if then == "read":
                self._expect_reading()
            elif then == "restored":
                self.restored = not self.trouble and not self._keep
                self._end("failed", self.error + ("; could not put back " + ", ".join(self.trouble)
                                                  if self.trouble else ""))
            else:
                self._send(None)
            return
        op = self._ops.popleft()
        if op[0] == "pause":
            self._waiting, self._until = "pause", self.now + op[1]
            return
        why = self._refusal(op)
        if why:
            self._ops.clear()
            if self.state == "running":
                self._fail(why)
            else:
                self._end("failed", f"{self.error}; {why}")
            return
        self._token += 1
        tok = self._token
        self._waiting, self._since = "ack", self.now
        if op[0] == "current":
            wire = ("set", CURRENT, op[1])
        elif op[0] == "offset":
            wire = ("set", OFFSET, op[1])
        else:
            wire = op
        self.io.do(wire, lambda result=None: self._acked(tok, op, result), lambda exc: self._write_failed(tok, op, exc))

    def _acked(self, tok: int, op, result):
        if tok != self._token or not self.busy:
            return
        kind = op[0]
        if kind == "current":
            self.i_written = op[1]
        elif kind == "offset":
            self.v_written = op[1]
        if kind in ("current", "offset", "set"):
            self._dirty = True
            self._written_at = self.now
        self._waiting = None
        if self.state == "stopping":  # the operation on its way has landed: now put things back
            self._restore()
        elif self._then == "answer" and not self._ops:
            self._then = None
            self._send(result)
        else:
            self._next()

    def _write_failed(self, tok: int, op, exc: Exception):
        if tok != self._token or not self.busy:
            return
        self._waiting = None
        if self.state == "restoring":
            if op[0] == "set":  # one setting could not be put back: say so, put back the rest
                self.trouble.append(op[1])
                self._next()
                return
            self._ops.clear()
            self._end("failed", f"{self.error}; could not put it back: {exc}")
        elif isinstance(exc, OSError):  # the connection is gone: nothing more to be done
            self._ops.clear()
            self.gen.close()
            self._end("failed", f"controller: {exc}" if self.state == "running" else self.error)
        elif self.state == "stopping":
            self._ops.clear()
            self._restore()
        else:  # the controller refused something: put things back
            self._ops.clear()
            self._fail(f"controller: {exc}")

    def _expect_reading(self):
        self._waiting, self._since = "read", self.now
        self._skip = self.SETTLE if self._dirty else 0
        self._invalid = 0

    # ------------------------------------------------------------------ inputs
    def on_reading(self, t: float, e: float):
        self.now = max(self.now, t)
        if t > self._heard:
            self._gaps.append(t - self._heard)
        self._heard = max(self._heard, t)
        if self._waiting == "read":
            if self._skip > 0:
                self._skip -= 1
                return
            if not math.isfinite(e) and self._invalid < self.INVALID:
                self._invalid += 1
                return
            self._waiting, self._dirty = None, False
            self._send((t, e))
        self.tick(t)

    def tick(self, now: float):
        self.now = max(self.now, now)
        if not self.busy:
            return
        now = self.now
        if self._waiting == "ack" and now - self._since > self.ACK_TIMEOUT:
            self._write_failed(self._token, ("ack",), TimeoutError("no answer"))
        elif self._waiting == "pause" and now >= self._until:
            self._waiting = None
            self._next()
        elif self.state == "running" and self._waiting == "confirm":
            if now - self._since > self.CONFIRM_TIMEOUT:
                self._fail("nobody said go on")
        elif self.state == "running" and self._waiting == "read" and self.rec.tells_lasing() and \
                now - max(self._since, self._heard) > self._quiet():
            # nothing new from the wavemeter (a laser that does not lase gives it no signal, the same each time,
            # which is not passed on): handed on as no valid reading, for the procedure to ask whether it lases
            self._waiting, self._dirty = None, False
            self._send((now, math.nan))
        elif self.state == "running" and self._waiting == "read" and now - self._since > self.READ_TIMEOUT:
            self._fail("no valid readings")
        elif self.state == "running" and now > self.deadline:
            self._fail("it took too long")
        elif self.state == "running" and self._waiting == "wait" and now >= self._until:
            self._waiting = None
            self._send(None)

    def _quiet(self) -> float:
        """s without anything new from the wavemeter that means it has nothing to say: READ_GAP, or four of its
        usual intervals between readings of this laser."""
        return max(self.READ_GAP, 4 * _median(self._gaps) if self._gaps else 0.0)

    def confirm(self):
        """Step by step: the user says go on."""
        if self.state == "running" and self._waiting == "confirm":
            self.deadline += self.now - self._since  # waiting for the user does not count
            self._waiting, self.prompt = None, ""
            self._send(True)

    def cancel(self, why: str = "cancelled"):
        if self.state == "running":
            self._fail(why)

    # -------------------------------------------------------------------- ends
    def _fail(self, why: str, keep: bool = False):
        self.gen.close()
        self.error, self._keep = why, keep
        self.prompt = ""
        self._ops.clear()
        if self._waiting == "ack":  # let the operation on its way land first, so the ramp back starts from it
            self.state = "stopping"
            return
        self._restore()

    def _restore(self):
        self.state = "restoring"
        self._waiting = None
        for name, value in reversed(list(self.journal.items())):
            self._ops.append(("set", name, value))
        if not self._keep:
            r = self.rec
            if abs(self.v_written - r.V0) > 1e-9:
                self._ops.append(("offset", r.V0))
            pace = STEP_MAX / max(r.p.ramp, 0.01) if r.seed_name else 0.0
            if r.lasing is False:  # not lasing: by way of the top, as a panic would
                self._ramp(min(r.nominal + r.p.jump_max, r.i_top))
                self._ops.append(("pause", 1.0))
            if abs(self._ops_current() - r.I0) > 1e-9:
                self._ramp(r.I0, pace)
        if not self._ops:
            self.restored = not self._keep
            self._end("failed")
            return
        self._then = "restored"
        self._next()

    def _end(self, state: str, error: str | None = None):
        self.state = state
        if error is not None:
            self.error = error
        self._waiting = None
        if self.finished is not None:
            done, self.finished = self.finished, None
            done(self)


# ---------------------------------------------------------------------- guard
class Guard:
    """Decides that a laser has hopped: it was on target (within tolerance) for SETTLED
    readings, then is off by more than the tolerance for `grace` seconds and two readings.

    For a laser locked to a cavity by its DLC pro, `track` follows the wavemeter's slow drift
    while the laser is on target, so the target moves with the wavemeter's frame."""

    SETTLED = 5
    TRACK_S = 600.0

    def __init__(self):
        self.recent = deque()  # start times of recent recoveries
        self.track = 0.0       # MHz
        self.reset()

    def reset(self):
        self.on_target = 0
        self.armed = False
        self.off_since = None
        self.off_count = 0
        self._t = None

    def update(self, t: float, e: float, tol: float, grace: float, track: bool = False) -> bool:
        """One reading, e = deviation from the target in MHz (before tracking). True: a hop."""
        if not math.isfinite(e):
            return False
        dt = 0.0 if self._t is None else max(t - self._t, 0.0)
        self._t = t
        e -= self.track
        if abs(e) <= tol:
            self.on_target += 1
            self.off_since, self.off_count = None, 0
            if self.on_target >= self.SETTLED:
                self.armed = True
                if track:
                    self.track += e * min(dt / self.TRACK_S, 1.0)
            return False
        self.on_target = 0
        if not self.armed:
            return False
        if self.off_since is None:
            self.off_since = t
        self.off_count += 1
        return self.off_count >= 2 and t - self.off_since >= grace

    def allow(self, now: float, per_hour: int) -> bool:
        while self.recent and now - self.recent[0] > 3600.0:
            self.recent.popleft()
        if len(self.recent) >= per_hour:
            return False
        self.recent.append(now)
        return True
