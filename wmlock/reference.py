"""Referencing the wavemeter to a cavity-locked laser.

The wavemeter's slow error is, to first order, the same *relative* error ε on
every channel (its interferometers' effective length drifts). A laser locked to
a stable cavity sits at a fixed frequency F0, so its reading r = F0·(1 + ε)
measures ε, and every other reading is corrected with it: f = f_raw / (1 + ε).
The locks act on corrected frequencies, which holds each locked laser at a
fixed ratio to the reference laser however the wavemeter drifts. (A constant
offset instead of a relative correction is an option.)

The estimate must never follow the reference laser itself when that misbehaves
(mode hop, cavity unlock, multimode, no light). Two facts make that possible:
a misbehaving laser jumps, sweeps or runs away, and the wavemeter cannot drift
faster than some rate, max_drift.

  TRACKING   a critically damped α-β tracker (time constant tau) estimates the
             wavemeter error and its rate, so real drift is followed without
             lag. A reading further than 6σ (≥ 2 MHz) from the prediction is
             ignored; 3 in a row, invalid readings, 10 s of silence, or a
             rate above max_drift (a slow runaway) -> HOLD
  HOLD       the error is frozen and the locks carry on with it. Readings
             are collected into a run of mutually consistent values; a
             straight-line fit must show that the run is quiet and its slope
             is below max_drift, with the fit's own uncertainty, before the
             reference is believed again (4 s at least; longer when noisy)
  then       if the run sits where the frozen error predicts, within what the
             wavemeter can have drifted meanwhile (max_drift x time, capped at
             max_jump), TRACKING resumes and the difference is taken as drift;
             otherwise
  MOVED      (e.g. relocked to another cavity mode): keep holding until the
             user decides whether the laser moved (re-anchor) or the wavemeter
             did (accept as drift)
  ACQUIRING  the first verified run after enabling sets F0 (or measures the
             wavemeter error against an F0 typed in)

What it cannot see: a reference that unlocks yet drifts no faster than the
wavemeter itself is indistinguishable from wavemeter drift.
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import asdict, dataclass, fields

ACQUIRING, TRACKING, HOLD, MOVED = "acquiring", "tracking", "hold", "moved"


@dataclass
class RefParams:
    tau: float = 30.0        # s, averaging time of the wavemeter error estimate
    max_drift: float = 2.0   # MHz/min, fastest the wavemeter itself is expected to drift
    max_jump: float = 50.0   # MHz, largest change across an outage still accepted as drift
    scale: bool = True       # correction proportional to frequency (else the same offset for all)

    @classmethod
    def from_dict(cls, d) -> "RefParams":
        out = cls()
        for f in fields(cls):
            if d and f.name in d:
                setattr(out, f.name, bool(d[f.name]) if f.type in (bool, "bool") else float(d[f.name]))
        return out

    def to_dict(self) -> dict:
        return asdict(self)


def fmt_duration(s: float) -> str:
    s = max(int(s), 0)
    if s < 120:
        return f"{s} s"
    if s < 7200:
        return f"{s // 60} min"
    return f"{s // 3600} h {s % 3600 // 60} min"


class _Run:
    """Consecutive mutually consistent readings (t, MHz), with a running straight-line fit."""

    def __init__(self, window: float):
        self.window = window
        self.clear()

    def clear(self):
        self.points = deque()
        self.t0 = None
        self.n = self.st = self.sy = self.stt = self.sty = self.syy = 0.0

    def __len__(self):
        return len(self.points)

    @property
    def span(self) -> float:
        return self.points[-1][0] - self.points[0][0] if self.points else 0.0

    def _sum(self, u, y, sign):
        self.n += sign
        self.st += sign * u
        self.sy += sign * y
        self.stt += sign * u * u
        self.sty += sign * u * y
        self.syy += sign * y * y

    def add(self, t: float, y: float):
        if self.t0 is None:
            self.t0 = t
        u = t - self.t0
        self.points.append((u, y))
        self._sum(u, y, 1)
        while u - self.points[0][0] > self.window:
            self._sum(*self.points.popleft(), -1)

    def fit(self):
        """(slope MHz/s, residual rms MHz, sqrt(Stt) s, mean u, mean y) or None."""
        n = self.n
        if n < 3:
            return None
        mu, my = self.st / n, self.sy / n
        stt = self.stt - n * mu * mu
        if stt <= 0:
            return None
        sty = self.sty - n * mu * my
        b = sty / stt
        rss = max(self.syy - n * my * my - b * sty, 0.0)
        return b, math.sqrt(rss / (n - 2)), math.sqrt(stt), mu, my

    def at(self, t: float, fit) -> float:
        b, _, _, mu, my = fit
        return my + b * (t - self.t0 - mu)


class Reference:
    GATE_SIGMA = 6.0    # gate half-width in units of the reading's scatter...
    GATE_MIN = 2.0      # ...but at least this, MHz
    BAD_IN_A_ROW = 3    # rejected readings in a row that mean the reference is misbehaving
    STALE_S = 10.0      # no reading for this long: hold
    STEADY_N = 8        # a run must have this many readings...
    STEADY_S = 4.0      # ...span this long, and prove its slope is plausible
    RUN_WINDOW = 120.0  # s, longest run kept for the fit
    SIGMA_MIN = 0.05    # MHz
    SIGMA_MEMORY = 100  # readings

    def __init__(self, channel: int, params: RefParams | None = None, anchor: float | None = None,
                 error: float | None = None, error_age: float = 0.0, carry=None, now: float | None = None):
        """anchor: THz, where the reference laser really is (None: wherever it first proves steady).
        error, error_age: a saved estimate (MHz) and how long ago it was last confirmed (s);
        it is held until the reference confirms it. carry: (eps, offset MHz) of a previous
        reference, applied while acquiring so the corrected frequencies do not jump."""
        now = time.monotonic() if now is None else now
        self.channel = channel
        self.p = params or RefParams()
        self.anchor = anchor           # THz: F0
        self.error = error or 0.0      # MHz: wavemeter error at F0, reading - F0
        self.rate = 0.0                # MHz/s: its rate of change
        self.sigma = 2.0               # MHz: scatter of single readings (a guess until measured)
        self.offset = math.nan         # MHz: last reading minus prediction
        self.message = "acquiring"
        self.since = now               # when the present state (or outage) began, monotonic
        self.confirmed_at = None       # wall clock time the estimate was last confirmed
        self._carry = carry
        self._run = _Run(self.RUN_WINDOW)
        self._bad = 0
        self._t_good = now
        self._f_good = None            # THz: last accepted reading
        self._origin = anchor          # THz: what run values are measured from
        self._steady = None            # MHz: where the last verified run sits (MOVED)
        self._history = deque()        # (t, error, rate) about once a second while tracking
        self._track_start = now
        self.state = ACQUIRING
        if anchor is not None and error is not None:
            self.state, self.message = HOLD, "waiting for the reference laser"
            self.since = now - error_age
            self.confirmed_at = time.time() - error_age

    # ------------------------------------------------------------ correction
    def eps_offset(self):
        """(relative error, offset in MHz) currently applied."""
        if self.state != ACQUIRING:
            return self.error * 1e-6 / self.anchor, self.error
        return self._carry or (0.0, 0.0)

    @property
    def active(self) -> bool:
        """False only while acquiring from scratch: no correction yet."""
        return self.state != ACQUIRING or self._carry is not None

    def correct(self, f: float) -> float:
        """Corrected frequency (THz) for a raw reading f (THz)."""
        eps, off = self.eps_offset()
        return f / (1.0 + eps) if self.p.scale else f - off * 1e-6

    def correction_mhz(self, f: float) -> float:
        return (self.correct(f) - f) * 1e6

    def preview(self, f: float, scale: bool | None = None, anchor: float | None = None,
                accept: bool = False) -> float:
        """What correct(f) would become with another model, a new anchor (THz; the wavemeter error
        then being measured against it), or after accept_as_drift()."""
        if not self.active:
            return f
        scale = self.p.scale if scale is None else scale
        eps, off = self.eps_offset()
        if accept and self.state == MOVED:
            eps, off = self._steady * 1e-6 / self.anchor, self._steady
        g = f / (1.0 + eps) if scale else f - off * 1e-6
        if anchor is not None and self.anchor:
            g = g * anchor / self.anchor if scale else g + anchor - self.anchor
        return g

    # -------------------------------------------------------------- readings
    @property
    def limit(self) -> float:
        """max_drift in MHz/s."""
        return self.p.max_drift / 60.0

    def gate(self) -> float:
        return max(self.GATE_SIGMA * self.sigma, self.GATE_MIN)

    def update(self, t: float, f: float):
        """One new reading of the reference channel: f in THz, NaN if the wavemeter flagged it."""
        if not f > 0:
            self.offset = math.nan
            self._reject(t, "no signal")
            return
        if self.state == TRACKING:
            self._track_reading(t, f)
            return
        if self.state != ACQUIRING:
            self.offset = (f - self.anchor) * 1e6 - self.error
        # MHz from a fixed origin: the anchor, or the run's first reading while there is none
        if not len(self._run):
            self._origin = self.anchor if self.anchor is not None else f
        self._add_to_run(t, (f - self._origin) * 1e6)
        self._evaluate(t)

    def tick(self, t: float):
        """Call now and then: a reference that has gone quiet is put on hold."""
        if self.state == TRACKING and t - self._t_good > self.STALE_S:
            self._hold("no readings")

    def _track_reading(self, t: float, f: float):
        dt = max(t - self._t_good, 1e-3)
        predicted = self.error + self.rate * dt
        self.offset = (f - self.anchor) * 1e6 - predicted
        if abs(self.offset) > self.gate() + self.limit * dt:
            self._reject(t, f"reading jumped {self.offset:+.1f} MHz")
            return
        theta = math.exp(-min(dt, 2.0) / self.p.tau)  # critically damped (fading memory) α-β filter
        self.error = predicted + (1 - theta * theta) * self.offset
        self.rate += (1 - theta) ** 2 * self.offset / min(dt, 2.0)
        if self._f_good is not None:  # noise from successive differences, winsorised
            d2 = ((f - self._f_good) * 1e6) ** 2 / 2
            b = 1.0 / self.SIGMA_MEMORY
            self.sigma = max(math.sqrt((1 - b) * self.sigma ** 2 + b * min(d2, 16 * self.sigma ** 2)),
                             self.SIGMA_MIN)
        self._bad, self._t_good, self._f_good = 0, t, f
        self.confirmed_at = time.time()
        if abs(self.rate) > self.limit:
            self._hold(f"moving {self.rate * 60:+.1f} MHz/min, faster than the wavemeter drifts")
            return
        h = self._history
        if not h or t - h[-1][0] >= 1.0:
            h.append((t, self.error, self.rate))
            while t - h[0][0] > 3 * self.p.tau:
                h.popleft()

    def _roll_back(self):
        """A slow runaway is followed for a while (1-2 tau) before the gate or the rate bound
        catches it. If the rate has changed by more than a quarter of max_drift since 2 tau ago,
        restart from there, carried forward at the rate the wavemeter had then. (A jump or a
        loss of signal leaves the rate alone, and the estimate stands. So does a runaway within
        3 tau of (re)acquiring: there is no settled history to go back to.)"""
        t, then = self._t_good, None
        for entry in self._history:
            if entry[0] > t - 2 * self.p.tau:
                break
            if entry[0] >= self._track_start + self.p.tau:  # rate estimate settled by then
                then = entry
        if then is None or abs(self.rate - then[2]) <= self.limit / 4:
            return
        t1, error, rate = then
        rate = max(-self.limit, min(rate, self.limit))
        self.error, self.rate = error + rate * (t - t1), rate

    def _reject(self, t: float, why: str):
        self._run.clear()  # a run must be uninterrupted
        if self.state == TRACKING:
            self._bad += 1
            if self._bad >= self.BAD_IN_A_ROW:
                self._hold(why)
        elif self.state == ACQUIRING:
            self.message = f"acquiring: {why}"
        else:
            self.state, self.message = HOLD, why

    def _hold(self, why: str):
        if self.state == TRACKING:
            self._roll_back()
        # the outage counts from the last reading that confirmed the estimate
        self.state, self.message, self.since, self._bad = HOLD, why, self._t_good, 0
        self._run.clear()

    def _add_to_run(self, t: float, y: float):
        """Append, unless y does not fit the run: then the run starts again from y."""
        run = self._run
        fit = run.fit()
        if fit is not None:
            expected, spread = run.at(t, fit), max(fit[1], self.sigma)
        elif len(run):
            expected, spread = run.points[-1][1], self.sigma
        else:
            expected = None
        if expected is not None and abs(y - expected) > max(self.GATE_SIGMA * spread, self.GATE_MIN):
            run.clear()
            if self.state == ACQUIRING:
                self.message = "acquiring: reference unsteady"
            else:
                self.state, self.message = HOLD, "unsteady"
        run.add(t, y)

    def _evaluate(self, t: float):
        run = self._run
        if len(run) < self.STEADY_N or run.span < self.STEADY_S:
            return
        fit = run.fit()
        b, rms, root_stt, _, _ = fit
        se = max(rms, self.SIGMA_MIN) / root_stt  # standard error of the slope
        if rms > max(3 * self.sigma, self.sigma + 1.0):
            verdict = "noisier than usual"
        elif abs(b) - 2 * se > self.limit:
            verdict = f"moving {b * 60:+.1f} MHz/min"
        elif abs(b) + 2 * se > self.limit:
            verdict = f"verifying ({run.span:.0f} s)"
        else:
            verdict = None
        if verdict:
            self.message = ("acquiring: " if self.state == ACQUIRING else "") + verdict
            if self.state == MOVED:
                self.state = HOLD
            return
        now_value = run.at(t, fit)  # MHz from the origin, which is the anchor once there is one
        if self.state == ACQUIRING:
            f_now = self._origin + now_value * 1e-6
            if self.anchor is None:  # F0 := here, continuous with the previous correction
                eps, off = self._carry or (0.0, 0.0)
                self.anchor = f_now / (1.0 + eps) if self.p.scale else f_now - off * 1e-6
            self.error = (f_now - self.anchor) * 1e6
            self.sigma = max(rms, 0.1)
            self._carry = None
            self._track(t, "")
            return
        jump = now_value - self.error
        held = t - self.since
        tolerance = min(self.gate() + self.limit * held, max(self.p.max_jump, self.gate()))
        self.offset = jump
        if abs(jump) <= tolerance:
            self.error = now_value  # the difference is what the wavemeter drifted
            self._track(t, f"recovered after {fmt_duration(held)} ({jump:+.1f} MHz)")
        else:
            self.state, self._steady = MOVED, now_value
            self.message = f"steady but {jump:+.1f} MHz from where it should be"

    def _track(self, t: float, message: str):
        """Start tracking from a verified value. The rate starts at 0: a short run's slope is too
        uncertain to seed it, and the tracker finds the real one within about tau."""
        self.state, self.message, self.since = TRACKING, message, t
        self.rate, self._track_start = 0.0, t
        self._t_good, self._f_good, self._bad = t, None, 0
        self._run.clear()
        self._history.clear()
        self.confirmed_at = time.time()

    # ---------------------------------------------------------- user actions
    def accept_as_drift(self, t: float) -> bool:
        """MOVED: the wavemeter moved, not the laser: take the new reading as its error."""
        if self.state != MOVED:
            return False
        self.error = self._steady
        self._track(t, "offset accepted as wavemeter drift")
        return True

    def reanchor(self, t: float) -> bool:
        """MOVED: the laser moved (e.g. another cavity mode): follow it, keep the correction."""
        if self.state != MOVED:
            return False
        self.anchor += (self._steady - self.error) * 1e-6
        self._track(t, "re-anchored")
        return True

    def set_anchor(self, f: float, t: float):
        """User-given true frequency of the reference (THz): re-measure the wavemeter error against it."""
        self._carry = self.eps_offset() if self.active else None
        self.anchor, self.state, self.message, self.since = f, ACQUIRING, "acquiring", t
        self._run.clear()

    # ------------------------------------------------------------ persistence
    def to_config(self) -> dict:
        d = {"channel": self.channel, "params": self.p.to_dict()}
        if self.anchor is not None:
            d["anchor_thz"] = self.anchor
        if self.state != ACQUIRING:
            d["error_mhz"] = self.error
            d["confirmed_at"] = self.confirmed_at or time.time()
        return d

    @classmethod
    def from_config(cls, d: dict, now: float | None = None) -> "Reference":
        error = d.get("error_mhz")
        age = max(0.0, time.time() - float(d.get("confirmed_at", time.time()))) if error is not None else 0.0
        return cls(int(d["channel"]), RefParams.from_dict(d.get("params")), d.get("anchor_thz"),
                   None if error is None else float(error), age, now=now)
