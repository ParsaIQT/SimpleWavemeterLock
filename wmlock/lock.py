"""Discrete PI lock acting on one actuator: the DLC pro scan offset.

It runs once per *new* wavemeter reading of its channel, so the gains are per
reading and the loop behaves the same whatever the switch cycle time is.
Sign convention: positive gains for a laser whose frequency rises with the
offset. With tuning coefficient K (GHz/V), ki = 1/K corrects the whole error
in one reading; about a third of that is a robust choice.

Running away. Gains of the wrong sign for the laser push it away from the
setpoint, faster and faster. For WATCH_N readings after it is engaged, a lock
watches for that: the error, of one sign all the while, grown gradually (no
jump: that is a hop) from its smallest by AWAY_MIN, and by as much as it was
then (up to AWAY_MAX), while the offset moved the way the gains would bring it
back. The first time, it goes back to where the error was smallest and
reverses its gains (a laser whose frequency falls as the offset rises, or
negative gains where it rises: a channel's settings stay with the channel); a
second time, the laser does not follow the offset either way as the wavemeter
sees it: its gains are put back as they were, and it is unlocked there.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields


@dataclass
class LockParams:
    kp: float = 0.0          # proportional gain, V/GHz
    ki: float = 0.5          # integral gain, V/GHz per reading
    max_step: float = 0.25   # largest change per reading, V
    v_min: float = 0.0       # absolute limits of the offset, V
    v_max: float = 140.0
    excursion: float = 10.0  # max distance from the offset at engage, V (0 = no limit)
    capture: float = 3000.0  # |error| above this (MHz) is ignored, 3 in a row unlock (0 = off)

    @classmethod
    def from_dict(cls, d) -> "LockParams":
        names = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in (d or {}).items() if k in names})

    def to_dict(self) -> dict:
        return asdict(self)


class Lock:
    OUTLIERS = 3
    WATCH_N = 100      # readings after engaging (or reversing its gains) it is watched for running away
    AWAY_MIN = 30.0    # MHz: running away, the error grown by at least this from its smallest...
    AWAY_MAX = 300.0   # ...and by as much as it was then, up to this,
    AWAY_N = 3         # ...over this many readings at least

    def __init__(self, params: LockParams | None = None):
        self.p = params or LockParams()
        self.active = False
        self.message = ""
        self.note = ""          # what it did by itself (reversed its gains): for the caller to take
        self.reversed = False   # its gains reversed by itself since it was engaged
        self.u0 = self.u = self.integ = 0.0
        self._outliers = 0
        self._n, self._run = 0, None

    def engage(self, u0: float):
        """Start from the actuator's present value (bumpless)."""
        self.u0 = self.u = float(u0)
        self.integ = 0.0
        self._outliers = 0
        self.active, self.message = True, ""
        self.reversed = False
        self._n, self._run = 0, None

    def disengage(self, message: str = ""):
        self.active, self.message = False, message

    def limits(self):
        lo, hi = self.p.v_min, self.p.v_max
        if self.p.excursion > 0:
            lo, hi = max(lo, self.u0 - self.p.excursion), min(hi, self.u0 + self.p.excursion)
        return lo, hi

    def update(self, err_mhz: float):
        """Feed one new reading (measured - setpoint, MHz).

        Returns the new offset to send, or None to leave the laser alone.
        Disengages itself (see .message) when the error leaves the capture
        range or the offset would leave its limits.
        """
        if not self.active or not math.isfinite(err_mhz):
            return None
        p = self.p
        if p.capture > 0 and abs(err_mhz) > p.capture:
            self._outliers += 1
            if self._outliers >= self.OUTLIERS:
                self.disengage(f"unlocked: error {err_mhz / 1e3:+.2f} GHz outside capture range")
            return None
        self._outliers = 0
        back = self._away(err_mhz)
        if back is not None:
            return back
        e = err_mhz * 1e-3
        integ = self.integ + p.ki * e
        target = self.u0 - p.kp * e - integ
        u = target
        if p.max_step > 0:
            u = min(max(u, self.u - p.max_step), self.u + p.max_step)
        lo, hi = self.limits()
        if not lo <= u <= hi:
            self.disengage(f"unlocked: offset would leave {lo:.2f} … {hi:.2f}")
            return None
        if u != target:  # slew limited: keep the integrator consistent (anti-windup)
            integ = self.u0 - p.kp * e - u
        self.integ, self.u = integ, u
        return u

    def _away(self, err_mhz: float):
        """Running away (see the module's notes)? Then the offset to go back to (where the error was smallest), its
        gains reversed and the lock begun afresh from there, or (the second time) put back and unlocked; else None."""
        if self._n >= self.WATCH_N:
            return None
        self._n += 1
        size, sign = abs(err_mhz), math.copysign(1.0, err_mhz)
        run = self._run
        if run is None or run["sign"] != sign or size < run["small"]:  # (a run of one sign from its smallest)
            self._run = {"sign": sign, "small": size, "u": self.u, "n": 0, "jump": 0.0, "last": size}
            return None
        run["n"] += 1
        run["jump"], run["last"] = max(run["jump"], size - run["last"]), size
        grown, gain = size - run["small"], self.p.ki or self.p.kp
        push = -math.copysign(1.0, gain) * sign  # the way the gains move the offset, for an error of this sign
        if (not gain or run["n"] < self.AWAY_N or grown < max(self.AWAY_MIN, min(run["small"], self.AWAY_MAX))
                or run["jump"] > 0.6 * grown or (self.u - run["u"]) * push <= 0):
            return None
        back, moved = run["u"], self.u - run["u"]
        seen = f"{sign * grown:+.0f} MHz further from the setpoint as the offset moved {moved:+.3f}"
        self.p.kp, self.p.ki = -self.p.kp or 0.0, -self.p.ki or 0.0  # (not -0.0)
        self.u, self.integ = back, self.u0 - back  # (afresh from there; where it was engaged, and its limits, kept)
        self._n, self._run = 0, None
        if not self.reversed:
            self.reversed = True
            way = "rises" if (self.p.ki or self.p.kp) > 0 else "falls"
            self.note = (f"it ran away ({seen}): its frequency {way} as the offset rises, so its gains are reversed "
                         f"(P {self.p.kp:g}, I {self.p.ki:g}), from {back:.3f} again")
        else:  # (reversed, it ran away all the same)
            self.reversed = False
            self.disengage(f"unlocked: it ran away with its gains either way ({seen}): the laser does not follow this "
                           f"offset as the wavemeter sees it (the right laser, the right channel?); put back to "
                           f"{back:.3f}, its gains as they were")
        return back
