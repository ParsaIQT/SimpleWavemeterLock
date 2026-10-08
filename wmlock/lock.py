"""Discrete PI lock acting on one actuator: the DLC pro scan offset.

It runs once per *new* wavemeter reading of its channel, so the gains are per
reading and the loop behaves the same whatever the switch cycle time is.
Sign convention: positive gains for a laser whose frequency rises with the
offset. With tuning coefficient K (GHz/V), ki = 1/K corrects the whole error
in one reading; about a third of that is a robust choice.
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

    def __init__(self, params: LockParams | None = None):
        self.p = params or LockParams()
        self.active = False
        self.message = ""
        self.u0 = self.u = self.integ = 0.0
        self._outliers = 0

    def engage(self, u0: float):
        """Start from the actuator's present value (bumpless)."""
        self.u0 = self.u = float(u0)
        self.integ = 0.0
        self._outliers = 0
        self.active, self.message = True, ""

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
