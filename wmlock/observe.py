"""A laser's lock observed (ARTIQ's observe and end_observe: rpc.py), on the PC whose laser it is: from when the
observation begins until it ends, every time the laser was not locked, as [unix time it began, s]:

  its lock dropouts   a laser its DLC pro locks to a cavity: its dropout monitor's (dropouts.py), of a few ms on
  its unlocks         its state anything but 'on' (locked; a laser its DLC pro locks: watched): unlocked, locking,
                      being recovered, being mapped
  off target          if asked (mhz): its readings further than that from its setpoint, from the first such reading
                      to the first back within

One still going on when the observation ends lasts until then; one that began before it began, from when it began.
Overlapping ones are one. The engine feeds it (Engine.observe): state() at each change of the channel's state,
reading() at each reading, drop() at each dropout its monitor reads (a few seconds' worth at a time: after the end, the
observation waits for those up to its end).
"""
from __future__ import annotations

import math

MAX = 100_000  # times kept at most (more: counted, not kept)


class Observation:
    def __init__(self, n: int, tag: str, t0: float, state: str, since: float, mhz=None):
        """n: the channel; tag: its name (the caller's); t0: when it begins (time.time()); state: the channel's then,
        since then (time.time()); mhz: off target further than this (None: not looked at)."""
        self.n, self.tag, self.t0 = n, tag, t0
        self.mhz = float(mhz) if mhz is not None else None
        self.t1 = None         # when it ended (time.time())
        self.times = []        # [began, s] of those over
        self.lost = 0          # times not kept (beyond MAX)
        self.unlocked = None if state == "on" else since  # an unlock going on: since then
        self.away = None       # readings off target going on: since the first
        self.ended = 0.0       # monotonic: when it ended...
        self.deadline = 0.0    # ...and when it stops waiting for its dropouts
        self.result = None     # its times, merged, once it is over (ready)

    def _keep(self, began: float, s: float):
        if len(self.times) < MAX:
            self.times.append([began, max(s, 0.0)])
        else:
            self.lost += 1

    def state(self, t: float, state: str):
        """The channel's state changed, at t."""
        if self.t1 is not None:
            return
        if state != "on" and self.unlocked is None:
            self.unlocked = t
        elif state == "on" and self.unlocked is not None:
            self._keep(self.unlocked, t - self.unlocked)
            self.unlocked = None

    def reading(self, t: float, dev):
        """A reading at t, dev MHz from its setpoint (None or NaN: none, or no setpoint)."""
        if self.t1 is not None or self.mhz is None:
            return
        off = dev is not None and math.isfinite(dev) and abs(dev) > self.mhz
        if off and self.away is None:
            self.away = t
        elif not off and self.away is not None and dev is not None and math.isfinite(dev):
            self._keep(self.away, t - self.away)
            self.away = None

    def drop(self, began: float, s: float):
        """A lock dropout its monitor read: from began, s long. Taken if it overlaps the observation."""
        if began + s < self.t0 or (self.t1 is not None and began > self.t1):
            return
        self._keep(began, s)

    def end(self, t1: float):
        """It ends at t1: what goes on, until then."""
        if self.t1 is not None:
            return
        self.t1 = t1
        for since in (self.unlocked, self.away):
            if since is not None:
                self._keep(since, t1 - since)
        self.unlocked = self.away = None

    def merged(self) -> list:
        """Its times in order, the overlapping ones made one: [[began, s], ...]."""
        out = []
        for began, s in sorted(self.times):
            if out and began <= out[-1][0] + out[-1][1]:
                last = out[-1]
                last[1] = max(last[1], began + s - last[0])
            else:
                out.append([began, s])
        return out
