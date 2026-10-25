"""Laser maps: a laser's current and piezo, and its temperature if asked, swept over all the ground you give them,
both ways, and all that the wavemeter and its DLC pro say at every step kept in a file: for a look at the physical
laser afterwards (where each of its modes lases sweeping up and sweeping down, where it runs on two at once, where it
stops lasing and where it starts again), and for modelling it (sim.py).

The plan (MapPlan): the current, as measured (as its DLC pro shows it: Act), from i_lo to i_hi mA, i_hi given by you
and never exceeded; the piezo from v_lo to v_hi V; each in steps. One of them is swept (`sweep`: the current at each
piezo voltage, the piezo at each current, or both maps one after the other), up and then back down at each step of
the other: hysteresis makes the two ways differ, and the file keeps them apart (legs: 1 up, 2 down, 0 a descent,
below). With t_step, the temperature (dl:tc:temp-set) is stepped from t_lo to t_hi too, a whole map at each once it
has settled.

At each point: the step written, `settle` s, then `reads` readings that began after that (each reading's exposure
says when it began), with each the wavemeter's power, exposure and fringe amplitudes and its interference patterns
(the spectrum its window shows: below), and from the DLC pro its current and piezo voltage as measured, its own
photodiode and whatever else `record` names, read meanwhile. The exposure is on auto: where the laser is dim the
wavemeter measures slowly, and the map waits for it; where it gives nothing to read (the laser does not lase), it
waits only as long as the exposure says a reading would take, and once its photodiode tells dark from lasing, not
even that.

Dark. Sweeping the current down, `dark` points in a row without lasing end the sweep that way: below there it does
not lase, and every point would cost a long exposure. The next line starts there, at its own piezo voltage: lasing
there, it looks down first for where it stops (a descent, leg 0); then it sweeps up from the dark (where it starts
lasing again: a laser starts well above where it stops) to i_hi, and back down into the dark.

Patterns: the wavemeter's interference patterns (its Fizeau interferometers', and its wide interferometer's or
grating's if it has one; with its analysis on, the spectrum that computes), where a second mode shows as fringes
that beat or fade. Kept at every point (`all`); around every hop, every change between lasing and dark, every
spread of readings and every fading of the fringes, and at every `every`-th point besides (`hops`); or never
(`off`). They take most of the file.

The file (MapFile): SQLite, one per map, in the folder given (~/laser-maps), written as the map goes (a map cut short
keeps what it had). Tables: meta (the plan, the laser, the wavemeter, where it began, how it ended: JSON), lines,
points (a row a point: where, what it read, how its sample ended), readings (each reading of each point), patterns
(compressed, losslessly), events (a hop, the dark, a temperature settled...), temps (each temperature's settling).
load(path) reads one back; python -m wmlock.lasermap FILE says what is in it (--csv: its points as CSV).

Its feedforward (the DLC pro's: the current moved factor mA per volt of piezo) is off for the map: the map is of
the current as measured against the piezo, a square; with the feedforward on, a square of set current and piezo is a
slanted band of measured current, above the highest current at one corner and below threshold at another. Switching
it off takes the current up by what it added (the factor times the piezo voltage, as the DLC pro reckons it: tens of
mA at 70 V, with a factor of a few tenths), so the current is first lowered by as much as that could be (the factor
times the piezo's distance from 0 V or from its farther limit), the feedforward switched off and the current read
back (what switching it off did tells from where the DLC pro reckons it). At the end it goes back on the same way:
from a current low enough that switching it on cannot take the current above where it began.

The ideal feedforward (fit_feedforward): the boundaries between the laser's modes, where it hops, lie along lines of
constant misalignment between its external cavity's mode and its grating; their slope, dI/dV, is the feedforward that
keeps it on one mode as the piezo moves. It is fitted at the end of each map, with the hysteresis (how much later it
hops sweeping up than back sweeping down), how far apart the modes are and how fast it tunes along it, kept in the
file and said; with ff_end 'fitted' its feedforward is set to it at the end (switched on, if it was off).

The procedure (Mapper.run, a generator, as modehop's Recenter) yields Go (a move, in steps), Sample (a point), Get,
Set and Wait; MapRunner carries them out through the laser's controller, to the safety rules: the measured current
never above i_hi (nor above where it began, if that was higher: on the way back), the set current never above the
controller's current-clip, the piezo within the controller's limits and the plan's, the temperature within the
controller's limits and the plan's, nothing else written but its scan and its feedforward (switching the feedforward
refused where it could take the current too high, its factor written only while it is off), and every move in
steps. A laser its DLC pro locks to a cavity, or with its TA on, is not mapped: that lock off first; the TA off first
(without seed it switches itself off, or worse). At the end, or when stopped, it goes back where it began: the
temperature, its feedforward, then the piezo and the current, and its scan on again if it was; there the frequency
is read and compared with where it began (hysteresis may leave it on another mode).
"""
from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import queue
import sqlite3
import statistics
import sys
import threading
import time
import zlib
from array import array
from collections import deque
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path

from . import __version__
from .modehop import CURRENT, OFFSET, Failed, Get, Set, Wait

log = logging.getLogger(__name__)

CURRENT_ACT = "dl:cc:current-act"
PIEZO_ACT = "dl:pc:voltage-act"
CLIP = "dl:cc:current-clip"
FF_ON, FF = "dl:cc:feedforward-enabled", "dl:cc:feedforward-factor"
PC_MIN, PC_MAX = "dl:pc:voltage-min", "dl:pc:voltage-max"
TEMP_SET, TEMP_ACT = "dl:tc:temp-set", "dl:tc:temp-act"
TEMP_MIN, TEMP_MAX = "dl:tc:limits:temp-min", "dl:tc:limits:temp-max"
PD_NAMES = ("dl:pd:photodiode", "dl:pd:power")  # the DL pro's own photodiode, as firmwares name it (the first there)
PD_UNITS = {"dl:pd:photodiode": "µA", "dl:pd:power": "mW"}
SCAN_ON = "scan:enabled"
LOCK_ON = "dl:lock:lock-enabled"
TA_ON = "amp:cc:enabled"
ABOUT = ("product-name", "label", "type", "emission")  # kept with the map, as the controller says them
WRITABLE = {SCAN_ON: bool, FF_ON: bool, FF: float}  # what a map writes besides current, piezo and temperature
FF_MAX = 2.0     # mA/V: the most a feedforward set from a map may be
FF_SETTLE = 0.3  # s after a step of its feedforward before its current is read
FF_NOISE = 0.03  # mA: how much the DLC pro's reading of its current (current-act) scatters, at most
FF_STEP = 0.95   # mA: the most a step of its feedforward's factor may move the current, by what is known of it
LOOK = (CLIP, FF_ON, FF, PC_MIN, PC_MAX, TEMP_SET, TEMP_ACT, TEMP_MIN, TEMP_MAX, SCAN_ON, LOCK_ON, TA_ON,
        *PD_NAMES)  # what the map dialog shows of a laser before it begins (Engine.map_look)
DARK = ("no signal", "low signal")  # what the wavemeter says when there is next to no light
FOLDER = Path.home() / "laser-maps"

RAMP_I = 1.0   # mA: the most a move changes the current per write (a sweep: its step)...
RAMP_V = 5.0   # V: ...and the piezo
STEP_I = 5.0   # mA: the largest step a plan may have, and so any write
STEP_V = 10.0  # V
MAX_POINTS = 5_000_000
MODEL_MIN = 200  # points that lased, the fewest the laser's model is fitted to
PATTERN_KINDS = {0: "interferometers", 1: "wide interferometer or grating", 2: "interferometers, 2nd pulse",
                 3: "wide interferometer, 2nd pulse", 4: "analysis x", 5: "analysis y"}


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def grid(lo: float, hi: float, step: float) -> list:
    """lo to hi, evenly spaced, at most step apart, both ends included."""
    lo, hi = min(lo, hi), max(lo, hi)
    if hi - lo < 1e-9 or not step > 0:
        return [lo]
    n = max(1, math.ceil((hi - lo) / step - 1e-9))
    return [round(lo + (hi - lo) * k / n, 9) for k in range(n + 1)]


def fmt_time(s: float) -> str:
    s = max(int(round(s)), 0)
    if s < 60:
        return f"{s} s"
    if s < 3600:
        return f"{s // 60} min {s % 60:02d} s" if s < 600 else f"{s // 60} min"
    return f"{s // 3600} h {s % 3600 // 60:02d} min"


def fmt_bytes(n: float) -> str:
    for unit in ("bytes", "kB", "MB", "GB"):
        if n < 1000 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "bytes" else f"{n:.3g} {unit}"
        n /= 1000.0
    return ""


@dataclass
class MapPlan:
    i_lo: float = 0.0       # mA, measured: the lowest current (a sweep down ends above it where it goes dark)
    i_hi: float = 0.0       # mA, measured: the highest, never exceeded (0: not given yet: no map)
    i_step: float = 0.2     # mA
    v_lo: float = 0.0       # V: the piezo (its scan offset, as a rule)
    v_hi: float = 0.0
    v_step: float = 1.0
    sweep: str = "current"  # swept up and back down at each step of the other: current | piezo | both (a map each)
    t_lo: float = 0.0       # °C: the temperature too, from t_lo to t_hi, a whole map at each...
    t_hi: float = 0.0
    t_step: float = 0.0     # ...in steps of this (0: where it is, never moved)
    t_tol: float = 0.01     # K: settled once within this of where it is set...
    t_hold: float = 10.0    # s: ...for this long...
    t_wait: float = 300.0   # s: ...or after this long at most (mapped all the same, and said)
    settle: float = 0.05    # s after a step before readings count
    reads: int = 2          # readings at each point
    dark: int = 3           # points in a row without lasing end a sweep down the current (0: never, down to i_lo)
    patterns: str = "hops"  # the wavemeter's interference patterns kept: all | hops (around events, and...) | off
    every: int = 10         # ...at every this many points besides (0: not)
    record: str = ""        # more DLC pro parameters read at each point: names below laserN:, comma-separated
    repeat: bool = True     # the first line swept again at the end (how far things moved meanwhile)
    model: bool = True      # the laser's model fitted to it at the end (lasermodel.py)
    ff_end: str = "back"    # its feedforward at the end: back (as it was) | fitted (the ideal one the map finds)
    folder: str = ""        # where the file goes ('': ~/laser-maps)

    @classmethod
    def from_dict(cls, d) -> "MapPlan":
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

    def kinds(self) -> list:
        """The maps, by what each sweeps: ['current'], ['piezo'] or both."""
        return {"piezo": ["piezo"], "both": ["current", "piezo"]}.get(self.sweep, ["current"])

    def grid(self, axis: str) -> list:
        if axis == "current":
            return grid(self.i_lo, self.i_hi, self.i_step)
        if axis == "piezo":
            return grid(self.v_lo, self.v_hi, self.v_step)
        return grid(self.t_lo, self.t_hi, self.t_step) if self.t_step > 0 else [None]

    def temps(self) -> list:
        """The temperatures (°C) a map is made at; [None]: where it is."""
        return self.grid("temp")

    def names(self) -> list:
        """The parameters `record` names."""
        return [n.strip() for n in self.record.replace(";", ",").split(",") if n.strip()]

    def points(self) -> int:
        """How many points it takes, at most (going dark cuts sweeps down the current short)."""
        ni, nv = len(self.grid("current")), len(self.grid("piezo"))
        one = sum(2 * ni * nv for _ in self.kinds())
        rep = 2 * (ni if self.kinds()[-1] == "current" else nv) if self.repeat else 0
        return len(self.temps()) * one + rep

    def estimate(self, rate: float, pattern_bytes: float = 0.0) -> tuple:
        """(s, bytes) it takes, roughly: readings at `rate` a second for the laser, and patterns of pattern_bytes
        each, compressed, where they are kept (an estimate: where it is dim, the wavemeter is slower)."""
        n = self.points()
        per = max(self.settle, 0.0) + 0.012 + (self.reads + 1) / max(rate, 0.1)
        s = n * per + (len(self.temps()) * min(self.t_wait, 3 * self.t_hold + 30) if self.t_step > 0 else 0)
        kept = {"all": 1.0, "off": 0.0}.get(self.patterns, min(0.5 + (1.0 / self.every if self.every > 0 else 0.0),
                                                                 1.0))  # (an ECDL in fine steps: about half)
        b = n * (180 + 40 * self.reads + kept * pattern_bytes) + 50_000
        return s, b

    def problems(self, limits: dict | None = None) -> list:
        """What is wrong with the plan, or with it on that laser (limits: what take_over reads of it: clip, offset
        (measured minus set current), ff, v0, pc, tc), as sentences; [] if nothing."""
        out = []
        if not self.i_hi > 0:
            out.append("give the highest current it may have (mA, measured)")
        elif not self.i_lo < self.i_hi:
            out.append("the lowest current must be below the highest")
        if self.i_lo < 0:
            out.append("the lowest current must not be negative")
        if not 0 < self.i_step <= STEP_I:
            out.append(f"the current's step must be more than 0 and {STEP_I:g} mA at most")
        if not self.v_lo < self.v_hi:
            out.append("the lowest piezo voltage must be below the highest")
        if not 0 < self.v_step <= STEP_V:
            out.append(f"the piezo's step must be more than 0 and {STEP_V:g} V at most")
        if self.sweep not in ("current", "piezo", "both"):
            out.append(f"sweep what? {self.sweep!r}")
        if self.patterns not in ("all", "hops", "off"):
            out.append(f"which patterns? {self.patterns!r}")
        if self.ff_end not in ("back", "fitted"):
            out.append(f"its feedforward at the end: {self.ff_end!r}?")
        if self.t_step > 0 and not self.t_lo < self.t_hi:
            out.append("the lowest temperature must be below the highest")
        if self.t_step < 0 or self.t_tol <= 0 or self.t_hold < 0 or self.t_wait <= 0:
            out.append("the temperature's step, tolerance and times must not be negative")
        if not 1 <= self.reads <= 50:
            out.append("1 to 50 readings at each point")
        if not 0 <= self.settle <= 60:
            out.append("the settling time: 0 to 60 s")
        if self.dark < 0 or self.every < 0:
            out.append("the dark points and every-so-many must not be negative")
        if self.points() > MAX_POINTS:
            out.append(f"{self.points():,} points: more than {MAX_POINTS:,}: larger steps")
        if not limits or out:
            return out
        clip, off, ff, v0 = limits["clip"], limits["offset"], limits["ff"], limits["v0"]
        pc_lo, pc_hi = limits.get("pc") or (-math.inf, math.inf)
        if self.v_lo < pc_lo - 1e-9 or self.v_hi > pc_hi + 1e-9:
            out.append(f"the piezo {self.v_lo:g} … {self.v_hi:g} V: beyond its limits, {pc_lo:g} … {pc_hi:g} V")
        top = max(self.i_hi - off - ff * (v - v0) for v in (self.v_lo, self.v_hi))
        if top > clip + 1e-9:
            out.append(f"{self.i_hi:g} mA measured needs {top:.2f} mA set, above its current-clip ({clip:.2f} mA)")
        low = min(self.i_lo - off - ff * (v - v0) for v in (self.v_lo, self.v_hi))
        if low < 0:
            out.append(f"{self.i_lo:g} mA measured needs {low:.2f} mA set: below 0")
        if self.t_step > 0:
            tc_lo, tc_hi = limits.get("tc") or (-math.inf, math.inf)
            if limits.get("t0") is None:
                out.append("its temperature cannot be read (dl:tc:temp-set): no temperature steps")
            elif self.t_lo < tc_lo - 1e-9 or self.t_hi > tc_hi + 1e-9:
                out.append(f"the temperature {self.t_lo:g} … {self.t_hi:g} °C: beyond its limits, {tc_lo:g} … "
                           f"{tc_hi:g} °C")
        return out


# ------------------------------------------------------------------ patterns
_U32 = "I" if array("I").itemsize == 4 else "L"
_BITS = {"H": 16, _U32: 32}


def pack(values, dtype: str) -> tuple:
    """(codec, bytes): an interference pattern, losslessly: unsigned integers ('H', 'I') as differences from pixel to
    pixel (mod 2**bits: smooth fringes give small ones), then zlib; anything else ('d') zlib'd as it is."""
    dtype = _U32 if dtype in ("I", "L") else dtype
    a = values if isinstance(values, array) and values.typecode == dtype else array(dtype, values)
    codec = "z"
    if dtype in _BITS and len(a):
        mask = (1 << _BITS[dtype]) - 1
        a = array(dtype, [(x - p) & mask for x, p in zip(a, [0] + a[:-1].tolist())])
        codec = "dz"
    if sys.byteorder != "little":
        a = array(dtype, a)
        a.byteswap()
    return codec, zlib.compress(a.tobytes(), 6)


def unpack(codec: str, dtype: str, blob: bytes) -> list:
    """The pattern pack() packed."""
    dtype = _U32 if dtype in ("I", "L") else dtype
    a = array(dtype)
    a.frombytes(zlib.decompress(blob))
    if sys.byteorder != "little":
        a.byteswap()
    if codec == "dz":
        mask, total, out = (1 << _BITS[dtype]) - 1, 0, []
        for d in a:
            total = (total + d) & mask
            out.append(total)
        return out
    return a.tolist()


def contrast(amps) -> float | None:
    """The fringes' contrast on the wavemeter's first array, from its amplitudes (min1, max1, ...): (max - min) /
    (max + min); None: not known."""
    if not amps or len(amps) < 2 or not all(_num(a) for a in amps[:2]) or amps[1] + amps[0] <= 0:
        return None
    return (amps[1] - amps[0]) / (amps[1] + amps[0])


class Leg:
    """One sweep one way, point by point: where it hops, goes dark or lases again, where its readings spread and its
    fringes fade (a second mode beginning), as they come: what decides which patterns are kept."""

    HOP_MIN = 300.0  # MHz: a step from point to point smaller than this is never a hop...
    HOP_TIMES = 6.0  # ...nor one smaller than this many times the sweep's usual step
    SPREAD = 50.0    # MHz: a point's readings spread more than this: on two modes, likely
    FADE = 0.85      # fringe contrast below this fraction of the sweep's usual: a second mode, likely

    def __init__(self, steps=()):
        """steps: MHz from point to point of the sweep before, the same axis swept: the usual step from the
        start."""
        self.f = None          # THz at the last point that had a frequency
        self.state = None      # the last point: lasing, dark or bad (nothing valid, yet not dark)
        self.steps = deque(steps, maxlen=40)
        self.contrasts = deque(maxlen=40)
        self.hops = 0

    def add(self, f, dark: bool, spread, fringe) -> str:
        """A point (f: THz or None; spread: MHz or None; fringe: its contrast or None): '' or what happened there:
        hop, dark, lasing (again), invalid (nothing valid, yet light), spread, fade."""
        event = ""
        state = "dark" if dark else "lasing" if f is not None else "bad"
        if self.state is not None and state != self.state:
            event = {"dark": "dark", "lasing": "lasing", "bad": "invalid"}[state]
        self.state = state
        if f is not None:
            if self.f is not None:
                step = abs(f - self.f) * 1e6
                usual = statistics.median(self.steps) if len(self.steps) >= 3 else 0.0
                if step > max(self.HOP_MIN, self.HOP_TIMES * usual):
                    event = event or "hop"
                    self.hops += 1
                else:
                    self.steps.append(step)
            self.f = f
        if not event and spread is not None and spread > self.SPREAD:
            event = "spread"
        if fringe is not None:
            if not event and len(self.contrasts) >= 3 and fringe < self.FADE * statistics.median(self.contrasts):
                event = "fade"
            self.contrasts.append(fringe)
        return event


# ---------------------------------------------------------------------- file
SCHEMA = """
CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT);
CREATE TABLE lines (id INTEGER PRIMARY KEY, map INTEGER, axis TEXT, line INTEGER, slow REAL, temp REAL,
                    repeat INTEGER, t0 REAL, t1 REAL);
CREATE TABLE points (id INTEGER PRIMARY KEY, line_id INTEGER, leg INTEGER, k INTEGER, t REAL,
                     i REAL, i_set REAL, i_act REAL, v REAL, v_act REAL, temp REAL,
                     f REAL, f_lo REAL, f_hi REAL, n INTEGER, bad INTEGER, status TEXT, how TEXT, wait REAL,
                     power REAL, exp1 REAL, exp2 REAL,
                     amin1 REAL, amax1 REAL, aavg1 REAL, amin2 REAL, amax2 REAL, aavg2 REAL,
                     vals TEXT, event TEXT);
CREATE TABLE readings (point_id INTEGER, t REAL, f REAL, status TEXT, power REAL);
CREATE TABLE patterns (point_id INTEGER, kind INTEGER, dtype TEXT, n INTEGER, codec TEXT, data BLOB);
CREATE TABLE events (t REAL, kind TEXT, text TEXT);
CREATE TABLE temps (t REAL, set_c REAL, act_c REAL);
CREATE INDEX points_by_line ON points (line_id, leg, k);
CREATE INDEX readings_by_point ON readings (point_id);
CREATE INDEX patterns_by_point ON patterns (point_id);
"""
POINT_COLUMNS = ("id", "line_id", "leg", "k", "t", "i", "i_set", "i_act", "v", "v_act", "temp", "f", "f_lo", "f_hi",
                 "n", "bad",
                 "status", "how", "wait", "power", "exp1", "exp2", "amin1", "amax1", "aavg1", "amin2", "amax2",
                 "aavg2", "vals", "event")


class MapFile:
    """A map's file (SQLite, the schema above), written by a thread of its own as the map goes: nothing here waits
    for the disk, and a map cut short keeps what it had. error: why it could not be written ('' if it could)."""

    COMMIT_S = 1.0

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            raise FileExistsError(f"{self.path} exists")
        self.error = ""
        self.points = self.patterns = 0
        self.pattern_bytes = 0  # compressed, so far
        self._ids = {"points": 0, "lines": 0}
        self._q = queue.Queue()
        self._db = sqlite3.connect(str(self.path), check_same_thread=False)
        self._db.executescript(SCHEMA)
        self._db.commit()
        self._thread = threading.Thread(target=self._run, daemon=True, name="map file")
        self._thread.start()

    def _next(self, table: str) -> int:
        self._ids[table] += 1
        return self._ids[table]

    def meta(self, key: str, value):
        self._q.put(("INSERT OR REPLACE INTO meta VALUES (?, ?)", (key, json.dumps(value, default=str))))

    def line(self, map_n: int, axis: str, line: int, slow: float, temp, repeat: bool, t0: float) -> int:
        n = self._next("lines")
        self._q.put(("INSERT INTO lines VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                     (n, map_n, axis, line, slow, temp, int(repeat), t0, None)))
        return n

    def line_over(self, line_id: int, t1: float):
        self._q.put(("UPDATE lines SET t1 = ? WHERE id = ?", (t1, line_id)))

    def point(self, row: dict) -> int:
        """A point (POINT_COLUMNS, but its id): its id."""
        n = self._next("points")
        self.points += 1
        self._q.put((f"INSERT INTO points VALUES ({', '.join('?' * len(POINT_COLUMNS))})",
                     tuple([n] + [row.get(c) for c in POINT_COLUMNS[1:]])))
        return n

    def readings(self, point_id: int, rows):
        """[(t, THz or None, status, power or None)]"""
        self._q.put(("INSERT INTO readings VALUES (?, ?, ?, ?, ?)", [(point_id, *r) for r in rows]))

    def pattern(self, point_id: int, kind: int, dtype: str, values):
        """A pattern of that point (packed, losslessly, in the file's thread)."""
        self.patterns += 1
        self._q.put(("pattern", (point_id, kind, dtype, values)))

    def event(self, t: float, kind: str, text: str):
        self._q.put(("INSERT INTO events VALUES (?, ?, ?)", (t, kind, text)))

    def temp(self, t: float, set_c, act_c):
        self._q.put(("INSERT INTO temps VALUES (?, ?, ?)", (t, set_c, act_c)))

    def close(self, timeout: float = 30.0):
        """Everything written, and the file closed."""
        if self._thread.is_alive():
            self._q.put(None)
            self._thread.join(timeout)

    def _run(self):
        db, last = self._db, time.monotonic()
        try:
            while True:
                try:
                    item = self._q.get(timeout=self.COMMIT_S)
                except queue.Empty:
                    item = ()
                if item is None:
                    break
                if item and not self.error:
                    sql, args = item
                    if sql == "pattern":
                        point_id, kind, dtype, values = args
                        codec, blob = pack(values, dtype)
                        self.pattern_bytes += len(blob)
                        sql = "INSERT INTO patterns VALUES (?, ?, ?, ?, ?, ?)"
                        args = (point_id, kind, "I" if dtype in ("I", "L") else dtype, len(values), codec, blob)
                    try:
                        if isinstance(args, list):
                            db.executemany(sql, args)
                        else:
                            db.execute(sql, args)
                    except sqlite3.Error as e:  # (the disk full, say: the map goes on, and says so)
                        self.error = str(e) or type(e).__name__
                        log.error("map file %s: %s", self.path, self.error)
                if time.monotonic() - last >= self.COMMIT_S and not self.error:
                    try:
                        db.commit()
                    except sqlite3.Error as e:
                        self.error = str(e) or type(e).__name__
                    last = time.monotonic()
        finally:
            try:
                db.commit()
            except sqlite3.Error as e:
                self.error = self.error or str(e)
            db.close()

    @property
    def size(self) -> int:
        """Bytes on the disk now."""
        try:
            return self.path.stat().st_size
        except OSError:
            return 0


def file_name(channel: int, name: str = "", when: float | None = None) -> str:
    """A map's file name: when, which channel, which laser."""
    stamp = time.strftime("%Y-%m-%d_%H%M%S", time.localtime(when))
    tag = "".join(c if c.isalnum() or c in "-." else "-" for c in name).strip("-")
    return f"map_{stamp}_ch{channel}" + (f"_{tag}" if tag else "") + ".sqlite"


class LaserMap:
    """A map's file, read back: meta (dict), lines, points and events (lists of dicts, in order); readings() and
    patterns() of a point; legs() the points sweep by sweep."""

    def __init__(self, path):
        self.path = Path(path)
        self.db = sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True)
        self.db.row_factory = sqlite3.Row
        self.meta = {}
        for key, value in self.db.execute("SELECT key, value FROM meta"):
            try:
                self.meta[key] = json.loads(value)
            except ValueError:
                self.meta[key] = value
        self.lines = [dict(r) for r in self.db.execute("SELECT * FROM lines ORDER BY id")]
        self.points = [dict(r) for r in self.db.execute("SELECT * FROM points ORDER BY id")]
        for p in self.points:
            p["vals"] = json.loads(p["vals"]) if p["vals"] else {}
        self.events = [dict(r) for r in self.db.execute("SELECT * FROM events ORDER BY rowid")]
        self.temps = [dict(r) for r in self.db.execute("SELECT * FROM temps ORDER BY rowid")]

    @property
    def plan(self) -> MapPlan:
        return MapPlan.from_dict(self.meta.get("plan") or {})

    def readings(self, point_id: int) -> list:
        return [dict(r) for r in self.db.execute("SELECT t, f, status, power FROM readings WHERE point_id = ? "
                                                 "ORDER BY rowid", (point_id,))]

    def patterns(self, point_id: int) -> dict:
        """kind -> the pattern (a list), of that point (PATTERN_KINDS)."""
        return {r["kind"]: unpack(r["codec"], r["dtype"], r["data"]) for r in
                self.db.execute("SELECT kind, dtype, codec, data FROM patterns WHERE point_id = ?", (point_id,))}

    def views(self) -> list:
        """The maps as the window had them (MapView: the frequency from the first that lased), to look at or fit
        (fit_feedforward) as the map did."""
        p, out, by_id = self.plan, [], {}
        pd = (self.meta.get("recorded") or {}).get("photodiode") or ""
        f_ref = next((q["f"] for q in self.points if q["f"] is not None), None)
        for line in self.lines:
            n = line["map"]
            if n >= len(out):
                fast, slow = ("current", "piezo") if line["axis"] == "current" else ("piezo", "current")
                sv = p.grid(slow)
                out.append(MapView(n, line["axis"], line["temp"], p.grid(fast), sv[:1] if line["repeat"] else sv,
                                   bool(line["repeat"])))
            by_id[line["id"]] = (out[n], line["line"])
        for q in self.points:
            if q["line_id"] not in by_id:
                continue
            view, j = by_id[q["line_id"]]
            if not 0 <= q["k"] < len(view.fv) or not 0 <= j < len(view.sv):
                continue
            dark = q["f"] is None and (q["how"] in ("dark", "quiet") or q["status"] in DARK)
            ghz = (q["f"] - f_ref) * 1e3 if q["f"] is not None else MapView.DARK if dark else MapView.BAD
            power = q["power"] if q["power"] is not None else MapView.DARK if dark else MapView.BAD
            x = q["vals"].get(pd) if pd else None
            view.put(q["leg"], j, q["k"], ghz, power, x if _num(x) else MapView.BAD)
        return out

    def sweeps(self, current: str = "act") -> list:
        """Each map's sweeps, for fit_sweeps: [(map, kind, temp, {1: up, 2: down})], each sweep (the slow axis's
        value, the swept one's values, GHz from the first point that lased or DARK, BAD), the current as measured:
        'act' as read at each point (i_act), or where it was not read 'set': the set current and what the current
        was besides where it began (a file from before the current was read at each point, made with the feedforward
        on: that moved nothing as the piezo moved, not as the map then took it to); 'plan' as the map had it (i)."""
        start = self.meta.get("start") or {}
        off = (start.get("current_act") or 0.0) - (start.get("current_set") or 0.0)

        def amps(q):
            if current == "act" and q.get("i_act") is not None:
                return q["i_act"]
            if current in ("act", "set") and q.get("i_set") is not None:
                return q["i_set"] + off
            return q["i"]

        f_ref = next((q["f"] for q in self.points if q["f"] is not None), None)
        maps = {}
        for line, leg, pts in self.legs():
            if line.get("repeat") or leg not in (1, 2):
                continue
            n, kind = line["map"], line["axis"]
            key = (n, kind, line["temp"])
            if kind == "current":
                xs, y = [amps(q) for q in pts], line["slow"]
            else:
                xs, y = [q["v"] for q in pts], statistics.median([amps(q) for q in pts])
            gs = [(q["f"] - f_ref) * 1e3 if q["f"] is not None else MapView.DARK if q["how"] in ("dark", "quiet")
                  or q["status"] in DARK else MapView.BAD for q in pts]
            maps.setdefault(key, {1: [], 2: []})[leg].append((y, xs, gs))
        return [(n, kind, temp, sw) for (n, kind, temp), sw in sorted(maps.items(), key=lambda kv: kv[0][0])]

    def fits(self, current: str = "act") -> list:
        """fit_sweeps of each map (those it could), the current as `current` has it (sweeps)."""
        return [f for f in (fit_sweeps(sw, kind, n, temp) for n, kind, temp, sw in self.sweeps(current)) if f]

    def legs(self) -> list:
        """[(line, leg, [points])], sweep by sweep, in order."""
        by_line = {line["id"]: line for line in self.lines}
        out, key = [], None
        for p in self.points:
            k = (p["line_id"], p["leg"])
            if k != key:
                out.append((by_line.get(p["line_id"], {}), p["leg"], []))
                key = k
            out[-1][2].append(p)
        return out

    def close(self):
        self.db.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def load(path) -> LaserMap:
    return LaserMap(path)


# ------------------------------------------------------------------ requests
@dataclass
class Go:
    """Move there, in steps (at most step_i mA and step_v V a write), nothing read: the current (mA, measured), the
    piezo (V), the temperature (°C, set: not waited for); None: as it is. With the feedforward on, the measured
    current stays as the map says while the piezo moves."""
    current: float | None = None
    piezo: float | None = None
    temp: float | None = None
    step_i: float = RAMP_I
    step_v: float = RAMP_V


@dataclass
class Sample:
    """A point: `settle` s after the last write, then readings that began after that, `reads` valid ones (or what the
    wavemeter gives in its time: MapRunner), and the parameters `names` read meanwhile. dark_pd (name, level): that
    parameter below level says the laser does not lase, and the wavemeter is not waited for."""
    reads: int = 1
    settle: float = 0.0
    names: tuple = ()
    dark_pd: tuple | None = None


@dataclass
class Taken:
    """What a Sample got: readings [(t, THz or NaN, status, extra or None)], in order; values {name: value or None};
    t0 when readings began to count, t1 when it was over (monotonic); how it ended: done | dark (the photodiode said
    so) | quiet (the wavemeter said there is no light, and nothing new since, in the time a reading takes) |
    invalid | timeout; early: readings passed over, begun before the step; status: the last thing the wavemeter
    said of the laser ('': valid), those passed over too."""
    readings: list = field(default_factory=list)
    values: dict = field(default_factory=dict)
    t0: float = 0.0
    t1: float = 0.0
    how: str = ""
    early: int = 0
    status: str = ""


# ----------------------------------------------------------------- the window's
class MapView:
    """One map as it fills, for the window: at each point, each way (up: leg 1; down: legs 2 and 0), three layers:
    'f' the frequency in GHz from where the laser began (DARK where it did not lase, BAD where the wavemeter said
    nothing valid), 'power' the power at the wavemeter in µW (DARK where it did not lase and the wavemeter gave none,
    BAD where it gave none), 'pd' its photodiode (BAD: not read); None where not measured yet. up and down: the
    frequency's. repeat: the first line of the map before it, swept again."""

    DARK = -math.inf
    BAD = math.nan
    LAYERS = ("f", "power", "pd")

    def __init__(self, n: int, kind: str, temp, fv: list, sv: list, repeat: bool = False):
        self.n, self.kind, self.temp, self.repeat = n, kind, temp, repeat
        self.fast, self.slow = ("current", "piezo") if kind == "current" else ("piezo", "current")
        self.fv, self.sv = fv, sv
        self.layers = {name: ([[None] * len(fv) for _ in sv], [[None] * len(fv) for _ in sv]) for name in self.LAYERS}
        self.up, self.down = self.layers["f"]
        self.version = 0  # bumped at every point...
        self.up_v = [0] * len(sv)  # ...and its line's, each way
        self.down_v = [0] * len(sv)

    def grid(self, layer: str, up: bool) -> list:
        return self.layers[layer][0 if up else 1]

    def put(self, leg: int, j: int, k: int, value, power=None, pd=None):
        w = 0 if leg == 1 else 1
        for name, x in (("f", value), ("power", power), ("pd", pd)):
            self.layers[name][w][j][k] = x
        (self.up_v if leg == 1 else self.down_v)[j] += 1
        self.version += 1

    def turn(self, j: int, k: int):
        """Where line j turns (point k): the same both ways."""
        self.put(2, j, k, *(self.layers[name][0][j][k] for name in self.LAYERS))


# ----------------------------------------------------------- the feedforward
def _runs(xs: list, gs: list, bridge: int = 2) -> list:
    """A sweep's stretches of lasing, in its order: [[(x, GHz)]], across `bridge` points in a row at most that read
    nothing valid (two modes at once, say: not dark), broken where it is dark or not measured."""
    out, run, gap = [], [], 0
    for x, g in zip(xs, gs):
        if g is None or g == -math.inf:
            if run:
                out.append(run)
            run, gap = [], 0
        elif g != g:
            gap += 1
            if gap > bridge and run:
                out.append(run)
                run = []
        else:
            run.append((x, g))
            gap = 0
    if run:
        out.append(run)
    return out


def sweep_hops(xs: list, gs: list, jump: float = 1.0, rate: float = 0.0) -> list:
    """Where a sweep hops: [(x, sign, GHz before, GHz after)] in its order, x half way between the last point before
    and the first after: a step of more than `jump` GHz that holds (two points at least either side, each two within
    `jump` of each other, their medians as far apart: a reading that is nonsense once, two modes at once, is no
    hop), how it tunes on a mode (rate, GHz per unit of x) taken off first (across points that read nothing valid,
    it tunes on): GHz before and after, less rate times x, so that one mode reads the same wherever along it."""
    out = []
    for run in _runs(xs, gs):
        r = [g - rate * x for x, g in run]
        for k in range(2, len(run) - 1):
            step = r[k] - r[k - 1]
            if abs(step) <= jump:
                continue
            before, after = r[max(k - 3, 0):k], r[k:k + 3]
            if abs(before[-1] - before[-2]) > jump or abs(after[1] - after[0]) > jump:
                continue
            b, a = statistics.median(before), statistics.median(after)
            if abs(a - b) > jump and (a - b) * step > 0:
                out.append(((run[k - 1][0] + run[k][0]) / 2, 1 if step > 0 else -1, b, a))
    return out


def fit_feedforward(view: MapView, jump: float | None = None) -> dict | None:
    """The ideal feedforward from a map as the window has it (fit_sweeps)."""
    sweeps = {}
    for leg, up in ((1, True), (2, False)):
        order = list(range(len(view.fv))) if up else list(range(len(view.fv) - 1, -1, -1))
        sweeps[leg] = [(view.sv[j], [view.fv[k] for k in order], [row[k] for k in order])
                       for j, row in enumerate(view.grid("f", up))]
    return fit_sweeps(sweeps, view.kind, view.n, view.temp, jump)


def fit_sweeps(sweeps: dict, kind: str, n: int = 0, temp=None, jump: float | None = None) -> dict | None:
    """The ideal feedforward from a map's hops (None: too few to tell). sweeps: {1: up, 2: down}, each [(the slow
    axis's value, the swept one's values in the sweep's order, GHz or DARK, BAD, None)], line by line; kind: what
    is swept ('current': mA across, V from line to line; 'piezo': the other way round).

    The hops of each sweep (sweep_hops: steps of more than `jump` GHz, how it tunes on a mode taken off; None: 1 GHz,
    or three times the usual step if more), followed from line to line: the nearest on the next line that hops the
    same way by about as much (a laser may hop by one mode of its diode chip, or by two: each its own) between the
    same two modes (their frequencies, its tuning taken off, as far on as one mode goes from line to line), each the
    other's nearest, no farther than the usual distance between such hops. Its hops repeat along a sweep; by where
    they are alone, a boundary moving by that distance from line to line would look the same (aliased: alias below):
    the modes either side tell one boundary from another. The slope of each boundary so followed over three lines
    at least, pooled (each about its own middle, then once more without the points far off it).

    A dict: ff (mA/V) and err (its standard error), hops and boundaries it rests on; spacing (the usual distance
    between hops along the swept axis, mA or V); hysteresis (how much later it hops sweeping up than back sweeping
    down, the same two modes: mA or V) and pairs; df_di (GHz/mA) and df_dv (GHz/V), how it tunes on a mode; tuning
    (GHz/V) along the ideal feedforward; jump (GHz); alias, how much more or less a boundary may move from line to
    line and look the same by where it is alone (the widest distance between hops of a kind over the slow axis's
    step: mA/V, or V/mA with the piezo swept); shift, how far a boundary moves from line to line (mA or V)."""
    fast_i = kind == "current"
    pairs_ = [((b - a), (x2 - x1)) for leg in sweeps for _, xs, gs in sweeps[leg]
              for x1, x2, a, b in zip(xs, xs[1:], gs, gs[1:]) if _num(a) and _num(b) and x2 != x1]
    if not pairs_:
        return None
    rate = statistics.median([dg / dx for dg, dx in pairs_])  # GHz per unit: on a mode, as a rule
    if jump is None:
        jump = max(1.0, 3.0 * statistics.median([abs(dg - rate * dx) for dg, dx in pairs_]))
    step = statistics.median([abs(dx) for _, dx in pairs_])
    lines = {leg: [(y, sweep_hops(xs, gs, jump, rate)) for y, xs, gs in sweeps[leg]] for leg in sweeps}
    sizes = [abs(h[3] - h[2]) for leg in lines for _, hops in lines[leg] for h in hops]
    same = 0.4 * statistics.median(sizes) if sizes else 3 * jump  # GHz: one mode to the next, at least
    tol = 3 * jump  # GHz: one mode, its tuning taken off (a mode a cycle of hops later is another)
    spacing = {}  # sign -> the usual distance between such hops along a sweep: between two in a sweep if there are
    for sign in (1, -1):  # enough of those, else the length swept lasing over how many there were
        gaps = [abs(b[0] - a[0]) for leg in lines for _, hops in lines[leg]
                for a, b in zip([h for h in hops if h[1] == sign], [h for h in hops if h[1] == sign][1:])]
        if len(gaps) >= max(5, 0.3 * sum(len(lines[leg]) for leg in lines)):
            spacing[sign] = statistics.median(gaps)
            continue
        each = []
        for leg in sweeps:
            extent = sum(abs(run[-1][0] - run[0][0]) for _, xs, gs in sweeps[leg] for run in _runs(xs, gs))
            count = sum(1 for _, hops in lines[leg] for h in hops if h[1] == sign)
            if count:
                each.append(extent / count)
        if each:
            spacing[sign] = min(each)

    rates_fast, rates_slow = [], []
    for leg in sweeps:
        for _y, xs, gs in sweeps[leg]:
            for run in _runs(xs, gs, bridge=0):
                rates_fast += [(g2 - g1) / (x2 - x1) for (x1, g1), (x2, g2) in zip(run, run[1:])
                               if abs(g2 - g1) < jump / 2 and x2 != x1]
        rows = [(y, {round(x / step): g for x, g in zip(xs, gs) if _num(g)}) for y, xs, gs in sweeps[leg]] \
            if step > 0 else []
        for (y1, r1), (y2, r2) in zip(rows, rows[1:]):
            if y2 != y1:
                rates_slow += [(r2[k] - g) / (y2 - y1) for k, g in r1.items() if k in r2 and abs(r2[k] - g) < same]
    df_fast = statistics.median(rates_fast) if rates_fast else None
    df_slow = statistics.median(rates_slow) if rates_slow else 0.0  # GHz per slow unit: one mode, line to line
    reach = max(spacing.values()) if spacing else 0.0

    def alike(h, g):
        return h[1] == g[1] and abs((h[3] - h[2]) - (g[3] - g[2])) <= 0.35 * max(abs(h[3] - h[2]), abs(g[3] - g[2]))

    def follow(df_slow):
        """The boundaries, followed from line to line: [[(slow value, where, GHz before, GHz after)]]."""
        def same_two(h, g, dy):  # the same two modes (each as far on as the next line moves it): the same boundary
            drift = df_slow * dy
            return abs(g[2] - h[2] - drift) + abs(g[3] - h[3] - drift) <= 2 * tol

        chains = []
        for leg in lines:
            open_ = {}  # hop index on the line before -> its chain
            for j, (y, hops) in enumerate(lines[leg]):
                now = {}
                if j > 0:
                    y0, before = lines[leg][j - 1]
                    for k, h in enumerate(hops):
                        near = min(((abs(h[0] - g[0]), m) for m, g in enumerate(before)
                                    if alike(h, g) and same_two(g, h, y - y0)), default=None)
                        if near is None or near[0] > reach:
                            continue
                        back = min((abs(before[near[1]][0] - c[0]), q) for q, c in enumerate(hops)
                                   if alike(c, h) and same_two(before[near[1]], c, y - y0))
                        if back[1] == k and near[1] in open_:
                            now[k] = open_[near[1]]
                            now[k].append((y, h[0], h[2], h[3]))
                for k, h in enumerate(hops):
                    if k not in now:
                        now[k] = [(y, h[0], h[2], h[3])]
                        chains.append(now[k])
                open_ = now
        return chains

    def drift_of(chains):  # GHz per slow unit: how one mode moves from line to line, along its boundaries
        each = [(c2[2] - c1[2] + c2[3] - c1[3]) / 2 / (c2[0] - c1[0]) for c in chains for c1, c2 in zip(c, c[1:])
                if c2[0] != c1[0]]
        return statistics.median(each) if each else None

    dy_usual = statistics.median([abs(b[0] - a[0]) for leg in lines for a, b in zip(lines[leg], lines[leg][1:])
                                  if b[0] != a[0]] or [0.0])
    chains = follow(df_slow)
    along = drift_of(chains)
    if along is not None and abs(along - df_slow) * dy_usual > 0.5:  # (followed again, as far on as that)
        chains = follow(along)
        along = drift_of(chains) if drift_of(chains) is not None else along
    chains = [[(y, x) for y, x, _, _ in c] for c in chains]
    chains = [c for c in chains if len({y for y, _ in c}) >= 3]
    if len(chains) < 2 or sum(len(c) for c in chains) < 8:
        return None

    def pooled(points):
        syy = sum(dy * dy for dy, _ in points)
        return sum(dy * dx for dy, dx in points) / syy if syy > 0 else None

    def centred(c):
        my, mx = sum(y for y, _ in c) / len(c), sum(x for _, x in c) / len(c)
        return [(y - my, x - mx) for y, x in c]

    pts = [q for c in chains for q in centred(c)]
    slope = pooled(pts)
    if slope is None:
        return None
    res = [dx - slope * dy for dy, dx in pts]
    sigma = 1.4826 * statistics.median([abs(r) for r in res])
    keep = [q for q, r in zip(pts, res) if abs(r) <= max(3 * sigma, step)]
    if 8 <= len(keep) < len(pts):
        slope = pooled(keep) or slope
        pts = keep
    res = [dx - slope * dy for dy, dx in pts]
    syy = sum(dy * dy for dy, _ in pts)
    se = math.sqrt(sum(r * r for r in res) / max(len(pts) - len(chains) - 1, 1) / syy) if syy > 0 else math.inf
    if fast_i:  # dI/dV along a boundary: the feedforward
        ff, err = slope, se
    elif abs(slope) > 1e-9:  # dV/dI
        ff, err = 1.0 / slope, se / slope ** 2
    else:
        return None
    pairs = []  # the same two modes: up, a hop off one; down, the hop back onto it
    for (_y, ups), (_, downs) in zip(lines.get(1, []), lines.get(2, [])):
        for x, sign, before, after in ups:
            near = [(abs(x - xd), x - xd) for xd, sd, bd, ad in downs if sd == -sign and abs(bd - after) < tol
                    and abs(ad - before) < tol]
            if near and min(near)[0] <= spacing.get(sign, 0.0):
                pairs.append(min(near)[1])
    df_slow = along if along is not None else statistics.median(rates_slow) if rates_slow else None
    df_di, df_dv = (df_fast, df_slow) if fast_i else (df_slow, df_fast)
    usual = statistics.median(spacing.values()) if spacing else None
    dys = [abs(b[0] - a[0]) for leg in lines for a, b in zip(lines[leg], lines[leg][1:]) if b[0] != a[0]]
    alias = max(spacing.values()) / statistics.median(dys) if spacing and dys else None
    out = {"map": n, "kind": kind, "temp": temp, "ff": round(ff, 6), "err": round(err, 6), "hops": len(pts),
           "boundaries": len(chains), "spacing": round(usual, 4) if usual else None,
           "hysteresis": round(statistics.median(pairs), 4) if pairs else None, "pairs": len(pairs),
           "df_di": round(df_di, 5) if df_di is not None else None,
           "df_dv": round(df_dv, 5) if df_dv is not None else None, "jump": round(jump, 3),
           "alias": round(alias, 4) if alias else None,
           "shift": round(abs(slope) * statistics.median(dys), 4) if dys else None}
    out["tuning"] = round(df_dv + df_di * ff, 5) if df_di is not None and df_dv is not None else None
    return out


def say_fit(fit: dict) -> str:
    """A fit (fit_feedforward), said."""
    unit = "mA" if fit["kind"] == "current" else "V"
    out = (f"the ideal feedforward {fit['ff']:+.4f} ± {fit['err']:.4f} mA/V (from {fit['hops']} hops along "
           f"{fit['boundaries']} mode boundaries")
    out += f"; with it the laser tunes {fit['tuning']:+.3f} GHz/V on one mode)" if fit.get("tuning") is not None \
        else ")"
    if fit.get("hysteresis") is not None:
        out += f"; hysteresis {fit['hysteresis']:.3f} {unit} ({fit['pairs']} pairs)"
    if fit.get("spacing"):
        out += f"; a hop every {fit['spacing']:.3f} {unit}"
    if fit.get("shift") and fit.get("spacing") and fit["shift"] > 0.4 * fit["spacing"]:
        slow = "piezo" if fit["kind"] == "current" else "current"
        out += (f" (its boundaries move {fit['shift']:.2f} {unit} from line to line, nearly as far as from one hop to "
                f"the next: told apart by the modes either side; smaller steps of the {slow} to be sure)")
    return out


# ----------------------------------------------------------------- procedure
class Mapper:
    """The map (run(): a generator, driven by MapRunner): see the module's docstring. out: the file (None: none);
    info: what to keep with it about the laser ('laser': its channel, name, controller...) and the wavemeter
    ('wavemeter')."""

    AROUND = 3      # points whose patterns are kept around an event, each side (patterns 'hops')
    PD_KEEP = 200   # photodiode values remembered, dark and lasing (as the wavemeter told them), to tell them apart

    def __init__(self, plan: MapPlan, out: MapFile | None = None, info: dict | None = None):
        self.plan = plan
        self.out = out
        self.info = dict(info or {})
        self.stage = "starting"
        self.started = 0.0         # monotonic: when the map began (MapRunner sets it)
        # where it began (take_over), and what it is set to now (mA measured, V, °C set)
        self.i0 = self.a0 = self.v0 = None
        self.t0 = None
        self.I = self.V = None
        self.T = None
        self.act_offset = 0.0      # mA: measured minus set current (the piezo never moves while the feedforward adds)
        # its feedforward: as it began, and as it is (MapRunner says each write of it as it is acked: ff_switched)
        self.ff_was = False        # on where it began...
        self.ff0 = 0.0             # ...with this factor (mA/V; its factor even if off)
        self.ff_now = False        # on now...
        self.ff_factor = 0.0       # ...with this factor
        self.gap = None            # V: what it adds at rest, per mA/V of its factor (ff_learn; None: not known)...
        self.gap_u = None          # ...to within this
        self._steps = []           # (mA/V, mA): each step of its factor while on, and what it did to the current
        self.off_offset = None     # mA: measured minus set current with it adding nothing (off, or its factor 0)
        self.ff_applied = False    # the ideal one the map found is on (left on, should anything fail after)
        self.fits = []             # what fit_feedforward told of each map (those it could)
        self.ff_end = {}           # how its feedforward ended
        self.clip = math.inf       # mA set: the controller's current-clip
        self.pc = (-math.inf, math.inf)  # V: the piezo's limits
        self.tc = (-math.inf, math.inf)  # °C: the temperature's
        self.scan_was = False      # its scan was on (switched off for the map)
        self.pd = ""               # the DL photodiode's parameter ('': none)
        self.names = ()            # what is read at each point
        self.pd_dark = deque(maxlen=self.PD_KEEP)
        self.pd_lasing = deque(maxlen=self.PD_KEEP)
        self.f0 = None             # THz where it began...
        self.f_end = None          # ...and back there at the end
        # progress, and what the window shows
        self.total = plan.points()
        self.points = self.lasing = self.kept = 0
        self.hops = {1: 0, 2: 0}   # seen sweeping up, down
        self.notes = []            # [(s, text)]
        self.model = None          # the laser's model, fitted at the end (lasermodel.LaserModel; engine's thread)
        self.model_note = ""       # how that went
        self.views = []            # MapView, a map each
        self.view = None           # the one filling
        self.line_no = (0, 0)      # (line, of how many) of the map filling
        self.t_last = 0.0          # s: the last point's
        self.trace = []            # [(fast value, GHz, µW, photodiode: each None if not)] of the sweep going on...
        self.trace_before = []     # ...and the one before
        self.trace_legs = (None, None)  # (leg now, leg before): 1 up, 2 down, 0 a descent
        self.pattern_now = None    # the last pattern seen (kind 0, or the first there): (dtype, values)
        self.result = ""
        self._key = None           # (line id, leg) of the sweep going on...
        self._kind = None          # ...the axis it sweeps
        self._leg = Leg()
        self.f_ref = None          # THz the window's frequencies are from: where it began (or its first reading)
        self._ring = deque()       # (point id, patterns) not kept yet (patterns 'hops')
        self._after = 0            # points still to keep after an event

    # ---------------------------------------------------------------- helpers
    def now(self, t: float) -> float:
        """s since the map began, of a monotonic time."""
        return round(t - self.started, 3)

    def mA(self, i_set: float, v: float = 0.0) -> float:
        """A set current as measured (wherever the piezo: it never moves while the feedforward adds anything)."""
        return i_set + self.act_offset

    def set_for(self, i: float, v: float = 0.0) -> float:
        """The set current that has the measured current at i."""
        return i - self.act_offset

    def note(self, t, text: str, kind: str = "note"):
        s = self.now(t) if t else None
        self.notes.append((s, text))
        if self.out is not None:
            self.out.event(s, kind, text)
        log.info("map: %s", text)

    def limits(self) -> dict:
        return {"clip": self.clip, "offset": self.act_offset, "ff": 0.0, "v0": self.v0, "pc": self.pc,
                "tc": self.tc, "t0": self.t0}

    @property
    def progress(self) -> float:
        return min(self.points / self.total, 1.0) if self.total else 0.0

    def pd_level(self):
        """The photodiode's value below which the laser surely does not lase, once the wavemeter has told dark from
        lasing often enough (None until then, or while the two overlap): a quarter of the way from what it gives
        dark to what it gives lasing (the highest tenth of the one, the lowest of the other: a point the wavemeter
        misjudged does not move it)."""
        if not self.pd or len(self.pd_dark) < 3 or len(self.pd_lasing) < 3:
            return None
        dark = sorted(self.pd_dark)[(9 * len(self.pd_dark)) // 10]
        lasing = sorted(self.pd_lasing)[len(self.pd_lasing) // 10]
        return dark + 0.25 * (lasing - dark) if dark < lasing else None

    # ------------------------------------------------------- its feedforward
    # What a DLC pro's feedforward adds to the current at rest is its factor times a voltage: the piezo's from 0 V,
    # say, or nothing at all (a DL pro moved by its scan's offset, its scan off: its feedforward follows the scan
    # alone). Not known beforehand, it is learned as its factor is stepped (ff_learn): until then it could be the
    # factor times the piezo's distance from 0 V or from its farther limit (ff_most), tens of mA. So its factor is
    # taken to 0 and back in steps that cannot move the current more than FF_STEP each by what is known so far, the
    # current read after each (ff_ramp), and it is switched off and on only at a factor of 0, where that does nothing.

    def ff_most(self) -> float:
        """V: the most what the feedforward adds can be, per mA/V of its factor (the gap learned and its uncertainty;
        until then the piezo's distance from 0 V or from its farther limit, whichever is farthest)."""
        if self.gap is not None:
            return abs(self.gap) + self.gap_u
        return max(abs(self.v0 - x) for x in (0.0, *(x for x in self.pc if math.isfinite(x))))

    def ff_rise(self, change: float) -> float:
        """mA: the most a change of `change` mA/V in the feedforward's factor takes the current up (a step of it while
        it is on; switching it on: its factor; off: minus that)."""
        return max(self._high(change), 0.0)

    def _high(self, change: float) -> float:
        """mA: what a change of `change` mA/V in its feedforward's factor does to the current, at the most."""
        if self.gap is not None:
            return change * self.gap + abs(change) * self.gap_u
        return abs(change) * self.ff_most()

    def ff_learn(self, step: float, moved: float):
        """A step of its factor (mA/V) while on moved the current that much (mA): what it adds, learned (least squares
        over every step; uncertain by three times what the DLC pro's reading of its current scatters, FF_NOISE, over
        the steps taken so far, and by 2 % at least)."""
        self._steps.append((step, moved))
        sxx = sum(x * x for x, _ in self._steps)
        if sxx <= 0:
            return
        self.gap = sum(x * y for x, y in self._steps) / sxx
        self.gap_u = max(3 * math.sqrt(2) * FF_NOISE / math.sqrt(sxx), 0.02 * abs(self.gap))

    def ff_switched(self, name: str, value):
        """Its DLC pro took a write of its feedforward (MapRunner says so as it is acked, whatever the procedure is
        doing): what its current is reckoned from follows, as high as it could be until it is read; back as it began,
        as it began exactly."""
        if name == FF:
            if self.ff_now:
                self.act_offset += self._high(value - self.ff_factor)
            self.ff_factor = value
        elif value != self.ff_now:
            self.ff_now = value
            self.act_offset += self._high(self.ff_factor if value else -self.ff_factor)
        if self.ff_now and self.ff_was and self.ff_factor == self.ff0:
            self.act_offset = self.a0 - self.i0
        elif not self.ff_now or self.ff_factor == 0.0:
            if self.off_offset is not None:
                self.act_offset = self.off_offset

    def ff_ramp(self, target: float):
        """Its factor taken to `target` while it is on (the piezo where it began), in steps that cannot move the
        current more than FF_STEP each by what is known of what it adds, the current read after each (what it adds
        learned meanwhile), and kept no higher than where it began (lowered first where a step could take it
        higher)."""
        for _ in range(400):
            want = target - self.ff_factor
            if abs(want) < 1e-9:
                if self.ff_now and self.ff_was and self.ff_factor == self.ff0:
                    self.act_offset = self.a0 - self.i0  # (as it began: back to its set current exactly)
                return
            most = self.ff_most()
            step = want if abs(want) * most <= FF_STEP else math.copysign(FF_STEP / most, want)
            to = target if abs(want - step) < 1e-9 else round(self.ff_factor + step, 6)  # (as the DLC pro takes it)
            step = to - self.ff_factor
            rise = self.ff_rise(step)
            if self.I + rise > self.a0 + 1e-9:
                yield from self.go(current=max(self.a0 - rise, self.mA(0.0)))
            before = self.act_offset
            yield Set(FF, to)
            yield Wait(FF_SETTLE)
            i_set, act = yield Get((CURRENT, CURRENT_ACT))
            if not (_num(i_set) and _num(act)):
                raise Failed("cannot read its current while its feedforward is stepped")
            self.ff_learn(step, act - i_set - before)
            self.act_offset = act - i_set
            self.I = act
            if self.ff_factor == 0.0:
                self.off_offset = self.act_offset
        raise Failed(f"its feedforward did not come to {target:+.4f} mA/V in 400 steps")

    def fitted(self):
        """The ideal feedforward the maps found (fit_feedforward), where it began (its temperature): their
        estimates weighed by how well each tells it; None if none could."""
        fits = [f for f in self.fits if f["temp"] is None or self.t0 is None or abs(f["temp"] - self.t0) < 1e-6]
        if not fits and self.fits:  # (stepped temperatures, none of them where it began: the nearest)
            near = min(abs(f["temp"] - self.t0) for f in self.fits)
            fits = [f for f in self.fits if abs(abs(f["temp"] - self.t0) - near) < 1e-6]
        if not fits:
            return None
        w = [1.0 / max(f["err"], 1e-4) ** 2 for f in fits]
        ff = sum(wk * f["ff"] for wk, f in zip(w, fits)) / sum(w)
        return {"ff": round(ff, 6), "err": round(1.0 / math.sqrt(sum(w)), 6), "maps": [f["map"] for f in fits]}

    def go(self, current=None, piezo=None, temp=None, step_i=RAMP_I, step_v=RAMP_V):
        yield Go(current, piezo, temp, step_i, step_v)
        if current is not None:
            self.I = current
        if piezo is not None:
            self.V = piezo
        if temp is not None:
            self.T = temp

    # -------------------------------------------------------------- the map
    def run(self):
        yield from self.take_over()
        p = self.plan
        for temp in p.temps():
            if temp is not None:
                yield from self.go(temp=temp)
                yield from self.settle(temp)
            for kind in p.kinds():
                yield from self.map(kind, temp)
                self.fit(self.views[-1])
        if p.repeat and self.views:
            view = self.views[-1]
            again = MapView(len(self.views), view.kind, view.temp, view.fv, view.sv[:1], repeat=True)
            self.views.append(again)
            self.note(self.started + self.t_last, f"the first line of map {view.n + 1} again", "repeat")
            yield from self.line(again, 0, view.sv[0])
        yield from self.go_home()
        return self.summary()

    def take_over(self):
        p = self.plan
        self.stage = "reading its settings"
        extra = tuple(n for n in p.names() if n not in PD_NAMES + (CURRENT_ACT, PIEZO_ACT))
        names = (CURRENT, OFFSET, CURRENT_ACT, PIEZO_ACT, CLIP, FF_ON, FF, PC_MIN, PC_MAX, TEMP_SET, TEMP_ACT, TEMP_MIN,
                 TEMP_MAX, SCAN_ON, LOCK_ON, TA_ON, *ABOUT, *PD_NAMES, *extra)
        got = dict(zip(names, (yield Get(names))))
        i0, v0, a0 = got[CURRENT], got[OFFSET], got[CURRENT_ACT]
        if not (_num(i0) and _num(v0) and _num(a0)):
            raise Failed("cannot read its current and piezo")
        if got[TA_ON] is True:
            raise Failed("its TA is on: switch it off first (without seed it switches itself off, or worse)")
        if got[LOCK_ON] is True:
            raise Failed("its DLC pro locks it to a cavity: switch that lock off first")
        self.i0, self.v0, self.a0 = i0, v0, a0
        self.I, self.V = a0, v0
        self.act_offset = a0 - i0
        self.ff0 = got[FF] if _num(got[FF]) else 0.0
        self.ff_was = self.ff_now = got[FF_ON] is True
        self.ff_factor = self.ff0
        if not self.ff_was or self.ff0 == 0.0:
            self.off_offset = self.act_offset
        self.clip = got[CLIP] if _num(got[CLIP]) else math.inf
        if _num(got[PC_MIN]) and _num(got[PC_MAX]) and got[PC_MIN] < got[PC_MAX]:
            self.pc = (got[PC_MIN], got[PC_MAX])
        if _num(got[TEMP_MIN]) and _num(got[TEMP_MAX]) and got[TEMP_MIN] < got[TEMP_MAX]:
            self.tc = (got[TEMP_MIN], got[TEMP_MAX])
        self.t0 = self.T = got[TEMP_SET] if _num(got[TEMP_SET]) else None
        bad = p.problems(self.limits())  # (as it is; once its feedforward is off, again)
        if bad:
            raise Failed("; ".join(bad))
        self.pd = next((n for n in PD_NAMES if _num(got[n])), "")
        self.names = tuple(dict.fromkeys(([self.pd] if self.pd else []) + [CURRENT_ACT] +
                                         ([PIEZO_ACT] if _num(got[PIEZO_ACT]) else []) + list(extra)))
        if self.out is not None:
            o = self.out
            o.meta("plan", p.to_dict())
            o.meta("laser", {**self.info.get("laser", {}), **{n: got[n] for n in ABOUT}})
            o.meta("wavemeter", self.info.get("wavemeter", {}))
            o.meta("start", {"current_set": i0, "current_act": a0, "piezo": v0, "piezo_act": got[PIEZO_ACT],
                             "temp_set": self.t0, "temp_act": got[TEMP_ACT], "clip": self.clip,
                             "feedforward": self.ff0, "feedforward_enabled": got[FF_ON],
                             "piezo_limits": list(self.pc), "temp_limits": list(self.tc), "scan_enabled": got[SCAN_ON],
                             "act_offset": self.act_offset})
            o.meta("recorded", {"photodiode": self.pd, "names": list(self.names)})
            o.meta("program", {"name": "SimpleWavemeterLock", "version": __version__})
            o.meta("began", {"local": time.strftime("%Y-%m-%d %H:%M:%S"), "unix": time.time()})
            o.meta("units", {"t": "s since the map began", "i": "mA measured (as planned: set plus what it is besides, "
                                                            "its feedforward off)", "i_set": "mA set",
                             "i_act": "mA measured (read at the point)", "v": "V", "v_act": "V (read at the point)",
                             "temp": "°C set", "f": "THz", "power": "µW, the wavemeter's", "exp": "ms",
                             "legs": "1 up, 2 down, 0 a descent: the current down to where it stops lasing",
                             "fits": "ff mA/V, err mA/V, spacing and hysteresis mA (the current swept) or V (the "
                                     "piezo), df_di GHz/mA, df_dv and tuning GHz/V"})
        if got[SCAN_ON] is True:
            self.stage = "its scan switched off"
            yield Set(SCAN_ON, False, restore=True)
            self.scan_was = True
        self.stage = "where it is now"
        t = yield Sample(p.reads, p.settle, self.names)
        fs = [r[1] for r in t.readings if r[1] > 0]
        self.f0 = self.f_ref = statistics.median(fs) if fs else None
        self.note(t.t1, f"began at {a0:.3f} mA ({i0:.3f} mA set), {v0:.3f} V" +
                  (f", {self.t0:.3f} °C" if self.t0 is not None else "") +
                  (f": {self.f0:.6f} THz" if self.f0 else ": no valid reading there"), "start")
        if self.ff_was and self.ff0 != 0.0:
            yield from self.ff_off_for_map()
        elif self.ff_was:  # (on, and its factor 0: it adds nothing)
            yield Set(FF_ON, False)

    def ff_off_for_map(self):
        """Its feedforward off for the map: its factor taken to 0 in steps (ff_ramp: what it adds learned as it
        goes), switched off there (nothing moves), and its factor put back as it was (off, it does nothing: as it
        was, should the program stop)."""
        self.stage = "its feedforward taken off"
        i_set, act = yield Get((CURRENT, CURRENT_ACT))
        if not (_num(i_set) and _num(act)):
            raise Failed("cannot read its current")
        self.act_offset, self.I = act - i_set, act
        yield from self.ff_ramp(0.0)
        yield Set(FF_ON, False)
        yield Set(FF, self.ff0)
        top = max(self.plan.i_hi, self.a0)
        if self.I > top + 0.25:
            raise Failed(f"its feedforward taken off, it runs at {self.I:.3f} mA: above {top:.3f} mA")
        bad = self.plan.problems(self.limits())
        if bad:
            raise Failed("its feedforward off: " + "; ".join(bad))
        added = (f": it added {self.ff0 * self.gap:+.3f} mA at rest ({self.gap:.2f} ± {self.gap_u:.2f} V per mA/V: "
                 f"reckoned from {self.v0 - self.gap:.1f} V)" if self.gap is not None else "")
        self.note(self.started + self.t_last, f"its feedforward ({self.ff0:+.4f} mA/V) off for the map, its factor "
                                              f"taken to 0 in {len(self._steps)} steps{added}; now {self.I:.3f} mA",
                  "feedforward")
        if self.out is not None:
            self.out.meta("feedforward", self.ff_state())

    def ff_state(self) -> dict:
        """What the map knows of its feedforward (kept with it): as it began and as it is; what it adds at rest per
        mA/V of its factor (gap, V: the piezo's distance from where its DLC pro reckons it from), learned from the
        steps of its factor (mA/V, and what each did to the current, mA)."""
        return {"was_on": self.ff_was, "factor": self.ff0, "on": self.ff_now, "factor_now": self.ff_factor,
                "off_offset": self.off_offset, "gap": self.gap, "gap_u": self.gap_u,
                "adds_at_rest": round(self.ff0 * self.gap, 4) if self.gap is not None else None,
                "reckoned_from": round(self.v0 - self.gap, 3) if self.gap is not None else None,
                "steps": [list(x) for x in self._steps], "applied": self.ff_applied, "end": self.ff_end or None}

    def fit(self, view: MapView):
        """The ideal feedforward from that map (fit_feedforward), said and kept."""
        f = fit_feedforward(view)
        if f is None:
            self.note(self.started + self.t_last, f"map {view.n + 1}: too few hops followed from line to line for "
                                                  f"the ideal feedforward", "fit")
            return
        self.fits.append(f)
        self.note(self.started + self.t_last, f"map {view.n + 1}: {say_fit(f)}", "fit")
        if self.out is not None:
            self.out.meta("fits", self.fits)

    def settle(self, temp: float):
        """Wait for the temperature to settle at temp: within t_tol for t_hold, t_wait at most."""
        p = self.plan
        self.stage = f"waiting for {temp:.3f} °C"
        began = steady = None
        while True:
            now = yield Wait(1.0)
            act = (yield Get((TEMP_ACT,)))[0]
            began = now if began is None else began
            if self.out is not None:
                self.out.temp(self.now(now), temp, act if _num(act) else None)
            if _num(act) and abs(act - temp) <= p.t_tol:
                steady = now if steady is None else steady
                if now - steady >= p.t_hold:
                    self.note(now, f"{temp:.3f} °C, settled in {now - began:.0f} s", "temp")
                    return
            else:
                steady = None
            if now - began >= p.t_wait:
                off = f"{act - temp:+.3f} K off" if _num(act) else "its temperature not read"
                self.note(now, f"{temp:.3f} °C not settled within {p.t_wait:.0f} s ({off}): mapped all the same",
                          "temp")
                return

    def map(self, kind: str, temp):
        """A whole map: the `kind` axis swept up and down at each step of the other."""
        p = self.plan
        fast, slow = ("current", "piezo") if kind == "current" else ("piezo", "current")
        view = MapView(len(self.views), kind, temp, p.grid(fast), p.grid(slow))
        self.views.append(view)
        for j, s in enumerate(view.sv):
            yield from self.line(view, j, s)

    def line(self, view: MapView, j: int, s: float):
        """Line j: the slow axis to s, the fast one swept up from its start and back down."""
        p, fv, top = self.plan, view.fv, len(view.fv) - 1
        self.view = view
        self.line_no = (j + 1, len(view.sv))
        out = self.out
        line_id = out.line(view.n, view.kind, j, s, view.temp, view.repeat, self.t_last) if out is not None else j
        if view.kind == "current":  # the current swept at each piezo voltage
            self.stage = f"the piezo to {s:.3f} V"
            yield from self.go(piezo=s)
            k = 0
            if p.dark:  # from where the last sweep down went dark (else the bottom); lasing there: down to dark first
                k = max((n for n, i in enumerate(fv) if i <= self.I + 1e-9), default=0)
            yield from self.go(current=fv[k])
            t = yield from self.point(view, line_id, j, 1, k, record=False)
            if p.dark and k > 0 and not self.is_dark(t):
                self.keep(view, line_id, j, 0, k, t)
                run = 0
                while k > 0 and run < p.dark:
                    k -= 1
                    yield from self.go(current=fv[k], step_i=p.i_step)
                    t = yield from self.point(view, line_id, j, 0, k)
                    run = run + 1 if self.is_dark(t) else 0
                t = yield from self.point(view, line_id, j, 1, k, record=False)
            self.keep(view, line_id, j, 1, k, t)
            for n in range(k + 1, top + 1):
                yield from self.go(current=fv[n], step_i=p.i_step)
                yield from self.point(view, line_id, j, 1, n)
            view.turn(j, top)
            run = 0
            for k in range(top - 1, -1, -1):
                yield from self.go(current=fv[k], step_i=p.i_step)
                t = yield from self.point(view, line_id, j, 2, k)
                run = run + 1 if self.is_dark(t) else 0
                if p.dark and run >= p.dark:
                    if k > 0:
                        self.note(t.t1, f"no lasing below {fv[k + p.dark]:.3f} mA at {s:.3f} V: turned back there",
                                  "turn")
                    break
        else:  # the piezo swept at each current
            self.stage = f"the current to {s:.3f} mA"
            yield from self.go(current=s)
            yield from self.go(piezo=fv[0])
            dark = True
            for k in range(top + 1):
                yield from self.go(piezo=fv[k], step_v=p.v_step)
                t = yield from self.point(view, line_id, j, 1, k)
                dark = dark and self.is_dark(t)
            view.turn(j, top)
            if dark:
                self.note(t.t1, f"no lasing all the way up at {s:.3f} mA: not swept back", "turn")
            else:
                for k in range(top - 1, -1, -1):
                    yield from self.go(piezo=fv[k], step_v=p.v_step)
                    yield from self.point(view, line_id, j, 2, k)
        if out is not None:
            out.line_over(line_id, self.t_last)

    def is_dark(self, t: Taken) -> bool:
        """The sample says the laser does not lase: nothing valid, and its photodiode said so, or the wavemeter said
        nothing new in the time a reading takes, or said that there is no light."""
        if any(r[1] > 0 for r in t.readings):
            return False
        return t.how in ("dark", "quiet") or (bool(t.readings) and all(r[2] in DARK for r in t.readings))

    def point(self, view: MapView, line_id: int, j: int, leg: int, k: int, record: bool = True):
        """The point k of line j sampled, and (record) kept: its Taken."""
        p = self.plan
        level = self.pd_level()
        unit = "mA" if view.fast == "current" else "V"
        self.stage = (f"{'repeat' if view.repeat else f'map {view.n + 1}'}, line {self.line_no[0]} of "
                      f"{self.line_no[1]}, {'up' if leg == 1 else 'down'}, {view.fv[k]:.3f} {unit}")
        t = yield Sample(p.reads, p.settle, self.names, (self.pd, level) if level is not None else None)
        if record:
            self.keep(view, line_id, j, leg, k, t)
        return t

    def keep(self, view: MapView, line_id: int, j: int, leg: int, k: int, t: Taken):
        """A point kept: in the file, in the window's map, its patterns if they are to be."""
        x = view.fv[k]
        i, v = (x, view.sv[j]) if view.fast == "current" else (view.sv[j], x)
        valid = [r for r in t.readings if r[1] > 0]
        fs = [r[1] for r in valid]
        f = statistics.median(fs) if fs else None
        spread = (max(fs) - min(fs)) * 1e6 if len(fs) > 1 else None
        bad = [r for r in t.readings if not r[1] > 0]
        dark = self.is_dark(t)
        last = next((r[3] for r in reversed(valid) if r[3]), None) or \
            next((r[3] for r in reversed(t.readings) if r[3]), None) or {}
        amps, exp = last.get("amps") or None, last.get("exposure") or None
        if (line_id, leg) != self._key:  # a new sweep
            self._key = (line_id, leg)
            if self.trace:
                self.trace_before = self.trace
            self.trace, self.trace_legs = [], (leg, self.trace_legs[0])
            self._leg = Leg(self._leg.steps if view.kind == self._kind else ())
            self._kind = view.kind
        event = self._leg.add(f, dark, spread, contrast(amps))
        if event == "hop":
            self.hops[1 if leg == 1 else 2] += 1
        pd = t.values.get(self.pd) if self.pd else None
        if _num(pd) and f is not None:
            self.pd_lasing.append(pd)
        elif _num(pd) and dark and t.how != "dark":  # (dark as the wavemeter said, not as the photodiode did)
            self.pd_dark.append(pd)
        powers = [r[3]["power"] for r in t.readings if r[3] and _num(r[3].get("power"))]
        power = statistics.median(powers) if powers else None
        self.points += 1
        self.lasing += f is not None
        self.t_last = self.now(t.t1)
        if f is not None and self.f_ref is None:
            self.f_ref = f  # (no valid reading where it began: from the first)
        ghz = (f - self.f_ref) * 1e3 if f is not None else None
        view.put(leg, j, k, ghz if f is not None else MapView.DARK if dark else MapView.BAD,
                 power if power is not None else MapView.DARK if dark else MapView.BAD,
                 pd if _num(pd) else MapView.BAD)
        self.trace.append((x, ghz, power, pd if _num(pd) else None))
        patterns = {kind: pv for kind, pv in (last.get("patterns") or {}).items() if pv and len(pv[1])}
        if patterns:
            self.pattern_now = patterns.get(0) or next(iter(patterns.values()))
        if self.out is None:
            return
        i_act, v_act = t.values.get(CURRENT_ACT), t.values.get(PIEZO_ACT)
        vals = {name: value for name, value in t.values.items() if name not in (CURRENT_ACT, PIEZO_ACT)}
        row = {"line_id": line_id, "leg": leg, "k": k, "t": self.t_last, "i": i, "i_set": round(self.set_for(i, v), 6),
               "i_act": i_act if _num(i_act) else None, "v": v, "v_act": v_act if _num(v_act) else None,
               "temp": self.T, "f": f, "f_lo": min(fs) if len(fs) > 1 else None,
               "f_hi": max(fs) if len(fs) > 1 else None, "n": len(fs), "bad": len(bad),
               "status": bad[-1][2] if bad else "", "how": t.how, "wait": round(t.t1 - t.t0, 4),
               "power": power, "exp1": exp[0] if exp else None,
               "exp2": exp[1] if exp and len(exp) > 1 else None, "event": event,
               "vals": json.dumps(vals, default=str) if vals else None}
        for name, a in zip(("amin1", "amax1", "aavg1", "amin2", "amax2", "aavg2"), amps or ()):
            row[name] = a
        pid = self.out.point(row)
        self.out.readings(pid, [(self.now(r[0]), r[1] if r[1] > 0 else None, r[2], (r[3] or {}).get("power"))
                                for r in t.readings])
        if event:
            self.out.event(self.t_last, event, f"{event} at {i:.3f} mA, {v:.3f} V, sweeping "
                                               f"{'up' if leg == 1 else 'down'}" +
                           (f": {f:.6f} THz" if f is not None else ""))
        if patterns:
            keep = self.keep_patterns(event)
            if keep is True:
                self._store(pid, patterns)
            elif keep:
                self._ring.append((pid, patterns))
                while len(self._ring) > self.AROUND:
                    self._ring.popleft()

    def keep_patterns(self, event: str):
        """Keep this point's patterns: True, 'later' (if an event comes within AROUND points), or False."""
        p = self.plan
        if p.patterns == "all":
            return True
        if p.patterns != "hops":
            return False
        if event:
            for pid, patterns in self._ring:  # (the points before it)
                self._store(pid, patterns)
            self._ring.clear()
            self._after = self.AROUND
            return True
        if self._after > 0:
            self._after -= 1
            return True
        if p.every and self.points % p.every == 0:
            return True
        return "later"

    def _store(self, pid: int, patterns: dict):
        for kind, (dtype, values) in patterns.items():
            self.out.pattern(pid, kind, dtype, values)
        self.kept += 1

    def go_home(self):
        """Back where it began: the temperature, then its feedforward (on again as it was, or on with the ideal one the
        map found: ff_end), the piezo and the current; its scan on again if it was."""
        self.stage = "back where it began"
        if self.t0 is not None and self.T is not None and abs(self.T - self.t0) > 1e-9:
            yield from self.go(temp=self.t0)
            yield from self.settle(self.t0)
        self.stage = "back where it began"
        yield from self.ff_home()
        yield from self.go(current=self.a0, piezo=self.v0)
        t = yield Sample(self.plan.reads, max(self.plan.settle, 0.2), self.names)
        fs = [r[1] for r in t.readings if r[1] > 0]
        self.f_end = statistics.median(fs) if fs else None
        where = f"back at {self.a0:.3f} mA, {self.v0:.3f} V"
        if self.f_end and self.f0:
            self.note(t.t1, f"{where}: {(self.f_end - self.f0) * 1e3:+.3f} GHz from where it began", "end")
        else:
            self.note(t.t1, f"{where}: " + (f"{self.f_end:.6f} THz" if self.f_end else "no valid reading there"), "end")
        if self.scan_was:
            yield Set(SCAN_ON, True)

    def ff_home(self):
        """Its feedforward as it ends: on again as it was, or (ff_end 'fitted') on with the ideal one the maps found:
        the piezo where it began first (while the feedforward adds nothing, the piezo moves alone), switched on at a
        factor of 0, its factor taken there in steps (ff_ramp: the current kept no higher than where it began)."""
        on, factor, why = self.ff_was, self.ff0, ""
        if self.plan.ff_end == "fitted":
            best = self.fitted()
            if best is None:
                why = "no ideal feedforward found in the map"
            elif not abs(best["ff"]) <= FF_MAX:
                why = f"the ideal feedforward found, {best['ff']:+.4f} mA/V, is more than {FF_MAX:g} mA/V"
            elif best["err"] > 0.25 * abs(best["ff"]) + 1e-3:
                why = f"the ideal feedforward found, {best['ff']:+.4f} ± {best['err']:.4f} mA/V, is too uncertain"
            else:
                on, factor = True, best["ff"]
            if why:
                self.note(self.started + self.t_last, f"{why}: its own back as it was", "feedforward")
        if not on or (self.ff_now and self.ff_factor == factor):
            self.ff_end = {"on": self.ff_now, "factor": self.ff_factor, "fitted": False}
            return
        self.stage = "its feedforward on again" if factor == self.ff0 and self.ff_was else \
            f"its feedforward on, {factor:+.4f} mA/V"
        yield from self.go(current=min(self.I, self.a0), piezo=self.v0)
        if not self.ff_now:
            if self.ff_factor != 0.0:
                yield Set(FF, 0.0)
            yield Set(FF_ON, True)
        yield from self.ff_ramp(factor)
        self.ff_applied = not (self.ff_was and factor == self.ff0)
        self.ff_end = {"on": True, "factor": factor, "fitted": self.ff_applied}
        if self.ff_applied:
            was = f"{self.ff0:+.4f} mA/V, {'on' if self.ff_was else 'off'}"
            self.note(self.started + self.t_last, f"its feedforward set to the ideal one the map found, "
                                                  f"{factor:+.4f} mA/V, and on (it was {was})", "feedforward")
        else:
            self.note(self.started + self.t_last, f"its feedforward on again ({factor:+.4f} mA/V)", "feedforward")
        if self.out is not None:
            self.out.meta("feedforward", self.ff_state())

    def summary(self) -> str:
        p = self.plan
        maps = "; ".join(f"the {v.kind} swept over {len(v.fv)} steps at {len(v.sv)} "
                         f"{'piezo voltages' if v.kind == 'current' else 'currents'}" + (
                             f" ({v.temp:.3f} °C)" if v.temp is not None else "")
                         for v in self.views if not v.repeat)
        self.result = (f"{self.points:,} points, {self.lasing:,} lasing: {maps}; hops seen {self.hops[1]:,} sweeping "
                       f"up, {self.hops[2]:,} down; patterns kept at {self.kept:,} points" +
                       (f" ({p.patterns})" if p.patterns != "off" else ""))
        best = self.fitted()
        if best is not None:
            self.result += (f"; the ideal feedforward {best['ff']:+.4f} ± {best['err']:.4f} mA/V" +
                            (" (set)" if self.ff_applied else ""))
        if self.notes and self.notes[-1][1].startswith("back at"):
            self.result += f"; {self.notes[-1][1]}"
        return self.result


# -------------------------------------------------------------------- driver
def exposure_s(extra) -> float | None:
    """s a reading's exposure took (its arrays' together), from its extra; None: not known."""
    exp = (extra or {}).get("exposure")
    if exp and all(_num(x) and x >= 0 for x in exp) and sum(exp) > 0:
        return sum(exp) / 1e3
    return None


class MapRunner:
    """Carries out a Mapper's requests through `io` (as modehop's Runner: io.do(op, done, failed), op ('set', name,
    value) or ('get', names), names below laserN:, the current and the piezo as modehop.CURRENT and OFFSET) and hands
    it the channel's readings (wavemeter.Reading, with their extra); keeps it to the safety rules (the module's
    docstring); on failure, or stopped, puts things back: its scan, then the temperature (not waited for), the
    piezo and the current where they were. exposure(): s the wavemeter's exposure for the laser is now, between its
    readings (None: not known). finished(runner) is called once, when it is over."""

    ACK_TIMEOUT = 5.0
    QUIET_MIN, QUIET_MAX = 0.3, 15.0     # s: nothing new from the wavemeter for this long (by its exposure): dark
    BUDGET_MIN, BUDGET_MAX = 3.0, 60.0   # s a point may take at most, by the wavemeter's pace
    INVALID_MAX = 8                      # invalid readings and nothing valid: given up (the exposure adapting takes
    #                                      a few, coming out of the dark)
    SILENT_MAX = 3                       # points in a row with nothing at all from the wavemeter, in their time:
    #                                      it does not read the laser (its channel gone): the map stops

    def __init__(self, mapper: Mapper, io, now: float, finished=None, exposure=None):
        self.mapper, self.io, self.finished = mapper, io, finished
        self.exposure = exposure
        mapper.started = now
        self.gen = mapper.run()
        self.started = self.now = now
        self.state = "running"   # running | stopping | restoring | done | failed
        self.result = ""
        self.error = ""
        self.restored = False
        self.journal = {}        # name -> what to put back on failure
        self.trouble = []        # what could not be put back
        self.i_written = None    # mA set, V, °C set: what the controller has
        self.v_written = None
        self.t_written = None
        self._ops = deque()
        self._then = None        # after the ops: 'advance', 'answer' or 'restored'
        self._waiting = None     # ack | settle | sample | wait | pause
        self._since = now
        self._until = now
        self._written_at = now
        self._token = 0
        self._req = None         # the Sample going on...
        self._taken = None
        self._get_tok = None     # ...its Taken while its read of parameters is on its way
        self._valid = self._bad = 0
        self._heard = now        # when the last reading (valid or not) came...
        self._status = ""        # ...and what it said ('': valid)
        self._gaps = deque(maxlen=25)
        self._exp = None         # s: the last reading's exposure
        self._hint = None        # s: the longest exposure the source said since the step
        self._silent = 0         # points in a row with nothing from the wavemeter
        self._send(None)

    @property
    def busy(self) -> bool:
        return self.state in ("running", "stopping", "restoring")

    @property
    def stage(self) -> str:
        return self.mapper.stage if self.state == "running" else "going back where it began"

    # ------------------------------------------------------------- the procedure
    def _send(self, value):
        try:
            req = self.gen.send(value)
        except StopIteration as stop:
            self.result = stop.value or ""
            self._end("done")
            return
        except Failed as f:
            self._fail(str(f))
            return
        except Exception as exc:  # a bug: stop, and put things back
            log.exception("map")
            self._fail(f"internal error: {exc}")
            return
        out = self.mapper.out
        if out is not None and out.error:  # (the disk full, say)
            self._fail(f"cannot write its file: {out.error}")
            return
        if isinstance(req, Go):
            self._go(req)
            self._then = "advance"
            self._next()
        elif isinstance(req, Sample):
            self._sample(req)
        elif isinstance(req, Get):
            self._ops.append(("get", tuple(req.names)))
            self._then = "answer"
            self._next()
        elif isinstance(req, Set):
            if req.restore is not None and req.name not in self.journal:
                self.journal[req.name] = req.restore
            self._ops.append(("set", req.name, req.value))
            self._then = "advance"
            self._next()
        elif isinstance(req, Wait):
            self._until = self.now + max(float(req.seconds), 0.0)
            self._waiting = "wait"
            self.tick(self.now)
        else:
            self._fail(f"internal error: unknown request {req!r}")

    def _limits(self):
        """(highest measured current, (lowest, highest) piezo voltage, (lowest, highest) temperature) a write may
        give: the plan's, or as far as where the map began, if that was outside."""
        m = self.mapper
        p = m.plan
        i_max = max(p.i_hi, m.a0) if m.a0 is not None else p.i_hi
        v = (max(min(p.v_lo, m.v0), m.pc[0]), min(max(p.v_hi, m.v0), m.pc[1])) if m.v0 is not None else m.pc
        if p.t_step > 0 and m.t0 is not None:
            t = (max(min(p.t_lo, m.t0), m.tc[0]), min(max(p.t_hi, m.t0), m.tc[1]))
        else:
            t = (m.t0, m.t0) if m.t0 is not None else (math.inf, -math.inf)
        return i_max, v, t

    def _go(self, req: Go):
        """The ops of a move: the temperature (one write), then the piezo and the current together, in steps of at
        most step_i (the set current too) and step_v; a step that has the feedforward take the measured current up
        writes the current first, so that it never overshoots."""
        m = self.mapper
        if req.temp is not None and req.temp != self._ops_value("temp", self.t_written):
            self._ops.append(("temp", float(req.temp)))
        i_set0, v_from = self._ops_value("current", self.i_written), self._ops_value("piezo", self.v_written)
        i_set0 = m.i0 if i_set0 is None else i_set0
        v_from = m.v0 if v_from is None else v_from
        v_to = float(req.piezo) if req.piezo is not None else v_from
        i_from = m.mA(i_set0, v_from)
        i_to = float(req.current) if req.current is not None else i_from
        set_to = m.set_for(i_to, v_to)
        if abs(set_to - i_set0) < 1e-9 and abs(v_to - v_from) < 1e-9:
            return
        step_i, step_v = min(req.step_i, STEP_I), min(req.step_v, STEP_V)
        n = max(1, math.ceil(max(abs(i_to - i_from) / step_i, abs(set_to - i_set0) / step_i,
                                 abs(v_to - v_from) / step_v) - 1e-9))
        i_set, v = i_set0, v_from
        for k in range(1, n + 1):
            v_k = v_from + (v_to - v_from) * k / n
            i_k = m.set_for(i_from + (i_to - i_from) * k / n, v_k)
            if abs(v_k - v) > 1e-12:
                self._ops.append(("piezo", v_k))
            if abs(i_k - i_set) > 1e-12:
                self._ops.append(("current", i_k))
            i_set, v = i_k, v_k

    def _ops_value(self, kind: str, written):
        for op in reversed(self._ops):
            if op[0] == kind:
                return op[1]
        return written

    def _refusal(self, op) -> str:
        """Why op must not be done ('' if it may): the last line of defence."""
        m, kind = self.mapper, op[0]
        i_max, (v_lo, v_hi), (t_lo, t_hi) = self._limits()
        if kind == "current":
            last = self.i_written if self.i_written is not None else m.i0
            if 0.0 <= op[1] <= m.clip + 1e-9 and m.mA(op[1]) <= i_max + 1e-6 and abs(op[1] - last) <= STEP_I + 1e-6:
                return ""
            return f"refused to set {op[1]:.3f} mA ({m.mA(op[1]):.3f} mA measured; from {last:.3f} mA)"
        if kind == "piezo":
            if m.ff_now and m.ff_factor != 0.0:  # (what the feedforward adds as the piezo moves: not known)
                return f"refused to move the piezo to {op[1]:.3f} V while its feedforward adds to the current"
            last = self.v_written if self.v_written is not None else m.v0
            i = self.i_written if self.i_written is not None else m.i0
            if v_lo - 1e-9 <= op[1] <= v_hi + 1e-9 and abs(op[1] - last) <= STEP_V + 1e-6 and \
                    m.mA(i, op[1]) <= i_max + 1e-6:
                return ""
            return f"refused to set the piezo to {op[1]:.3f} V (from {last:.3f} V)"
        if kind == "temp":
            if t_lo - 1e-9 <= op[1] <= t_hi + 1e-9:
                return ""
            return f"refused to set the temperature to {op[1]:.3f} °C"
        if kind == "set":
            want = WRITABLE.get(op[1])
            if want is None or type(op[2]) is not want:  # noqa: E721
                return f"refused to set {op[1]} to {op[2]!r}"
            change = None  # mA/V: what the write changes in what the feedforward adds
            if op[1] == FF:
                if not (abs(op[2]) <= FF_MAX or op[2] == m.ff0):  # (its own, as it was, whatever it was)
                    return f"refused to set its feedforward to {op[2]:+.4f} mA/V"
                change = op[2] - m.ff_factor if m.ff_now else 0.0
            elif op[1] == FF_ON and op[2] != m.ff_now:
                change = m.ff_factor if op[2] else -m.ff_factor
            if change:
                now = m.mA(self.i_written if self.i_written is not None else m.i0)
                if abs(change) * m.ff_most() > STEP_I + 1e-6 or now + m.ff_rise(change) > i_max + 1e-6:
                    return (f"refused to set {op[1]} to {op[2]!r} at {now:.3f} mA: it could take the current "
                            f"{m.ff_rise(change):.3f} mA up, {abs(change) * m.ff_most():.3f} mA either way")
            return ""
        return ""

    def _next(self):
        if not self._ops:
            then, self._then = self._then, None
            if then == "restored":
                self.restored = not self.trouble
                self._end("failed", self.error + ("; could not put back " + ", ".join(self.trouble)
                                                  if self.trouble else ""))
            elif then == "home":  # (its feedforward on again: home as that has it)
                self._home()
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
        wire = {"current": ("set", CURRENT, op[1]) if op[0] == "current" else None,
                "piezo": ("set", OFFSET, op[1]) if op[0] == "piezo" else None,
                "temp": ("set", TEMP_SET, op[1]) if op[0] == "temp" else None}.get(op[0]) or op
        self.io.do(wire, lambda result=None: self._acked(tok, op, result), lambda exc: self._write_failed(tok, op, exc))

    def _acked(self, tok: int, op, result):
        if tok != self._token or not self.busy:
            return
        kind = op[0]
        if kind == "current":
            self.i_written = op[1]
        elif kind == "piezo":
            self.v_written = op[1]
        elif kind == "temp":
            self.t_written = op[1]
        elif kind == "set" and op[1] in (FF_ON, FF):
            self.mapper.ff_switched(op[1], op[2])
        if kind in ("current", "piezo", "temp", "set"):
            self._written_at = self.now
        self._waiting = None
        if self.state == "stopping":
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
            if op[0] == "set" and op[1] not in (FF_ON, FF):
                self.trouble.append(f"{op[1]} ({exc or type(exc).__name__})")
                self._next()
                return
            self._ops.clear()
            m = self.mapper
            if op[0] == "set":  # its feedforward: left as it is now, the current where it is (lowered for it)
                factor = m.ff_factor if m.ff_applied else m.ff0
                if not m.ff_now and m.ff_factor != factor:  # (its factor back as it was: off, that does nothing)
                    self._ops.append(("set", FF, factor))
                what = "switch it on again" if op[1] == FF_ON else f"step its factor to {op[2]:+.4f} mA/V"
                now = m.mA(self.i_written if self.i_written is not None else m.i0)
                self.trouble.append(f"its feedforward (could not {what}: {exc or type(exc).__name__}; left at "
                                    f"{now:.3f} mA, its feedforward {'on' if m.ff_now else 'off'})")
                self._then = "restored"
                self._next()
                return
            self._end("failed", f"{self.error}; could not put it back: {exc}")
        elif isinstance(exc, OSError):  # the connection is gone: nothing more to be done
            self._ops.clear()
            self.gen.close()
            self._end("failed", f"controller: {exc}" if self.state == "running" else self.error)
        elif self.state == "stopping":
            self._ops.clear()
            self._restore()
        else:
            self._ops.clear()
            self._fail(f"controller: {exc}")

    # ------------------------------------------------------------------ samples
    def _sample(self, req: Sample):
        self._req = req
        self._taken = Taken(t0=max(self.now, self._written_at + max(req.settle, 0.0)))
        self._valid = self._bad = 0
        self._hint = None
        self.quiet()  # (the exposure going on now)
        self._waiting = "settle"
        self.tick(self.now)

    def _begin_sampling(self):
        self._waiting = "sample"
        if self._req.names:  # (read meanwhile; its answer, if it comes too late for its point, is dropped)
            taken = self._get_tok = self._taken
            names = tuple(self._req.names)
            self.io.do(("get", names), lambda result=None: self._values(taken, dict(zip(names, result or ()))),
                       lambda exc: self._values(taken, {}))

    def _values(self, taken: Taken, values: dict):
        """A sample's parameters, read (or not: {})."""
        if taken is not self._taken or self._get_tok is not taken or self._waiting != "sample":
            return
        self._get_tok = None
        taken.values = values
        self._check_done()

    def _period(self) -> float:
        """s between the laser's readings, of late."""
        return statistics.median(self._gaps) if self._gaps else 0.05

    def _duration(self, r) -> float:
        """s a reading took: its exposure (both arrays'), or the usual time between readings."""
        exp = exposure_s(getattr(r, "extra", None))
        return exp + 0.003 if exp is not None else self._period()

    def quiet(self) -> float:
        """s without anything new from the wavemeter that means it has nothing to say: a little more than two readings
        take at the exposure it has (on auto it lengthens as the light goes): the one going on when the step was
        written, which began before it, and a whole one after."""
        hint = self.exposure() if self.exposure is not None else None
        if _num(hint):  # (the longest since the step: whether the wavemeter says the exposure going on or the next)
            self._hint = max(self._hint or 0.0, hint)
        exp = max((x for x in (self._hint, self._exp) if _num(x)), default=None)
        base = 2 * exp + 0.15 if exp is not None else 0.0
        return min(max(base, 3 * self._period(), self.QUIET_MIN), self.QUIET_MAX)

    def budget(self) -> float:
        """s a point may take at most."""
        return min(max(4 * self.quiet() + self._req.reads * self._period(), self.BUDGET_MIN), self.BUDGET_MAX)

    def on_reading(self, r):
        """A new reading of the laser (wavemeter.Reading)."""
        self.now = max(self.now, r.t)
        if r.t > self._heard:
            self._gaps.append(r.t - self._heard)
        self._heard = max(self._heard, r.t)
        self._status = r.status
        exp = exposure_s(getattr(r, "extra", None))
        if exp is not None:
            self._exp = exp
        if self._waiting == "settle" and self.now >= self._taken.t0:
            self._begin_sampling()
        if self._waiting == "sample":
            if r.t - self._duration(r) < self._taken.t0 - 1e-6:
                self._taken.early += 1
            else:
                self._taken.readings.append((r.t, r.freq, r.status, getattr(r, "extra", None)))
                if r.freq > 0:
                    self._valid += 1
                else:
                    self._bad += 1
                self._check_done()
        self.tick(r.t)

    def _check_done(self):
        req, t = self._req, self._taken
        due = self._get_tok is not None
        if self._valid >= req.reads and not due:
            self._done("done")
        elif self._valid == 0 and not due and self._bad >= self.INVALID_MAX:
            self._done("invalid")
        elif self._valid == 0 and not due and req.dark_pd and _num(t.values.get(req.dark_pd[0])) and \
                t.values[req.dark_pd[0]] < req.dark_pd[1]:
            self._done("dark")

    def _done(self, how: str):
        t = self._taken
        t.how, t.t1, t.status = how, self.now, self._status
        self._req = self._taken = None
        self._waiting = None
        self._silent = self._silent + 1 if how == "timeout" and not t.readings and not t.early else 0
        if self._silent >= self.SILENT_MAX:
            self._fail(f"the wavemeter gave no readings of it, {self._silent} points in a row")
            return
        self._send(t)

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
        elif self._waiting == "wait" and now >= self._until:
            self._waiting = None
            self._send(now)
        elif self._waiting == "settle" and now >= self._taken.t0:
            self._begin_sampling()
            self._check_done()
        elif self._waiting == "sample":
            t0 = self._taken.t0
            if self._get_tok is not None and now - t0 > self.ACK_TIMEOUT:
                self._get_tok = None  # (its parameters did not come: without them)
                self._check_done()
            if self._waiting != "sample":
                return
            # nothing new, and the last it said was no light: dark (overexposed, say, and nothing new: the same again,
            # its exposure adapting: waited for)
            quiet = not self._taken.readings and self._status in DARK and now - max(t0, self._heard) > self.quiet()
            if self._get_tok is None and (quiet or now - t0 > self.budget()):
                self._done("quiet" if quiet else "invalid" if not self._valid and self._bad else
                           "done" if self._valid else "timeout")

    def cancel(self, why: str = "stopped"):
        """Stop, and put things back."""
        if self.state == "running":
            self._fail(why)

    # -------------------------------------------------------------------- ends
    def _fail(self, why: str):
        self.gen.close()
        self.error = why
        self._ops.clear()
        self._req = self._taken = None
        if self._waiting == "ack":  # let the write on its way land first, so the way back starts from it
            self.state = "stopping"
            return
        self._restore()

    def _restore(self):
        self.state = "restoring"
        self._waiting = None
        self._get_tok = None
        m = self.mapper
        if m.t0 is not None and self.t_written is not None and abs(self.t_written - m.t0) > 1e-9:
            self._ops.append(("temp", m.t0))
        on, factor = (m.ff_now, m.ff_factor) if m.ff_applied else (m.ff_was, m.ff0)  # (the ideal one on: left so)
        if m.ff_now != on or (m.ff_factor != factor):
            self._ff_back(on, factor)
            self._then = "home"
            self._next()
            return
        self._home()

    def _ff_back(self, on: bool, factor: float):
        """The ops that put its feedforward back (on or off, its factor), with nothing read: the piezo where it began
        first (while the feedforward adds nothing; else it is there), the current lowered by as much as the rest
        could take it up, switched only at a factor of 0, its factor stepped as ff_ramp would (each step moving the
        current FF_STEP at most by what is known of what it adds)."""
        m = self.mapper
        if self.v_written is not None and abs(self.v_written - m.v0) > 1e-9:
            self._go(Go(None, m.v0))
        f_from, f_to = (m.ff_factor if m.ff_now else 0.0), (factor if on else 0.0)
        rise = m.ff_rise(f_to - f_from)
        i_set = self._ops_value("current", self.i_written)
        now = m.mA(m.i0 if i_set is None else i_set)
        if now + rise > m.a0 + 1e-9:
            self._go(Go(max(m.a0 - rise, m.mA(0.0)), None))
        if on and not m.ff_now:
            if m.ff_factor != 0.0:
                self._ops.append(("set", FF, 0.0))
            self._ops.append(("set", FF_ON, True))
        most, f = m.ff_most(), f_from
        while abs(f_to - f) > 1e-9:
            want = f_to - f
            f = f_to if abs(want) * most <= FF_STEP else round(f + math.copysign(FF_STEP / most, want), 6)
            self._ops.append(("set", FF, f))
        if m.ff_now and not on:
            self._ops.append(("set", FF_ON, False))
        if not on and factor != f_to:  # (off: it does nothing)
            self._ops.append(("set", FF, factor))

    def _home(self):
        """The rest of the way back: the piezo and the current where they were, then what the journal says (its
        scan on again once the piezo is back)."""
        m = self.mapper
        if self.i_written is not None or self.v_written is not None:
            self._go(Go(m.a0, m.v0))
        for name, value in reversed(list(self.journal.items())):
            self._ops.append(("set", name, value))
        if not self._ops:
            self.restored = True
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


# ----------------------------------------------------------------------- CLI
def describe(m: LaserMap) -> str:
    """What is in a map's file, said."""
    p, start, meta = m.plan, m.meta.get("start", {}), m.meta
    laser = meta.get("laser", {})
    out = [f"{m.path.name}"]
    began = meta.get("began", {}).get("local", "?")
    out.append(f"  began {began}; {laser.get('product-name') or 'a laser'} on channel {laser.get('channel', '?')} "
               f"({laser.get('name') or 'no name'}), {laser.get('device', '?')} laser{laser.get('laser', '?')}")
    wlm = meta.get("wavemeter", {})
    if wlm:
        out.append(f"  wavemeter {wlm.get('model', '')} {wlm.get('serial', '')}".rstrip())
    out.append(f"  plan: current {p.i_lo:g} … {p.i_hi:g} mA (measured) in {p.i_step:g} mA, piezo {p.v_lo:g} … "
               f"{p.v_hi:g} V in {p.v_step:g} V, sweeping {p.sweep}" +
               (f", temperature {p.t_lo:g} … {p.t_hi:g} °C in {p.t_step:g} K" if p.t_step > 0 else "") +
               f"; {p.reads} readings a point, patterns {p.patterns}")
    if start:
        out.append(f"  began at {start.get('current_act', 0):.3f} mA, {start.get('piezo', 0):.3f} V" +
                   (f", {start['temp_set']:.3f} °C" if start.get("temp_set") is not None else "") +
                   (f"; feedforward {start['feedforward']:+.4f} mA/V" if start.get("feedforward") else ""))
    pts = m.points
    lasing = [q for q in pts if q["f"] is not None]
    n_pat = m.db.execute("SELECT COUNT(DISTINCT point_id), COUNT(*) FROM patterns").fetchone()
    took = pts[-1]["t"] if pts else 0.0
    out.append(f"  {len(pts):,} points over {len(m.lines)} lines, {len(lasing):,} lasing, in {fmt_time(took)}; "
               f"patterns at {n_pat[0]:,} points ({n_pat[1]:,} arrays)")
    if lasing:
        fs = [q["f"] for q in lasing]
        out.append(f"  frequencies {min(fs):.6f} … {max(fs):.6f} THz ({(max(fs) - min(fs)) * 1e3:.1f} GHz)")
    hops = {1: 0, 2: 0, 0: 0}
    for q in pts:
        if q["event"] == "hop":
            hops[q["leg"]] = hops.get(q["leg"], 0) + 1
    out.append(f"  hops: {hops[1]:,} sweeping up, {hops[2] + hops[0]:,} down")
    ff = meta.get("feedforward") or {}
    if ff.get("was_on"):
        out.append(f"  its feedforward ({ff.get('factor', 0):+.4f} mA/V) off for the map" +
                   (f": {ff['jump']:+.3f} mA from switching it off" if ff.get("jump") is not None else "") +
                   (f", reckoned from {ff['reckoned_from']:.1f} V" if ff.get("reckoned_from") is not None else ""))
    fits = meta.get("fits")
    if fits is None:  # (a file from before fits were kept, or cut short: fitted now)
        fits = m.fits()
    for f in fits:
        out.append(f"  map {f['map'] + 1} ({f['kind']} swept" + (f", {f['temp']:.3f} °C" if f.get("temp") is not None
                                                                else "") + f"): {say_fit(f)}")
    if (ff.get("end") or {}).get("fitted"):
        out.append(f"  its feedforward set to {ff['end']['factor']:+.4f} mA/V at the end")
    end = meta.get("end", {})
    if end:
        out.append(f"  ended: {end.get('outcome', '?')}")
    facts, doubted = meta.get("model_facts"), False
    if meta.get("model"):  # (said again by this version: what it doubts of a model an older one kept)
        try:
            from . import lasermodel

            model = lasermodel.stored(m)
            if model is not None:
                facts, doubted = model.facts(), not model.plausible
        except (ImportError, TypeError, ValueError, KeyError):
            pass
    if facts:
        out.append("  the laser, as its model has it (kept in the file):")
        out += [f"   - {fact}" for fact in facts]
        if doubted:
            out.append("  (fit it again: --model --save-model)")
    return "\n".join(out)


def export_csv(m: LaserMap, path) -> int:
    """Its points as CSV (one row a point, the columns of the points table, vals spread out): how many rows."""
    names = sorted({k for q in m.points for k in q["vals"]})
    cols = [c for c in POINT_COLUMNS if c != "vals"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols + names)
        for q in m.points:
            w.writerow([q[c] for c in cols] + [q["vals"].get(n) for n in names])
    return len(m.points)


def park_text(m: LaserMap, nm: float, model=None, temps=None, current: str = "act", robust: bool = False) -> str:
    """Where to park a map's laser to lase at nm nm (parking.py), in words: by its model (given, else kept in its file
    if it can be its laser's, else fitted); robust: ranked by the worst with the temperature's other coefficients."""
    from . import lasermodel, parking
    from .wavemeter import C_NM_THZ

    if model is None:
        model = lasermodel.stored(m)
        if model is None or not model.plausible:
            model = lasermodel.fit(m, current, log=lambda text: print(f"  model: {text}", flush=True))
    start, fed = m.meta.get("start") or {}, m.meta.get("feedforward") or {}
    ff = start.get("feedforward") if start.get("feedforward_enabled") else None
    thz = C_NM_THZ / nm
    spots = parking.park(model, thz, temps=tuple(temps) if temps else None, feedforward=ff, robust=robust)
    say_nm = f"{nm:.6f}".rstrip("0").rstrip(".")
    out = [f"  where to park it at {say_nm} nm ({thz:.6f} THz), by its model: the widest mode first" +
           (" whichever of the temperature's coefficients its map allows" if robust else "") + ", power not counted"]
    if not spots:
        out.append("   none: no mode on target holds there, as its model has it")
    off = fed.get("off_offset") if fed.get("off_offset") is not None else start.get("act_offset", 0.0)
    for sp in spots:
        out.append(f"   - {parking.say_spot(sp)}")
        text = f"     set: the current {sp.i - off:.2f} mA, its feedforward off"
        if ff and fed.get("reckoned_from") is not None:
            text += (f" ({sp.i - off - ff * (sp.v_set - fed['reckoned_from']):.2f} mA with it on, {ff:+.3f} mA/V "
                     f"reckoned from {fed['reckoned_from']:.1f} V)")
        out.append(text + "; the piezo (scan offset) as above, the temperature as above and settled")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m wmlock.lasermap", description="What a laser map's file holds.")
    ap.add_argument("files", nargs="+", metavar="FILE")
    ap.add_argument("--csv", action="store_true", help="write each one's points as FILE.csv")
    ap.add_argument("--model", action="store_true",
                    help="fit the laser's model to it (lasermodel.py; needs numpy and scipy) and say its facts")
    ap.add_argument("--save-model", action="store_true", help="with --model: keep the model in the file")
    ap.add_argument("--map", type=int, default=0, metavar="N",
                    help="with --model: which of its maps (1, 2...; 0: the last)")
    ap.add_argument("--current", choices=("act", "set", "plan"), default="act",
                    help="with --model: the current as read at each point (act), else set (set: files from before "
                         "it was read), or as planned")
    ap.add_argument("--park", type=float, metavar="NM",
                    help="where to park it to lase at NM nm, on the mode that holds most widely there, by its model "
                         "(kept in the file, else fitted; a map at several temperatures: the temperature too)")
    ap.add_argument("--temp", type=float, nargs=2, metavar=("LO", "HI"),
                    help="with --park: the temperatures to look over, °C (else those it was mapped at)")
    ap.add_argument("--robust", action="store_true",
                    help="with --park: ranked by the worst of each spot and the spots near it were the temperature's "
                         "coefficients the others its map allows as well")
    args = ap.parse_args(argv)
    for name in args.files:
        with load(name) as m:
            print(describe(m))
            if args.csv:
                out = Path(name).with_suffix(".csv")
                print(f"  {export_csv(m, out):,} points written to {out}")
            model = None
            if args.model:
                from . import lasermodel

                model = lasermodel.fit(m, args.current, args.map - 1 if args.map > 0 else None,
                                       log=lambda text: print(f"  model: {text}", flush=True))
                print("  the laser, as its model has it:")
                for fact in model.facts():
                    print(f"   - {fact}")
            if args.park is not None:
                print(park_text(m, args.park, model, args.temp, args.current, args.robust))
        if args.model and args.save_model:
            lasermodel.save(name, model)
            print(f"  kept in {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
