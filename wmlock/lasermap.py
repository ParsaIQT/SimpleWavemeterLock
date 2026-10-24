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
(the spectrum its window shows: below), and the DLC pro's own photodiode and whatever else `record` names, read
meanwhile. The exposure is on auto: where the laser is dim the wavemeter measures slowly, and the map waits for it;
where it gives nothing to read (the laser does not lase), it waits only as long as the exposure says a reading would
take, and once its photodiode tells dark from lasing, not even that.

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

The procedure (Mapper.run, a generator, as modehop's Recenter) yields Go (a move, in steps), Sample (a point), Get,
Set and Wait; MapRunner carries them out through the laser's controller, to the safety rules: the measured current
never above i_hi (nor above where it began, if that was higher: on the way back), the set current never above the
controller's current-clip, the piezo within the controller's limits and the plan's, the temperature within the
controller's limits and the plan's, nothing else written but its scan switched off, and every move in steps. With
the DLC pro's feedforward on, the set current is written so that the measured current is what the map says (the
feedforward moves it ff mA per volt of piezo). A laser its DLC pro locks to a cavity, or with its TA on, is not
mapped: that lock off first; the TA off first (without seed it switches itself off, or worse). At the end, or when
stopped, it goes back where it began: the temperature, then the piezo and the current, and its scan on again if it
was; there the frequency is read and compared with where it began (hysteresis may leave it on another mode).
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
CLIP = "dl:cc:current-clip"
FF_ON, FF = "dl:cc:feedforward-enabled", "dl:cc:feedforward-factor"
PC_MIN, PC_MAX = "dl:pc:voltage-min", "dl:pc:voltage-max"
TEMP_SET, TEMP_ACT = "dl:tc:temp-set", "dl:tc:temp-act"
TEMP_MIN, TEMP_MAX = "dl:tc:limits:temp-min", "dl:tc:limits:temp-max"
PD_NAMES = ("dl:pd:photodiode", "dl:pd:power")  # the DL pro's own photodiode, as firmwares name it (the first there)
SCAN_ON = "scan:enabled"
LOCK_ON = "dl:lock:lock-enabled"
TA_ON = "amp:cc:enabled"
ABOUT = ("product-name", "label", "type", "emission")  # kept with the map, as the controller says them
WRITABLE = {SCAN_ON: bool}  # what a map writes besides current, piezo and temperature
LOOK = (CLIP, FF_ON, FF, PC_MIN, PC_MAX, TEMP_SET, TEMP_ACT, TEMP_MIN, TEMP_MAX, SCAN_ON, LOCK_ON, TA_ON,
        *PD_NAMES)  # what the map dialog shows of a laser before it begins (Engine.map_look)
DARK = ("no signal", "low signal")  # what the wavemeter says when there is next to no light
FOLDER = Path.home() / "laser-maps"

RAMP_I = 1.0   # mA: the most a move changes the current per write (a sweep: its step)...
RAMP_V = 5.0   # V: ...and the piezo
STEP_I = 5.0   # mA: the largest step a plan may have, and so any write
STEP_V = 10.0  # V
MAX_POINTS = 5_000_000
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
                     i REAL, i_set REAL, v REAL, temp REAL,
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
POINT_COLUMNS = ("id", "line_id", "leg", "k", "t", "i", "i_set", "v", "temp", "f", "f_lo", "f_hi", "n", "bad",
                 "status", "how", "wait", "power", "exp1", "exp2", "amin1", "amax1", "aavg1", "amin2", "amax2", "aavg2",
                 "vals", "event")


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
    """One map as it fills, for the window: at each point, each way (up: leg 1; down: legs 2 and 0), the frequency
    in GHz from where the laser began, DARK where it did not lase, BAD where the wavemeter said nothing valid, None
    where not measured yet. repeat: the first line of the map before it, swept again."""

    DARK = -math.inf
    BAD = math.nan

    def __init__(self, n: int, kind: str, temp, fv: list, sv: list, repeat: bool = False):
        self.n, self.kind, self.temp, self.repeat = n, kind, temp, repeat
        self.fast, self.slow = ("current", "piezo") if kind == "current" else ("piezo", "current")
        self.fv, self.sv = fv, sv
        self.up = [[None] * len(fv) for _ in sv]
        self.down = [[None] * len(fv) for _ in sv]
        self.version = 0  # bumped at every point...
        self.up_v = [0] * len(sv)  # ...and its line's, each way
        self.down_v = [0] * len(sv)

    def put(self, leg: int, j: int, k: int, value):
        (self.up if leg == 1 else self.down)[j][k] = value
        (self.up_v if leg == 1 else self.down_v)[j] += 1
        self.version += 1


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
        self.act_offset = 0.0      # mA: measured minus set current, with the piezo at v0...
        self.ff = 0.0              # mA/V: ...and the feedforward's (0: off)
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
        self.views = []            # MapView, a map each
        self.view = None           # the one filling
        self.line_no = (0, 0)      # (line, of how many) of the map filling
        self.t_last = 0.0          # s: the last point's
        self.trace = []            # [(fast value, GHz or None)] of the sweep going on...
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

    def mA(self, i_set: float, v: float) -> float:
        """A set current as measured, with the piezo at v."""
        return i_set + self.act_offset + self.ff * (v - self.v0)

    def set_for(self, i: float, v: float) -> float:
        """The set current that has the measured current at i with the piezo at v."""
        return i - self.act_offset - self.ff * (v - self.v0)

    def note(self, t, text: str, kind: str = "note"):
        s = self.now(t) if t else None
        self.notes.append((s, text))
        if self.out is not None:
            self.out.event(s, kind, text)
        log.info("map: %s", text)

    def limits(self) -> dict:
        return {"clip": self.clip, "offset": self.act_offset, "ff": self.ff, "v0": self.v0, "pc": self.pc,
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
        extra = tuple(n for n in p.names() if n not in PD_NAMES)
        names = (CURRENT, OFFSET, CURRENT_ACT, CLIP, FF_ON, FF, PC_MIN, PC_MAX, TEMP_SET, TEMP_ACT, TEMP_MIN, TEMP_MAX,
                 SCAN_ON, LOCK_ON, TA_ON, *ABOUT, *PD_NAMES, *extra)
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
        self.ff = got[FF] if got[FF_ON] is True and _num(got[FF]) else 0.0
        self.clip = got[CLIP] if _num(got[CLIP]) else math.inf
        if _num(got[PC_MIN]) and _num(got[PC_MAX]) and got[PC_MIN] < got[PC_MAX]:
            self.pc = (got[PC_MIN], got[PC_MAX])
        if _num(got[TEMP_MIN]) and _num(got[TEMP_MAX]) and got[TEMP_MIN] < got[TEMP_MAX]:
            self.tc = (got[TEMP_MIN], got[TEMP_MAX])
        self.t0 = self.T = got[TEMP_SET] if _num(got[TEMP_SET]) else None
        bad = p.problems(self.limits())
        if bad:
            raise Failed("; ".join(bad))
        self.pd = next((n for n in PD_NAMES if _num(got[n])), "")
        self.names = tuple(dict.fromkeys(([self.pd] if self.pd else []) + list(extra)))
        if self.out is not None:
            o = self.out
            o.meta("plan", p.to_dict())
            o.meta("laser", {**self.info.get("laser", {}), **{n: got[n] for n in ABOUT}})
            o.meta("wavemeter", self.info.get("wavemeter", {}))
            o.meta("start", {"current_set": i0, "current_act": a0, "piezo": v0, "temp_set": self.t0,
                             "temp_act": got[TEMP_ACT], "clip": self.clip, "feedforward": self.ff,
                             "feedforward_enabled": got[FF_ON], "piezo_limits": list(self.pc),
                             "temp_limits": list(self.tc), "scan_enabled": got[SCAN_ON],
                             "act_offset": self.act_offset})
            o.meta("recorded", {"photodiode": self.pd, "names": list(self.names)})
            o.meta("program", {"name": "SimpleWavemeterLock", "version": __version__})
            o.meta("began", {"local": time.strftime("%Y-%m-%d %H:%M:%S"), "unix": time.time()})
            o.meta("units", {"t": "s since the map began", "i": "mA measured", "i_set": "mA set", "v": "V",
                             "temp": "°C set", "f": "THz", "power": "µW, the wavemeter's", "exp": "ms",
                             "legs": "1 up, 2 down, 0 a descent: the current down to where it stops lasing"})
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
                  (f": {self.f0:.6f} THz" if self.f0 else ": no valid reading there") +
                  (f"; its feedforward, {self.ff:+.4f} mA/V, taken into account" if self.ff else ""), "start")

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
            view.put(2, j, top, view.up[j][top])  # (where it turns: the same both ways)
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
            view.put(2, j, top, view.up[j][top])
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
        self.points += 1
        self.lasing += f is not None
        self.t_last = self.now(t.t1)
        if f is not None and self.f_ref is None:
            self.f_ref = f  # (no valid reading where it began: from the first)
        ghz = (f - self.f_ref) * 1e3 if f is not None else None
        view.put(leg, j, k, ghz if f is not None else MapView.DARK if dark else MapView.BAD)
        self.trace.append((x, ghz))
        patterns = {kind: pv for kind, pv in (last.get("patterns") or {}).items() if pv and len(pv[1])}
        if patterns:
            self.pattern_now = patterns.get(0) or next(iter(patterns.values()))
        if self.out is None:
            return
        row = {"line_id": line_id, "leg": leg, "k": k, "t": self.t_last, "i": i, "i_set": round(self.set_for(i, v), 6),
               "v": v, "temp": self.T, "f": f, "f_lo": min(fs) if len(fs) > 1 else None,
               "f_hi": max(fs) if len(fs) > 1 else None, "n": len(fs), "bad": len(bad),
               "status": bad[-1][2] if bad else "", "how": t.how, "wait": round(t.t1 - t.t0, 4),
               "power": last.get("power"), "exp1": exp[0] if exp else None,
               "exp2": exp[1] if exp and len(exp) > 1 else None, "event": event,
               "vals": json.dumps(t.values, default=str) if t.values else None}
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
        """Back where it began: the temperature, then the piezo and the current; its scan on again if it was."""
        self.stage = "back where it began"
        if self.t0 is not None and self.T is not None and abs(self.T - self.t0) > 1e-9:
            yield from self.go(temp=self.t0)
            yield from self.settle(self.t0)
        self.stage = "back where it began"
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

    def summary(self) -> str:
        p = self.plan
        maps = "; ".join(f"the {v.kind} swept over {len(v.fv)} steps at {len(v.sv)} "
                         f"{'piezo voltages' if v.kind == 'current' else 'currents'}" + (
                             f" ({v.temp:.3f} °C)" if v.temp is not None else "")
                         for v in self.views if not v.repeat)
        self.result = (f"{self.points:,} points, {self.lasing:,} lasing: {maps}; hops seen {self.hops[1]:,} sweeping "
                       f"up, {self.hops[2]:,} down; patterns kept at {self.kept:,} points" +
                       (f" ({p.patterns})" if p.patterns != "off" else ""))
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
            ops = [("piezo", v_k)] if abs(v_k - v) > 1e-12 else []
            if abs(i_k - i_set) > 1e-12:
                ops.insert(0 if m.ff * (v_k - v) > 0 else len(ops), ("current", i_k))
            self._ops.extend(ops)
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
            v = self.v_written if self.v_written is not None else m.v0
            last = self.i_written if self.i_written is not None else m.i0
            if (0.0 <= op[1] <= m.clip + 1e-9 and m.mA(op[1], v) <= i_max + 1e-6
                    and abs(op[1] - last) <= STEP_I + 1e-6):
                return ""
            return f"refused to set {op[1]:.3f} mA ({m.mA(op[1], v):.3f} mA measured; from {last:.3f} mA)"
        if kind == "piezo":
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
            if want is not None and type(op[2]) is want:  # noqa: E721
                return ""
            return f"refused to set {op[1]} to {op[2]!r}"
        return ""

    def _next(self):
        if not self._ops:
            then, self._then = self._then, None
            if then == "restored":
                self.restored = not self.trouble
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
            if op[0] == "set":
                self.trouble.append(f"{op[1]} ({exc or type(exc).__name__})")
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
        if self.i_written is not None or self.v_written is not None:
            self._go(Go(m.a0, m.v0))
        for name, value in reversed(list(self.journal.items())):  # (its scan on again once the piezo is back)
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
    end = meta.get("end", {})
    if end:
        out.append(f"  ended: {end.get('outcome', '?')}")
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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m wmlock.lasermap", description="What a laser map's file holds.")
    ap.add_argument("files", nargs="+", metavar="FILE")
    ap.add_argument("--csv", action="store_true", help="write each one's points as FILE.csv")
    args = ap.parse_args(argv)
    for name in args.files:
        with load(name) as m:
            print(describe(m))
            if args.csv:
                out = Path(name).with_suffix(".csv")
                print(f"  {export_csv(m, out):,} points written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
