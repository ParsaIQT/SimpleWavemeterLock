"""Mode-hop recovery: put a laser that has hopped back on its target frequency, on a mode that
will hold, by moving its diode current as well as its piezo, within safe limits; and, for a laser
its DLC pro locks to a cavity (optionally an SHG laser), relock it the way you would by hand.

Why the current. An external-cavity diode laser lases on the external-cavity mode that is best
aligned with its selection filter: the grating together with the diode chip's own modes. The
piezo moves the mode and, more slowly, the filter; the current moves the filter quickly (the
chip modes shift by GHz per mA) and the mode only slowly. Temperature drifts slide the filter
until a neighbouring mode wins: a mode hop. Pulling the frequency back with the piezo alone,
as a lock would, leaves the laser at the very edge of its mode, where it hops again (or
cannot get back at all). Moving the current re-centers the filter on a mode that reaches the
target, and a re-centered mode holds for a long time.

The procedure (Recenter.run, a generator: see "Driving it" below)
  take over  (a laser locked by its DLC pro, or with an SHG stage) read how it is set up, turn
             its cavity lock off (and the SHG cavity's lock), and stop the scan if that starts
  near       if the laser is within reach of its target, tune the piezo onto it and check the
             margins (below). Fine: done; the cavity lock had just jumped to another peak, say.
  map        sweep the current over its allowed window at fixed piezo, down, up and down
             again, one reading per step. The readings fall into straight segments, one per
             mode, separated by hops. A mode's stable range is where it lases in both sweep
             directions (outside it, hysteresis decides which of two modes you get). With an
             SHG stage its cavity is scanned meanwhile, and wherever the intra-cavity sweep
             shows a second mode (cavity.py) is cut out of the stable ranges.
  choose     a stable range that is wide (up to twice the margin either side counts) and
             needs little piezo to reach the target from its middle (an edge that is only the
             window's is taken to go on a little); go to that middle, or as near as lets the
             piezo reach. The piezo tuning is measured on the way.
  tune       piezo onto the target, at fixed current, watching for hops.
  check      step the current by +-margin around the operating point: the frequency must
             stay on one straight line and come back, i.e. no hop within the margin (and with
             an SHG stage, single mode at both ends and in the middle). Otherwise, or after a
             big piezo move (the piezo moves the modes too), map again here, up to three maps.
  hand back  lock the SHG cavity (its intra-cavity signal must come up); then the cavity lock
             (relock.py): with lockpoints, start the scan, pick the TEM00 peak where the
             wavemeter puts the target among the DLC pro's lockpoint candidates, null the
             error signal's DC level with the FALC pro's input offset, select the peak, lock,
             and make sure the wavemeter and the transmission agree; else just re-enable it.
             A wrong peak: try again.
The current never leaves nominal +- window (nor exceeds the controller's own current limit),
never changes by more than STEP_MAX per write, and the piezo stays within its limits; Runner
also refuses any other write but the few named in WRITABLE. On failure, current, offset and
everything else it changed go back to where they were, except that a cavity lock it turned off
stays off.

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
    relock: str = "auto"      # the DLC pro's lock: auto | lockpoint (pick the peak) | plain (re-enable)
    falc: bool = True         # null the error signal's DC with the FALC pro's input offset first
    falc_max: float = 0.05    # V: the most the input offset may move from where it was
    window: float = 150.0     # MHz: the right cavity peak is within this of where the wavemeter puts it
    confirm: bool = False     # step by step: ask before each step
    i_nom: float = 0.0        # mA, the laser's normal current (0: read it when enabled)
    i_window: float = 2.0     # mA: the current stays within i_nom +- this, always
    v_window: float = 10.0    # V: how far a recovery may move the scan offset
    step: float = 0.1         # mA between readings while mapping
    margin: float = 0.5       # mA: wanted distance from a hop, both ways
    tolerance: float = 100.0  # MHz: further than this from the target is off target...
    grace: float = 2.0        # s: ...for this long (and two readings) means a hop
    max_per_hour: int = 4     # recoveries; more than that and something else is wrong
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

    LEARNED = ("kv", "spacing", "peak_y", "lp_kind", "lp_code", "shg_area", "shg_side", "shg_n", "falc_sign")

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
    "dl:lock:lock-enabled": bool, "scan:enabled": bool,
    "nlo:shg:lock:lock-enabled": bool, "nlo:shg:scan:enabled": bool,
    "scope:channel1:signal": int, "scope:channel2:signal": int,
    "nlo:shg:scope:channel1:signal": int, "nlo:shg:scope:channel2:signal": int,
}
FALC_OFFSET = re.compile(r"falc[1-4]:input:offset$")  # within falc_max of where it was
COMMANDS = {"dl:lock:select-lockpoint": 3}  # name: number of arguments
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


# ------------------------------------------------------------------- the map
@dataclass
class Point:
    i: float   # mA
    e: float   # MHz from the target, NaN: no valid reading
    leg: int   # sweep leg
    up: bool   # the current was rising
    bad: bool = False  # the SHG sweep there showed a second mode


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
    e_ref: float   # MHz at i_ref
    slope: float   # MHz/mA, common to all modes
    i_ref: float
    up: list = field(default_factory=list)      # current ranges where it lased, sweeping up
    down: list = field(default_factory=list)    # ... and sweeping down
    stable: list = field(default_factory=list)  # both: no other mode possible there
    n: int = 0

    def at(self, i: float) -> float:
        return self.e_ref + self.slope * (i - self.i_ref)


HOP_MIN = 150.0  # MHz: steps smaller than this are never taken for a hop


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
    common slope. Wherever a point is bad (two modes at once, by the SHG sweep) is no mode's
    stable range, whichever leg it was seen on."""
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
    bad = _merge([(p.i - step / 2, p.i + step / 2) for p in points if p.bad], 1e-9)
    for refs, segs in groups:
        n = sum(len(s) for s in segs)
        m = Mode(sum(r * len(s) for r, s in zip(refs, segs)) / n, slope, i_ref, n=n)
        for s in segs:
            rng = (min(p.i for p in s), max(p.i for p in s))
            (m.up if s[0].up else m.down).append(rng)
        m.up, m.down = _merge(m.up, gap), _merge(m.down, gap)
        m.stable = _subtract(_intersect(m.up, m.down), bad)
        modes.append(m)
    gaps = [b.e_ref - a.e_ref for a, b in zip(modes, modes[1:])]
    return MapResult(modes, threshold, slope, _median(gaps) if gaps else 0.0, noise)


@dataclass
class Band:
    """A stable current range of a mode, and the operating current chosen in it."""
    mode: Mode
    lo: float         # mA, as mapped
    hi: float
    v: float          # scan offset it was mapped at
    vlo: float = 0.0  # mA, the range's edges, an edge that is only the window's pushed out
    vhi: float = 0.0
    op: float = 0.0   # mA, where to operate: the middle, unless the piezo cannot reach from there
    score: float = 0.0

    @property
    def width(self) -> float:
        return self.hi - self.lo

    @property
    def center(self) -> float:
        return 0.5 * (self.vlo + self.vhi)

    @property
    def margin(self) -> float:
        return min(self.op - self.vlo, self.vhi - self.op)

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
    KV_PLAUSIBLE = (10.0, 5000.0)
    VERIFY_S = 10.0        # s for the cavity lock to show the right peak
    VERIFY_N = 5           # readings in a row on target that prove it

    def __init__(self, p: HopParams, current: float, offset: float, i_lo: float, i_hi: float,
                 v_lo: float, v_hi: float, kv: float = 0.0, cavity: bool = False, quick: bool = True,
                 shg: bool | None = None):
        """quick: if the laser is close to its target, first try the piezo alone (after a cavity
        lock caught the wrong peak, say); else always map, which finds the best mode.
        cavity: its DLC pro has it locked to a cavity (take that over, relock it after).
        shg: it has an SHG stage (by default p.shg)."""
        self.p = p
        self.quick = quick
        self.I0, self.V0 = current, offset  # where the laser was when this began
        self.I, self.V = current, offset    # where it is now
        self.i_lo, self.i_hi, self.v_lo, self.v_hi = i_lo, i_hi, v_lo, v_hi
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
        self.bad = []                       # current ranges where the SHG sweep showed two modes
        self.record = {"notes": [], "maps": [], "sweeps": []}
        CavitySteps.__init__(self, p.shg if shg is None else shg)

    # ---------------------------------------------------------------- helpers
    def note(self, text: str):
        self.notes.append(text)
        self.record["notes"].append([round(self.t, 2) if math.isfinite(self.t) else None, text])
        log.info("recenter: %s", text)

    def _got(self, reading):
        self.t, self.e = reading
        self.readings += 1
        return self.e

    def read(self):
        return self._got((yield Read()))

    def move(self, current=None, offset=None, read=True):
        if current is not None:
            current = min(max(current, self.i_lo), self.i_hi)
            self.I = current
        if offset is not None:
            offset = min(max(offset, self.v_lo), self.v_hi)
            self.V = offset
        reading = yield Move(current, offset, read)
        return self._got(reading) if read else None

    def ask(self, text: str):
        """Step by step: wait for the user before doing what `text` says."""
        if self.p.confirm:
            yield Confirm(text)

    def tol(self) -> float:
        """How close the piezo must bring it: well inside a cavity lock's capture range."""
        return min(5.0, self.p.tolerance / 10)

    def near(self) -> float:
        """Closer than this to the target, try the piezo alone first."""
        return max(0.3 * self.spacing if self.spacing else 1500.0, 2 * self.p.tolerance)

    # ------------------------------------------------------------------ steps
    def run(self):
        if self.cavity or self.shg:
            yield from self.take_over()
        yield from self.read()
        for _ in range(self.RELOCK_TRIES):
            yield from self.find()
            if not self.cavity and not self.shg:
                return self.result()
            if (yield from self.hand_back()):
                return self.result()
            yield from self.take_back()
        raise Failed(f"relocking failed {self.RELOCK_TRIES} times, last: {self.relock_trouble}", keep=True)

    def find(self):
        """Put the laser on target, on a mode with margins: the quick way if it is near, else map."""
        if self.shg_ok:
            yield from self.shg_scan(True)  # its sweep shows how single-mode the laser is, all along
        if self.quick and math.isfinite(self.e) and abs(self.e) <= self.near():
            yield from self.ask(f"The laser is {self.e:+.0f} MHz from its target: tune the piezo onto it")
            if ((yield from self.tune()) == "ok" and (yield from self.check(self.p.margin))
                    and (yield from self.mode_ok())):
                return
        for _ in range(self.MAX_MAPS):
            bands = yield from self.map()
            if not bands:
                raise Failed(f"no stable mode in {self.i_lo:.2f} … {self.i_hi:.2f} mA"
                             + (" within reach of the piezo" if self.kv_known else ""))
            for band in bands:
                if not (yield from self.goto(band)):
                    break                 # not where the map said: map again
                outcome = yield from self.tune(band)
                if outcome == "limit":
                    self.note(f"mode at {band.center:.2f} mA out of the piezo's reach")
                    continue              # next candidate
                if outcome != "ok":
                    break                 # it hopped while tuning: map again, here
                if abs(self.V - band.v) > self.REMAP_V and self.maps < self.MAX_MAPS:
                    break                 # the piezo moved the modes too: map again, here
                if ((yield from self.check(max(self.MIN_MARGIN, min(self.p.margin, 0.8 * band.margin))))
                        and (yield from self.mode_ok())):
                    return
                break                     # too close to a hop: map again, here
        raise Failed(f"no mode that holds reaches the target within {self.i_lo:.2f} … {self.i_hi:.2f} mA")

    def map(self):
        self.maps += 1
        start, v = self.I, self.V
        n = sum(len(_steps(a, b, self.p.step)) for a, b in ((start, self.i_lo), (self.i_lo, self.i_hi),
                                                              (self.i_hi, start)))
        yield from self.ask(f"Map the modes: sweep the current {self.i_lo:.2f} … {self.i_hi:.2f} mA "
                            f"in {self.p.step:g} mA steps ({n} readings) at {v:.2f} V"
                            + (", scanning the SHG cavity" if self.shg_ok else ""))
        self.stage = f"mapping modes ({self.maps})"
        points = [Point(start, self.e, 0, False)]
        sweeps, traces = [None], [None]
        legs = [(start, self.i_lo, False), (self.i_lo, self.i_hi, True), (self.i_hi, start, False)]
        for leg, (a, b, up) in enumerate(legs):
            for i in _steps(a, b, self.p.step):
                e = yield from self.move(current=i)
                points.append(Point(i, e, leg, up))
                s = (yield from self.shg_sweep()) if self.shg_ok else None
                sweeps.append(s)
                traces.append(cavity.decimate(self._last_trace, 300) if s is not None else None)
        if any(s is not None for s in sweeps):
            ref = cavity.reference_from([s for s in sweeps if s is not None])
            if ref is not None:
                self.shg_ref = ref
            judged = [(cavity.judge(s, self.shg_ref), pt, s, y) for pt, s, y in zip(points, sweeps, traces)
                      if s is not None]
            for (ok, _), pt, _, _ in judged:
                pt.bad = ok is False
            good = [j for j in judged if j[0][0]]
            bad = [j for j in judged if j[0][0] is False]
            examples = ([max(good, key=lambda j: j[2].area)] if good else []) + (
                [min(bad, key=lambda j: j[2].area)] if bad else [])  # for a look afterwards
            for (ok, text), pt, _, y in examples:
                self.record["sweeps"].append({"why": f"map {self.maps}", "i": round(pt.i, 3), "ok": ok,
                                              "text": text, "y": y})
        self.bad = _merge([(pt.i - self.p.step / 2, pt.i + self.p.step / 2) for pt in points if pt.bad], 1e-9)
        res = analyse(points, self.p.step)
        modes, self.threshold, self.slope = res.modes, res.threshold, res.slope
        if res.spacing:
            self.spacing = res.spacing
        if res.noise:
            self.noise = res.noise
        if not self.kv_measured and modes:
            yield from self.measure_kv(modes, v)
        bands = []
        for mode in modes:
            for lo, hi in mode.stable:
                band = self.operating_point(mode, lo, hi, v)
                if band is not None:
                    bands.append(band)
        bands.sort(key=lambda b: -b.score)
        bad = sum(pt.bad for pt in points)
        self.note(f"map {self.maps} at {v:.2f} V: {len(modes)} modes"
                  + (f", {bad} points multimode in the SHG cavity" if bad else "")
                  + (": " if bands else "")
                  + ", ".join(f"{b.lo:.2f}–{b.hi:.2f} mA ({b.e:+.0f} MHz)" for b in bands[:3]))
        self.record["maps"].append({
            "v": round(v, 4), "points": [[round(pt.i, 4), round(pt.e, 1) if math.isfinite(pt.e) else None,
                                          pt.leg, pt.bad] for pt in points],
            "bands": [[round(b.lo, 3), round(b.hi, 3), round(b.op, 3), round(b.e, 1)] for b in bands[:5]]})
        return bands

    def operating_point(self, mode: Mode, lo: float, hi: float, v: float):
        """Where to operate in a stable range: its middle, or as near as the piezo allows (once the
        tuning is measured); None if the piezo cannot reach the target from anywhere in it.

        An edge at the window's edge is not a hop: the mode goes on beyond, by how much the map
        cannot tell; it is counted on for another margin's worth (unless it is where the SHG sweep
        showed a second mode)."""
        edge = 1.5 * self.p.step

        def cut(x):
            return any(a - self.p.step <= x <= b + self.p.step for a, b in self.bad)

        b = Band(mode, lo, hi, v,
                 vlo=lo - self.p.margin if lo <= self.i_lo + edge and not cut(lo) else lo,
                 vhi=hi + self.p.margin if hi >= self.i_hi - edge and not cut(hi) else hi)
        lo2 = max(b.vlo + self.MIN_MARGIN, self.i_lo)
        hi2 = min(b.vhi - self.MIN_MARGIN, self.i_hi)
        if lo2 > hi2:
            lo2 = hi2 = min(max(b.center, self.i_lo), self.i_hi)
        if self.kv_measured:  # mode.at(i) + kv * dv = 0 with v + dv within the piezo's limits
            e1, e2 = sorted((self.kv * (v - self.v_hi), self.kv * (v - self.v_lo)))
            if abs(mode.slope) > 1e-9:
                a, z = sorted((mode.i_ref + (e1 - mode.e_ref) / mode.slope,
                               mode.i_ref + (e2 - mode.e_ref) / mode.slope))
                lo2, hi2 = max(lo2, a), min(hi2, z)
            elif not e1 <= mode.e_ref <= e2:
                return None
            if lo2 > hi2:
                return None
        b.op = min(max(b.center, lo2), hi2)
        nominal = self.p.i_nom or self.I0
        b.score = (min(b.margin, 2 * self.p.margin) - self.SHIFT * abs(b.e / self.kv)
                   - 0.1 * abs(b.op - nominal) / max(self.p.i_window, 0.1))
        return b

    def measure_kv(self, modes, v: float):
        """Piezo +0.5 V and back in the middle of the widest stable range, where it cannot hop:
        how many MHz per volt, to choose between modes with."""
        widest = max(((hi - lo, mode, 0.5 * (lo + hi)) for mode in modes for lo, hi in mode.stable),
                     default=None, key=lambda x: x[0])
        if widest is None or widest[0] < 2 * self.MIN_MARGIN:
            return
        self.stage = "measuring the piezo tuning"
        e0 = yield from self.move(current=widest[2])
        e1 = yield from self.move(offset=v + 0.5)
        e2 = yield from self.move(offset=v)
        a, b = 2 * (e1 - e0), 2 * (e1 - e2)
        lo, hi = self.KV_PLAUSIBLE
        if all(math.isfinite(x) for x in (a, b)) and lo <= abs(a) <= hi and 0.7 < b / a < 1.4:
            self.kv, self.kv_known, self.kv_measured = 0.5 * (a + b), True, True
            self.note(f"piezo tuning {self.kv:+.0f} MHz/V")

    def goto(self, band: Band):
        yield from self.ask(f"Go to {band.op:.2f} mA ({band.lo:.2f}–{band.hi:.2f} mA, {band.e:+.0f} MHz "
                            f"from the target there), then tune the piezo onto the target")
        self.stage = "moving to the best mode"
        if abs(self.V - band.v) > 1e-6:
            yield from self.move(offset=band.v, read=False)
        e = yield from self.move(current=band.op)
        return math.isfinite(e) and abs(e - band.e) <= max(0.5 * self.threshold, 50.0)

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
                ok, text = cavity.judge(s, self.shg_ref) if s is not None else (None, "")
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
            self.note(f"hop within {m:.2f} mA of {ic:.2f} mA (off the line by {worst:.0f} MHz)")
            return False
        if modes:
            self.note(f"two modes within {m:.2f} mA of {ic:.2f} mA ({'; '.join(modes)})")
            return False
        if abs(self.e) > self.tol():
            return (yield from self.tune()) == "ok"
        return True

    def verify(self):
        """After relocking the cavity: VERIFY_N readings in a row within tolerance/2."""
        self.stage = "waiting for the cavity lock"
        t0, good = None, 0
        while True:
            e = yield from self.read()
            t0 = self.t if t0 is None else t0
            good = good + 1 if math.isfinite(e) and abs(e) <= self.p.tolerance / 2 else 0
            if good >= self.VERIFY_N:
                return True
            if self.t - t0 > self.VERIFY_S:
                return False

    def result(self) -> Result:
        if self.maps:
            summary = (f"recentered {self.I0:.2f} → {self.I:.2f} mA, {self.V0:.2f} → {self.V:.2f} V"
                       f" ({self.maps} map{'s' if self.maps != 1 else ''}, {self.readings} readings)")
        else:
            summary = f"the laser itself was fine; piezo {self.V0:.2f} → {self.V:.2f} V"
        if self.cavity:
            summary = "relocked, " + summary
        if self.mode_text:
            summary += f"; {self.mode_text}"
        if self.falc_text:
            summary += f"; {self.falc_text}"
        self.record.update({"i0": self.I0, "v0": self.V0, "i": self.I, "v": self.V, "summary": summary})
        return Result(self.I, self.V, self.kv if self.kv_measured else 0.0,
                      self.spacing if self.maps else 0.0, self.maps, self.readings, summary,
                      dict(self.learned), self.record)


# --------------------------------------------------------------------- driver
class Runner:
    """Carries out a Recenter's requests through `io` and feeds it the channel's readings.

    io.do(op, done, failed) performs one operation on the laser's controller and calls done(result)
    or failed(exc) afterwards:
      ('set', name, value)   result None
      ('get', names)         result [value, ...], None for a parameter the controller lacks
      ('exec', name, args)   result its output, or the DecopError if the controller refused
    Names are below laserN: unless absolute (ABSOLUTE). Whatever the procedure asks for, the current
    is kept within its window and ramped in steps of at most STEP_MAX, the offset within its
    window, and nothing else is written but WRITABLE, a FALC pro's input offset within falc_max of
    where it was, and COMMANDS. After a write, SETTLE readings are skipped: the exposure may have
    straddled it. On failure or cancel, everything written goes back where it was (a Set's
    `restore`; current and offset unless the failure says keep), before current and offset.
    finished(runner) is called once, when it is over."""

    SETTLE = 1
    INVALID = 3          # invalid readings in a row passed over before one is handed on
    ACK_TIMEOUT = 5.0    # s
    READ_TIMEOUT = 10.0  # s
    CONFIRM_TIMEOUT = 900.0  # s: step by step, nobody answering for this long stops it

    def __init__(self, rec: Recenter, io, now: float, timeout: float = 300.0, finished=None):
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

    def _ramp(self, target: float):
        r = self.rec
        target = min(max(float(target), r.i_lo), r.i_hi)
        last = self._ops_current()
        for i in _steps(last, target, STEP_MAX) if abs(target - last) > 1e-9 else [target]:
            self._ops.append(("current", i))

    def _ops_current(self) -> float:
        for op in reversed(self._ops):
            if op[0] == "current":
                return op[1]
        return self.i_written

    def _refusal(self, op) -> str:
        """Why op must not be done ('' if it may): the last line of defence."""
        r, kind = self.rec, op[0]
        if kind == "current":
            if r.i_lo - 1e-9 <= op[1] <= r.i_hi + 1e-9 and abs(op[1] - self.i_written) <= STEP_MAX + 1e-9:
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
        elif self.state == "running" and self._waiting == "confirm":
            if now - self._since > self.CONFIRM_TIMEOUT:
                self._fail("nobody said go on")
        elif self.state == "running" and self._waiting == "read" and now - self._since > self.READ_TIMEOUT:
            self._fail("no valid readings")
        elif self.state == "running" and now > self.deadline:
            self._fail("it took too long")
        elif self.state == "running" and self._waiting == "wait" and now >= self._until:
            self._waiting = None
            self._send(None)

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
            if abs(self.v_written - self.rec.V0) > 1e-9:
                self._ops.append(("offset", self.rec.V0))
            self._ramp(self.rec.I0)
            if self._ops and self._ops[-1] == ("current", self.i_written):
                self._ops.pop()  # the ramp of nothing
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
