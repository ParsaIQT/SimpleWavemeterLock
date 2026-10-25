"""A laser's physics, from its map (lasermap.py): an external-cavity diode laser described as two combs of modes and
the rule that picks which of them lases, each part fitted to what the map measured and said as facts about the laser.

The picture (a Littrow ECDL: a diode chip, a collimating lens, a grating moved by a piezo):

- The external cavity's modes, from the chip's back facet to the grating: a comb de GHz apart (de = c / 2L, L its
  optical length), moved by the current (e_i GHz/mA), by the piezo (e_v GHz/V, and e_vv GHz/V² as a piezo is not
  linear: the grating pushed along the cavity) and drifting (e_t: the cavity warming or cooling).
- The chip's own resonances. A front facet that reflects (its anti-reflection coating not perfect, or none) makes
  the chip a Fabry-Pérot of its own: a comb dc GHz apart (dc = c / 2 n_g L_chip), moved fast by the current (c_i: the
  chip heats) and hardly by the piezo. The external modes near the middle of a chip resonance lase most easily
  (their light resonates in the chip as well). Without such a reflection there is one selection filter (the
  grating's and the gain's), moving as they do: the same rule with one broad resonance.
- A window, the grating's feedback and the chip's gain together, outside which it lases less and less (lo .. hi GHz;
  its edges soft: a mode `soft` GHz beyond one loses as much more as one halfway between chip modes, so a chip mode
  near an edge can still take over where the one inside is between two of its external modes).
- The rule: the external mode nearest its chip resonance's best place lases (its loss sin²(π δ / dc), δ its distance
  from that place), and goes on lasing until another's loss is lower by the hysteresis h (h_up sweeping up, h_down
  down). A resonance's best place: `offset` GHz from the chip comb's middle, moved as the chip's temperature lags the
  current's heating by `lag` s (ahead of the comb's middle sweeping the current up, behind it sweeping down), and
  pulled toward the mode lasing, `pull` of the way: the mode lasing changes the chip it lases in (its carriers, its
  heat), so that pull > 0 holds a mode once it lases and pull < 0 pushes it on.

So sweeping the current, the two combs slide past each other (c_i - e_i GHz/mA) and the best aligned pair steps from
one chip resonance to the next (a vernier: dc is not a whole number of de), wrapping around at the window's edges;
sweeping the piezo, the external comb slides alone. Along dI/dV = (e_v - c_v) / (c_i - e_i) the two stay aligned:
the ideal feedforward, for this laser.

- The diode's temperature (as its controller sets it, T °C): it moves the chip's modes c_T GHz/K (the chip's
  refractive index and length as it warms: some 0.04 to 0.06 nm/K for a red or near-infrared diode), the external
  modes e_T (the chip's share of the cavity, de/dc of c_T, and the cavity's own expansion as its mount warms with the
  diode) and the window w_T; the rule is the same at every temperature. So a temperature moves the chip's modes past
  the window and the external modes: which chip mode lases where, and how far from the window's edges.

fit(m) fits it to a map's file (lasermap.LaserMap), in steps, each from what the map shows most directly:
1. how a mode tunes along the swept axis (consecutive points on one mode);
2. the external comb's spacing: the lasing frequencies, that tuning taken off, bunch at whole multiples of it,
   line by line (each line's own offset absorbs the slow axis and the drift, so these cannot bias it); the chip
   comb's spacing likewise, first guessed from the sizes of the hops (whole numbers of chip modes, if a chip comb
   decides them);
3. each comb's offset line by line, unwrapped from line to line into one comb, then fitted by least squares as
   quadratic in the current and the piezo, linear in time, and offset by the way it was swept. The first line swept
   again at the end tells drift from the piezo, but only to whole modes: each nearby number of modes of drift is
   fitted, the best kept, and those nearly as good kept as well (the map cannot tell them apart, and says so);
4. the rule (window and how soft its edges, hysteresis, offset, lag, pull), and the chip comb's spacing and current
   tuning again (where it hops tells them better than where its frequencies lie): every sweep simulated from its
   first point, scored by how many of its points the model puts on the mode the laser was on (differential
   evolution, all candidates and sweeps at once), from the rule that makes what the laser did likeliest a point at
   a time (_likelihood), searched once where its hysteresis of its own holds a mode, once where its pull does
   (HOLDS): the better kept, the other as well (alt: the map tells them apart only so far, and the recovery plans
   for both);
5. the power at the wavemeter: linear in the current above a threshold, and how it changes along a mode (with δ).

A map at several temperatures (fit_temps; fit() goes there when a file has them): each temperature's combs fitted
alone (steps 1 to 3), then the temperature's coefficients from where they are at each (their offsets known only to
whole modes: every coefficient that puts them on one line, whole modes more or less per step, fits as well; the one a
diode's chip has chosen, then the external one that goes with it: the chip's share of the cavity, and less than a whole
cavity's expansion besides; the others kept and said), the combs fitted again over all the temperatures at once with
them; the window's from where it lased at each; the rule (step 4) over every temperature's sweeps at once.

LaserModel.facts() says it all as sentences; LaserModel.simulate() predicts the frequency along any path of current,
piezo, time and temperature, LaserModel.step() one point after another (the simulator's lasers can be made of it:
sim.ModelLaser), LaserModel.at(T) the model at another temperature; park() where to put the laser for a frequency,
on a mode that holds most widely. Needs numpy and scipy.
"""
from __future__ import annotations

import copy
import json
import math
import os
import sqlite3
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass, field

import numpy as np

from . import compiled

C_LIGHT = 299792458.0  # m/s
GROUP_GAN, GROUP_III_V = 2.6, 3.7  # a diode chip's group index, roughly: GaN (below 560 nm), else AlGaInP, GaAs...
UNIT_T = 600.0         # s: drift is fitted per this much time (said per 10 min)
SINGLE = 50.0          # one selection filter: its loss as a chip resonance this many external modes apart would have
RULE = ("lo", "hi", "h_up", "h_down", "offset", "pull", "lag")
WINDOW = ("w_v", "w_i", "soft")
THIN = 12000           # points enough for the combs' spacings (a larger map: thinned for those searches)
MAX_DRIFT = 10.0       # GHz per UNIT_T: the most a comb is taken to drift (more: free-running ECDLs don't)
BUDGET = 40            # generations of each of the rule's searches
THREADS = max(1, (os.cpu_count() or 2) - 1)  # threads a search's candidates are shared among (numpy lets go of the
#                                              GIL as it works: so many cores at once, one left for the rest)
CHUNK = 100            # sweeps scored at once (memory: sweeps x points x modes)
LENGTHS = ((5.0, 500.0), (0.5, 30.0))  # mm: the optical lengths an ECDL's external cavity and its chip (n_g L) have
HOPS_ON = 0.7          # of its map's hops, at least this share whole numbers of its external modes (else: not its comb)
CHIP_T = (-7.5e-5, -1.3e-4, -3e-5)  # per K, of its frequency: how a diode chip's modes move with its temperature (the
#                                     likeliest, and the most and least any has: 0.05, 0.085 and 0.02 nm/K at 650 nm)
MOUNT_T = 23e-6        # per K: an aluminium mount's expansion (an ECDL's cavity no more than this, at the diode's)
MODEL_VERSION = 2      # a model's make: 2, its temperature, its window's edges soft, its chip's modes not drifting


# --------------------------------------------------------------------- data
@dataclass
class MapData:
    """A map's points as arrays (one entry a point, in the order measured): line (its index among the lines, in the
    order swept), leg (1 up, 2 down), i (mA, as it was), v (V), t (s), f (GHz from f_ref; NaN: no frequency), ok (a
    clean frequency: not spread over two modes), power (µW, NaN: none), T (°C, as set; NaN: not known); lines: the
    map's lines (axis, slow value, repeat, t0, map, temp), f_ref THz; kind: which axis was swept ('current' or
    'piezo')."""
    line: np.ndarray
    leg: np.ndarray
    i: np.ndarray
    v: np.ndarray
    t: np.ndarray
    f: np.ndarray
    ok: np.ndarray
    power: np.ndarray
    lines: list
    f_ref: float
    kind: str = "current"
    T: np.ndarray | None = None

    def __post_init__(self):
        if self.T is None:
            self.T = np.full(len(self.f), math.nan)

    @property
    def fast(self) -> np.ndarray:
        return self.i if self.kind == "current" else self.v

    def temps(self) -> list:
        """°C: the temperatures its points were mapped at, in order."""
        return sorted({float(x) for x in self.T[np.isfinite(self.T)]})

    def maps(self) -> np.ndarray:
        """Each point's map (its line's)."""
        return np.array([ln.get("map", 0) for ln in self.lines], dtype=int)[self.line] if len(self.line) else \
            np.zeros(0, dtype=int)

    def sweeps(self):
        """(indices of a sweep's points, in order), sweep by sweep."""
        key = self.line * 4 + self.leg
        cut = np.flatnonzero(np.diff(key)) + 1
        return np.split(np.arange(len(key)), cut)

    def only(self, keep) -> MapData:
        """Its points where keep is true (whole sweeps: a temperature's, a map's)."""
        idx = np.flatnonzero(keep)
        return MapData(self.line[idx], self.leg[idx], self.i[idx], self.v[idx], self.t[idx], self.f[idx],
                       self.ok[idx], self.power[idx], self.lines, self.f_ref, self.kind, self.T[idx])

    def thinned(self, most: int) -> MapData:
        """At most about `most` of its points, evenly through each sweep (for searches that need no more)."""
        k = max(1, math.ceil(len(self.f) / max(most, 1)))
        if k == 1:
            return self
        idx = np.concatenate([sw[::k] for sw in self.sweeps()]) if len(self.f) else np.zeros(0, dtype=int)
        return MapData(self.line[idx], self.leg[idx], self.i[idx], self.v[idx], self.t[idx], self.f[idx],
                       self.ok[idx], self.power[idx], self.lines, self.f_ref, self.kind, self.T[idx])


def map_temps(m) -> dict:
    """°C each map of a file (lasermap.LaserMap) was made at, by its index: those swept as its last was (None: not
    known)."""
    mains = [ln for ln in m.lines if not ln.get("repeat")]
    if not mains:
        return {}
    last = max(ln["map"] for ln in mains)
    axis = next(ln for ln in mains if ln["map"] == last)["axis"]
    return {ln["map"]: ln.get("temp") for ln in mains if ln["axis"] == axis}


def data_of(m, current: str = "act", spread: float = 0.3, map_n: int | str | None = None) -> MapData:
    """One map's points (lasermap.LaserMap: its legs up and down; descents left out), with its first line swept
    again at the end if that was this map's: map_n (None: the last map, the one swept again; 'all': every map swept
    as the last was, one after another: a map at each of several temperatures); their frequencies
    from the middle (median) of the file's, whichever map (f_ref). The current as
    LaserMap.sweeps has it ('act': as read at each point, else the set current and what it was besides where it
    began). spread: GHz between a point's readings above which its frequency is no one mode's. A frequency far
    from all the others (beyond four times the range of the middle half, either side of it) is no mode's either:
    the wavemeter misread its interference order (THz away), both readings alike; as many as a quarter of them
    could not widen that range."""
    start = m.meta.get("start") or {}
    off = (start.get("current_act") or 0.0) - (start.get("current_set") or 0.0)
    fs = [q["f"] for q in m.points if q["f"] is not None]
    f_ref = round(float(np.median(fs)), 6) if fs else 0.0  # (THz: the map's middle frequency, whatever its misreads)
    mains = [ln for ln in m.lines if not ln.get("repeat")]
    if not mains:
        raise ValueError("no lines in the map")
    last = max(ln["map"] for ln in mains)
    ns = sorted(map_temps(m)) if map_n == "all" else [last if map_n is None else map_n]
    firsts = {n: next((ln for ln in mains if ln["map"] == n), None) for n in ns}
    for n, ln in firsts.items():
        if ln is None:
            raise ValueError(f"no map {n + 1} in the file")
    first = firsts[ns[0]]
    take = {ln["id"] for ln in mains if ln["map"] in firsts}
    if last in firsts:  # (the first line swept again comes after the last map, and is its)
        take |= {ln["id"] for ln in m.lines if ln.get("repeat") and ln["axis"] == firsts[last]["axis"]
                 and ln["temp"] == firsts[last]["temp"]}
    by_id = {ln["id"]: ln for ln in m.lines}
    order, rows = {}, []
    for line, leg, pts in m.legs():
        if leg not in (1, 2) or line.get("id") not in take:
            continue
        k = order.setdefault(line["id"], len(order))
        for q in pts:
            if current == "act" and q.get("i_act") is not None:
                i = q["i_act"]
            elif current in ("act", "set") and q.get("i_set") is not None:
                i = q["i_set"] + off
            else:
                i = q["i"]
            v = q["v_act"] if q.get("v_act") is not None else q["v"]
            f = (q["f"] - f_ref) * 1e3 if q["f"] is not None else math.nan
            clean = q["f"] is not None and (q["f_hi"] is None or (q["f_hi"] - q["f_lo"]) * 1e3 <= spread)
            temp = line.get("temp")
            rows.append((k, leg, i, v, q["t"], f, clean, q["power"] if q["power"] is not None else math.nan,
                         temp if temp is not None else math.nan))
    a = np.array(rows, dtype=float) if rows else np.zeros((0, 9))
    lines = [by_id[lid] for lid, _ in sorted(order.items(), key=lambda kv: kv[1])]
    ok = a[:, 6] > 0.5
    if ok.sum() > 20:  # (a wavemeter that misread its order, both readings alike: THz away; none of its modes)
        q1, q3 = np.percentile(a[ok, 5], [25, 75])
        w = max(q3 - q1, 1.0)
        ok &= (a[:, 5] > q1 - 4 * w) & (a[:, 5] < q3 + 4 * w)
    return MapData(a[:, 0].astype(int), a[:, 1].astype(int), a[:, 2], a[:, 3], a[:, 4], a[:, 5], ok,
                   a[:, 7], lines, f_ref, first["axis"], a[:, 8])


def pace_of(d: MapData) -> tuple:
    """(s per unit of the slow axis from line to line, s from point to point, the swept axis's step, s from the
    first line to that line swept again (None: not swept again)), as the map went."""
    mains = [ln for ln in d.lines if not ln.get("repeat")]
    per = [(b["t0"] - a["t0"]) / (b["slow"] - a["slow"]) for a, b in zip(mains, mains[1:])
           if b["slow"] != a["slow"] and a.get("t0") is not None and b.get("t0") is not None
           and a.get("map") == b.get("map")]
    dts, dxs = [], []
    for idx in d.sweeps():
        if len(idx) > 1:
            dts.append(np.diff(d.t[idx]))
            dxs.append(np.abs(np.diff(d.fast[idx])))
    rep = next((ln for ln in d.lines if ln.get("repeat")), None)
    again = None
    if rep is not None and mains and rep.get("t0") is not None:  # (from that line as first swept: its map's)
        was = [ln for ln in mains if ln["slow"] == rep["slow"] and ln.get("temp") == rep.get("temp")] or mains[:1]
        again = rep["t0"] - was[-1]["t0"] if was[-1].get("t0") is not None else None
    return (float(np.median(per)) if per else None, float(np.median(np.concatenate(dts))) if dts else None,
            float(np.median(np.concatenate(dxs))) if dxs else None, again)


# -------------------------------------------------------------------- combs
def onmode_rate(d: MapData, jump: float = 1.5) -> float:
    """GHz per unit of the swept axis (mA, or V) on one mode: the median slope between consecutive clean points of a
    sweep whose frequencies differ by less than `jump` GHz."""
    out = []
    for idx in d.sweeps():
        x, f, ok = d.fast[idx], d.f[idx], d.ok[idx]
        for a, b in zip(range(len(idx) - 1), range(1, len(idx))):
            if ok[a] and ok[b] and abs(f[b] - f[a]) < jump and x[b] != x[a]:
                out.append((f[b] - f[a]) / (x[b] - x[a]))
    return float(np.median(out)) if out else 0.0


def _bunching(x: np.ndarray, groups: np.ndarray, spacings: np.ndarray) -> np.ndarray:
    """For each spacing: how much x bunches at whole multiples of it, group by group (each its own offset): the
    groups' |mean of exp(2πi x / spacing)|, weighted by their sizes."""
    order, starts = _grouped(groups)
    return _bunched(x[order][None, :], starts, 1.0 / np.asarray(spacings, dtype=float)[:, None], len(x))


def _grouped(groups: np.ndarray) -> tuple:
    """(the order that puts points group by group, where each group begins in that order)."""
    order = np.argsort(groups, kind="stable")
    g = groups[order]
    return order, np.flatnonzero(np.concatenate([[True], g[1:] != g[:-1]])) if len(g) else np.zeros(0, dtype=int)


def _bunched(X: np.ndarray, starts: np.ndarray, per, total: int, most: int = 4_000_000) -> np.ndarray:
    """For each row of X (points in the order _grouped puts them; or one row for all) and of per (1 / a spacing, a
    row each, or one for all): the groups' |sum of exp(2πi x per)| summed, over total. Worked out in cycles less the
    nearest whole one (float32 is then as good as float64, and its sin and cos several times as fast), the groups'
    sums at once (reduceat), in parts of `most` at most."""
    per = np.asarray(per, dtype=float).reshape(-1, 1)
    rows = max(X.shape[0], per.shape[0])
    if X.shape[1] == 0:
        return np.zeros(rows)
    out = np.empty(rows)
    step = max(1, min(most // X.shape[1], -(-rows // max(THREADS, 1))))

    def part(a):
        u = (X if X.shape[0] == 1 else X[a:a + step]) * (per if per.shape[0] == 1 else per[a:a + step])
        f = (u - np.rint(u)).astype(np.float32) * np.float32(2 * np.pi)
        c = np.add.reduceat(np.cos(f), starts, axis=1, dtype=np.float64)
        s = np.add.reduceat(np.sin(f), starts, axis=1, dtype=np.float64)
        out[a:a + len(c)] = np.hypot(c, s).sum(axis=1)

    pool = _pool() if rows > step else None  # (the rows on several threads at once: numpy lets go of the GIL)
    list(map(part, range(0, rows, step))) if pool is None else list(pool.map(part, range(0, rows, step)))
    return out / max(total, 1)


def _largest_peak(score: np.ndarray, near: float = 0.9) -> int:
    """The index of the last (largest) local peak of score that is nearly as high as the highest: a comb's spacing
    rather than its halves, thirds... (on which its frequencies lie too)."""
    top = float(np.max(score))
    inner = (score[1:-1] >= score[:-2]) & (score[1:-1] >= score[2:])
    peaks = np.concatenate([[score[0] > score[1]], inner, [score[-1] > score[-2]]]) if len(score) > 2 else \
        np.ones(len(score), dtype=bool)
    good = np.flatnonzero(peaks & (score >= near * top))
    return int(good[-1]) if len(good) else int(np.argmax(score))


def comb_spacing(d: MapData, rate: float, lo: float, hi: float, quad: float = 0.0) -> tuple:
    """(spacing, how strongly the frequencies bunch at it): the frequencies, `rate` GHz per unit of the swept axis
    (and `quad` per unit squared) taken off, bunch at whole multiples of the spacing line by line; searched from lo
    to hi GHz (the largest that does nearly as well as the best: they bunch at its halves, thirds... as well),
    finely about it."""
    s = d.ok
    x0 = d.fast[s] - np.median(d.fast[s])
    x = d.f[s] - rate * x0 - quad * x0 ** 2
    g = d.line[s]
    grid = np.exp(np.linspace(math.log(lo), math.log(hi), int(1000 * math.log(hi / lo)) + 1))  # (a peak is s / x
    #                             wide in its logarithm, x GHz the spread of a line's frequencies: 20 to 30 steps)
    b = _bunching(x, g, grid)
    k = _largest_peak(b)
    fine = np.linspace(grid[max(k - 2, 0)], grid[min(k + 2, len(grid) - 1)], 201)
    bf = _bunching(x, g, fine)
    j = int(np.argmax(bf))
    return float(fine[j]), float(bf[j])


def hop_quantum(d: MapData, small: float) -> float | None:
    """GHz: the quantum the sizes of the larger hops are whole multiples of (larger than `small`, the external
    comb's spacing: the chip's modes), None if there is none."""
    sizes = []
    for idx in d.sweeps():
        f, ok = d.f[idx], d.ok[idx]
        for a in range(len(idx) - 1):
            if ok[a] and ok[a + 1]:
                step = abs(f[a + 1] - f[a])
                if step > 2.5 * small:
                    sizes.append(step)
    if len(sizes) < 10:
        return None
    sizes = np.array(sizes)
    f = d.f[d.ok]
    sizes = sizes[sizes <= np.ptp(np.percentile(f, [1, 99]))]  # (no hop is larger than where it lases)
    if len(sizes) < 10:
        return None
    grid = np.linspace(2.5 * small, min(sizes.max(), 500.0), 4000)
    u = sizes[None, :] / grid[:, None]  # (in cycles, less the nearest whole one: float32's cos as good, and faster)
    score = np.cos((u - np.rint(u)).astype(np.float32) * np.float32(2 * np.pi)).mean(axis=1, dtype=np.float64)
    k = _largest_peak(score)
    return float(grid[k]) if score[k] > 0.5 else None


def hops_on(d: MapData, spacing: float, small: float = 1.0, tol: float | None = None) -> float | None:
    """The share of the map's hops (consecutive clean points of a sweep more than `small` GHz apart, a mode's own
    tuning taken off) whose sizes are within tol GHz (a quarter of the spacing) of a whole number of spacings, not
    none: near 1 for the comb its modes are; small for one too coarse (its hops within its modes, which do not
    jump; with tol a quarter of the finer spacing, few of the rest near its multiples). None: no hops."""
    rate = onmode_rate(d)
    sizes = []
    for idx in d.sweeps():
        f, x, ok = d.f[idx], d.fast[idx], d.ok[idx]
        step = np.diff(f) - rate * np.diff(x)
        good = ok[:-1] & ok[1:] & (np.abs(step) > small)
        sizes.append(step[good])
    sizes = np.concatenate(sizes) if sizes else np.zeros(0)
    if len(sizes) < 10:
        return None
    k = np.round(sizes / spacing)
    return float(np.mean((k != 0) & (np.abs(sizes - k * spacing) < (0.25 * spacing if tol is None else tol))))


@dataclass
class Comb:
    """A comb of modes: spacing GHz apart (0: one selection filter, no comb), at frequencies m * spacing + p0 + p_i dI
    + p_ii dI² + p_v dV + p_vv dV² + p_t dt + p_T dT (dI, dV, dt, dT from the model's i0, v0, t0, T0; dt in units of
    UNIT_T, dT in K), m whole, and + p_leg / 2 sweeping down, - p_leg / 2 up; its rms residual (GHz), how strongly the
    frequencies bunched (0..1), how many points; drift_known: a line swept again later told drift from piezo (else
    p_t is 0 and the drift is in p_v, p_vv); others: the drifts the map allows nearly as well (whole modes more or less
    by the time it was swept again), each {p0, p_v, p_vv, p_t, p_leg}."""
    spacing: float
    p0: float = 0.0
    p_i: float = 0.0
    p_v: float = 0.0
    p_vv: float = 0.0
    p_t: float = 0.0
    rms: float = 0.0
    bunching: float = 0.0
    points: int = 0
    drift_known: bool = False
    p_ii: float = 0.0
    p_leg: float = 0.0
    others: list = field(default_factory=list)
    jump: float = 0.0  # the most its offset moved from one line to the next, of its spacing (near 1/2: ambiguous)
    p_T: float = 0.0   # GHz/K: as the diode's temperature moves it

    def base(self, di, dv, dt, up=None, dT=0.0):
        out = self.p0 + (self.p_i + self.p_ii * di) * di + (self.p_v + self.p_vv * dv) * dv + self.p_t * dt + \
            self.p_T * dT
        if up is not None:
            out = out + self.p_leg * (0.5 - np.asarray(up, dtype=float))
        return out


def _comb_lsq(d: MapData, s, x, offsets, spacing, extra: list, leg) -> tuple:
    """The comb by least squares, its mode numbers from each line's offset at first, from the fit after: (solution:
    spacing, p0, the extra columns', p_leg; residual sum of squares, points, runs: stretches of a sweep on one mode,
    whose residuals go together)."""
    m_num = np.full(len(d.f), np.nan)
    for L, u in enumerate(offsets):
        if not np.isnan(u):
            sl = s & (d.line == L)
            m_num[sl] = np.round((x[sl] - u) / spacing)
    return _lsq(d, s, m_num, extra, leg)


def _lsq(d: MapData, s, m_num, extra: list, leg) -> tuple:
    """The comb by least squares from its points' mode numbers as first guessed (m_num; NaN: none), each time again
    from the fit after: as _comb_lsq."""
    cols = [m_num, np.ones(len(d.f))] + extra + ([leg] if np.any(leg) else [])
    sol = None
    for _ in range(4):
        use = s & ~np.isnan(m_num)
        A = np.column_stack([c[use] for c in cols])
        sol, *_ = np.linalg.lstsq(A, d.f[use], rcond=None)
        base = np.column_stack(cols[1:]) @ sol[1:]
        m_num = np.where(s, np.round((d.f - base) / sol[0]), np.nan)
        cols[0] = m_num
    use = s & ~np.isnan(m_num)
    base = np.column_stack(cols[1:]) @ sol[1:]
    res = d.f[use] - m_num[use] * sol[0] - base[use]
    runs = 0
    for idx in d.sweeps():
        mm = m_num[idx][use[idx]]
        runs += 1 + int(np.count_nonzero(np.diff(mm))) if len(mm) else 0
    bunch = float(abs(np.exp(2j * np.pi * (d.f[use] - base[use]) / sol[0]).mean()))
    return sol, float(np.sum(res ** 2)), int(use.sum()), runs, bunch


def fit_comb(d: MapData, spacing: float, rate: float, i0: float, v0: float, t0: float, mask=None,
             legs: bool = True, drift: bool = True) -> Comb:
    """One comb fitted to the map: each line's offset (where its frequencies bunch, `rate` GHz per unit of the swept
    axis taken off), unwrapped from line to line; each point's mode number from that; then frequency = mode number
    * spacing + quadratic in current and piezo, linear in time, offset by the way it was swept, by least squares
    (iterated: the mode numbers again from the fit). The first line swept again at the end (a line at a piezo
    voltage swept before, later) tells drift from piezo, to whole modes: each of the nearest numbers of modes is
    fitted (as many as a drift of MAX_DRIFT could be), the one fitting best kept, those as good as it within what
    the residuals can tell (they go together along a mode) kept as others. No line swept again: no drift term.
    legs: an offset by the way it was swept (not for the chip comb: sweeping up and down at a current the laser is
    mostly on different chip modes, which that would take for a change of their spacing). drift: whether it drifts at
    all (not the chip comb: its temperature is held, and where frequencies bunch on it, the line swept again later
    on other modes of it, would make a drift of that, and the piezo's tuning its opposite)."""
    s = d.ok if mask is None else (d.ok & mask)
    di, dv, dt = d.i - i0, d.v - v0, (d.t - t0) / UNIT_T
    leg = np.where(d.leg == 2, 0.5, -0.5) if legs else np.zeros(len(d.f))
    x = d.f - rate * (d.fast - np.median(d.fast[s]))
    n_lines = len(d.lines)
    phase = np.full(n_lines, np.nan)
    for L in range(n_lines):
        sl = s & (d.line == L)
        if sl.sum() >= 3:
            phase[L] = np.angle(np.exp(2j * np.pi * x[sl] / spacing).mean()) / (2 * np.pi) * spacing
    slow = np.array([ln["slow"] for ln in d.lines])
    repeat = np.array([bool(ln.get("repeat")) for ln in d.lines])
    unwrapped = phase.copy()
    prev = None
    for L in range(n_lines):
        if np.isnan(phase[L]) or repeat[L]:
            continue
        if prev is not None:
            unwrapped[L] = phase[L] + spacing * round((unwrapped[prev] - phase[L]) / spacing)
        prev = L
    main = ~repeat & ~np.isnan(phase)
    jump = float(np.max(np.abs(np.diff(unwrapped[main]))) / spacing) if main.sum() > 1 else 0.0
    again = np.flatnonzero(repeat & ~np.isnan(phase))
    guess = {}
    if len(again) and main.sum() >= 3:  # (where the main lines put a line swept again at its piezo voltage, had
        a = np.polyfit(slow[main], unwrapped[main], 2)  # nothing drifted)
        guess = {L: phase[L] + spacing * round((np.polyval(a, slow[L]) - phase[L]) / spacing) for L in again}
    drift_known = bool(guess) and drift
    extra = [di, di * di, dv, dv * dv] + ([dt] if drift_known else [])
    tried = []
    span = max((float(np.max(d.t[s & (d.line == L)])) for L in again), default=0.0) - float(np.min(d.t[s]))
    most = int(MAX_DRIFT * span / UNIT_T / spacing) if drift_known else 0
    for k in range(-most, most + 1):
        u = unwrapped.copy()
        for L in guess:
            u[L] = guess[L] + k * spacing
        tried.append(_comb_lsq(d, s, x, u, spacing, extra, leg))
    best = min(tried, key=lambda r: r[1])
    sol, ss, n, runs, bunch = best
    margin = 9.0 * (ss / max(n, 1)) * n / max(runs, 1)  # (3 sigma, the residuals of a run counted once)

    def terms(sol):
        return {"p0": float(sol[1]), "p_v": float(sol[4]), "p_vv": float(sol[5]),
                "p_t": float(sol[6]) if drift_known else 0.0, "p_leg": float(sol[-1]) if legs else 0.0}

    others = [terms(r[0]) for r in sorted(tried, key=lambda r: r[1]) if r is not best and r[1] - ss < margin]
    return Comb(float(sol[0]), p_i=float(sol[2]), p_ii=float(sol[3]), rms=float(math.sqrt(ss / max(n, 1))),
                bunching=bunch, points=n, drift_known=bool(drift_known), others=others, jump=jump, **terms(sol))


# ----------------------------------------------------------------- the rule
_POOL, _POOL_LOCK = None, threading.Lock()


def _pool():
    """The threads its work is shared out among (THREADS of them, made once), or None if it is to be done here: on
    one thread only, or on one of its own (none of them waits on the others)."""
    global _POOL
    if THREADS < 2 or threading.current_thread().name.startswith("laser model"):
        return None
    with _POOL_LOCK:
        if _POOL is None:
            _POOL = ThreadPoolExecutor(THREADS, thread_name_prefix="laser model")
    return _POOL


def _shared(fn, P) -> np.ndarray:
    """fn(Q) for parts Q of P (candidates, a row each), on THREADS threads at once, joined: as fn(P), sooner. Only the
    candidates are shared out, each worked out as alone: the same results, to the last bit."""
    P = np.atleast_2d(P)
    k = min(4 * THREADS, len(P) // 4)  # (more parts than threads: one that takes longer holds none of them up)
    pool = _pool() if k > 1 else None
    if pool is None:
        return fn(P)
    return np.concatenate(list(pool.map(fn, np.array_split(P, k))))


def _losses(modes, mid, dc, lo, hi, soft=5.0):
    """Each mode's loss: sin²(π δ / dc), δ its distance from its chip resonance's best place (mid, the others dc
    apart: sin² repeats every dc, so δ is not folded into one, which took most of a search's time), and more outside
    the window (lo .. hi): (d / soft)², d GHz beyond its edge (its feedback and gain falling off: a mode soft GHz out
    loses as much more as one halfway between chip modes does)."""
    return np.sin(np.pi * (modes - mid) / dc) ** 2 + \
        ((np.maximum(modes - hi, 0.0) + np.maximum(lo - modes, 0.0)) / soft) ** 2


def _compiled_args(pk: dict) -> tuple:
    """pk's arrays as compiled.select and .likelihood take them: each as contiguous as numba wants, one float type
    for all (pk's M's), what is not there zeros (made once, kept in pk)."""
    keys = ("M", "B", "A", "V", "T", "up", "WT", "meas", "valid")
    sources = tuple(pk.get(k) for k in keys)
    kept = pk.get("_compiled")
    got = kept[1] if kept is not None and all(a is b for a, b in zip(kept[0], sources)) else None
    if got is None:  # (made again if any of its arrays was replaced since)
        M = np.ascontiguousarray(pk["M"])
        if M.dtype not in (np.float32, np.float64):
            M = M.astype(float)
        n, L = M.shape[:2]

        def each(key, dtype=M.dtype):
            x = pk.get(key)
            return np.zeros((n, L), dtype=dtype) if x is None else \
                np.ascontiguousarray(np.broadcast_to(np.asarray(x, dtype=dtype), (n, L)))

        got = (M, each("B"), each("A"), each("V"), each("T"), each("up", np.bool_), each("WT"), each("meas", np.int64),
               each("valid", np.bool_), float(pk["ci0"]), float(pk["i0"]), float(pk["v0"]))
        pk["_compiled"] = (sources, got)
    return got


def _select(pk: dict, P, cur, scored: bool = False, forced: bool = False):
    """The rule run along sweeps, all at once, for candidates all at once. pk (LaserModel._pack): M (n, L, K), the
    modes' GHz along each of n sweeps of L points (K modes); B (n, L), the chip comb's middle (GHz), as it moves ci0
    GHz/mA with the current from i0; A, V (n, L): mA, V; T (n, L): s; up (n, L): the swept axis rising there; meas,
    valid (n, L): the modes measured, the points that count; WT (n, L), if there: GHz the temperature moves the
    window (LaserModel.w_T). cur (n,): the mode each sweep begins on (an index into K). P (pop, 12): each candidate's
    rule (LaserModel.rule()). Returns the modes chosen (pop, n, L), or scored: how many valid points each candidate
    put on the mode measured (pop,); forced: each point from the mode measured at the one before (a point at a time:
    one wrong hop does not take the rest of its sweep with it). Compiled (compiled.select) if numba is installed."""
    if compiled.AVAILABLE:
        args = _compiled_args(pk)
        P = np.ascontiguousarray(np.atleast_2d(np.asarray(P, dtype=float)))
        n = args[0].shape[0]
        cur = np.ascontiguousarray(np.broadcast_to(np.asarray(cur, dtype=np.int64), (len(P), n)))
        k = min(4 * THREADS, n // 8) if len(P) <= 4 else 1  # (a few rules along many paths: the paths shared out)
        pool = _pool() if k > 1 else None
        if pool is None:
            right, chosen = compiled.select(*args, P, cur, scored, forced)
            return right if scored else chosen

        def part(ix):
            sl = slice(int(ix[0]), int(ix[-1]) + 1)
            return compiled.select(*(a[sl] if isinstance(a, np.ndarray) else a for a in args), P,
                                   np.ascontiguousarray(cur[:, sl]), scored, forced)

        got = list(pool.map(part, np.array_split(np.arange(n), k)))
        return np.sum([g[0] for g in got], axis=0) if scored else np.concatenate([g[1] for g in got], axis=1)
    M, B, A, V, T, up = (pk[k] for k in ("M", "B", "A", "V", "T", "up"))
    ci0, i0, v0, WT = pk["ci0"], pk["i0"], pk["v0"], pk.get("WT")
    P = np.atleast_2d(np.asarray(P, dtype=M.dtype))
    lo, hi, h_up, h_down, offset, pull, lag, dc, ci, w_v, w_i, soft = (P[:, k][:, None] for k in range(12))
    dc3, soft3 = dc[:, :, None], soft[:, :, None]
    npop, (n, L, _) = len(P), M.shape
    now = np.broadcast_to(np.asarray(cur, dtype=int), (npop, n)).copy()
    warm = np.broadcast_to(A[:, 0], (npop, n)).copy()  # the current the chip is as warm as (it lags)
    ar = np.arange(n)[None, :]
    right = np.zeros(npop)
    chosen = None if scored else np.empty((npop, n, L), dtype=int)
    for j in range(L):
        if j:
            fade = np.exp(-(T[:, j] - T[:, j - 1])[None, :] / np.maximum(lag, 1e-9))
            warm = A[None, :, j] + (warm - A[None, :, j]) * fade
        base = B[None, :, j] + (ci - ci0) * (A[None, :, j] - i0) + offset + ci * (warm - A[None, :, j])
        shift = w_v * (V[None, :, j] - v0) + w_i * (A[None, :, j] - i0)  # (the window, as it moves)
        if WT is not None:  # (and as the temperature moves it)
            shift = shift + WT[None, :, j]
        if j and forced:
            now = np.broadcast_to(pk["meas"][None, :, j - 1], (npop, n)).copy()
        fc = M[ar, j, now]
        mid = base + pull * ((fc - base + dc / 2) % dc - dc / 2)
        g = _losses(M[None, :, j, :], mid[:, :, None], dc3, (lo + shift)[:, :, None], (hi + shift)[:, :, None], soft3)
        if j:  # (at its first point it is on the mode it began on)
            best = np.argmin(g, axis=2)
            h = np.where(up[None, :, j], h_up, h_down)
            g_now = np.take_along_axis(g, now[:, :, None], 2)[:, :, 0]
            g_best = np.take_along_axis(g, best[:, :, None], 2)[:, :, 0]
            now = np.where(g_now > g_best + h, best, now)
        if scored:
            right += ((now == pk["meas"][None, :, j]) & pk["valid"][None, :, j]).sum(axis=1)
        else:
            chosen[:, :, j] = now
    return right if scored else chosen


def _likelihood(pk: dict, P, eps: float = 0.02) -> np.ndarray:
    """The mean log likelihood of what the laser did, a point at a time from the mode measured at the one before,
    for candidates all at once (P: pop x 13, _select's and a temperature, of a chip mode's loss): it goes on lasing,
    or hops to another mode as much as that one is better than the hysteresis asks (a softmax at that temperature),
    or, with eps, anything at all (what the rule cannot tell: a reading put on the wrong mode, a flicker). Smooth,
    and every hop counts: what a search needs to find where the rule is (_fit_rule). Compiled (compiled.likelihood)
    if numba is installed."""
    if compiled.AVAILABLE:
        return compiled.likelihood(*_compiled_args(pk), np.ascontiguousarray(np.atleast_2d(np.asarray(P, dtype=float))),
                                   float(eps))
    M, B, A, V, T, up, meas, valid = (pk[k] for k in ("M", "B", "A", "V", "T", "up", "meas", "valid"))
    ci0, i0, v0, WT = pk["ci0"], pk["i0"], pk["v0"], pk.get("WT")
    P = np.atleast_2d(np.asarray(P, dtype=M.dtype))
    lo, hi, h_up, h_down, offset, pull, lag, dc, ci, w_v, w_i, soft, temp = (P[:, k][:, None] for k in range(13))
    npop, (n, L, K) = len(P), M.shape
    warm = np.broadcast_to(A[:, 0], (npop, n)).copy()
    ar = np.arange(n)
    out = np.zeros(npop)
    for j in range(1, L):
        fade = np.exp(-(T[:, j] - T[:, j - 1])[None, :] / np.maximum(lag, 1e-9))
        warm = A[None, :, j] + (warm - A[None, :, j]) * fade
        base = B[None, :, j] + (ci - ci0) * (A[None, :, j] - i0) + offset + ci * (warm - A[None, :, j])
        shift = w_v * (V[None, :, j] - v0) + w_i * (A[None, :, j] - i0)
        if WT is not None:
            shift = shift + WT[None, :, j]
        was = meas[:, j - 1]
        fc = M[ar, j, was][None, :]
        mid = base + pull * ((fc - base + dc / 2) % dc - dc / 2)
        g = _losses(M[None, :, j, :], mid[:, :, None], dc[:, :, None], (lo + shift)[:, :, None],
                    (hi + shift)[:, :, None], soft[:, :, None])
        h = np.where(up[None, :, j], h_up, h_down)
        z = (g[:, ar, was][:, :, None] - g - h[:, :, None]) / temp[:, :, None]
        z[:, ar, was] = 0.0
        top = z.max(axis=2, keepdims=True)
        p = np.exp(z[:, ar, meas[:, j]] - top[:, :, 0]) / np.exp(z - top).sum(axis=2)
        out += (np.log((1 - eps) * p + eps / K) * valid[None, :, j]).sum(axis=1)
    return out / max(int(valid.sum()), 1)


# ----------------------------------------------------------------- the model
@dataclass
class LaserModel:
    """A laser as two combs and a rule (the module's docstring). Frequencies in GHz from f_ref (THz); the combs'
    terms from i0 (mA), v0 (V), t0 (s); what it says of the map it came from besides (pace: s per unit of its slow
    axis; step_s, step_x: s and mA (or V) from one point of a sweep to the next; again_s: s from its first line to
    that line swept again; hysteresis: mA (or V) its modes sweeping down are from where they were sweeping up, apart:
    the part of the range where the two are on different modes, as the model has it at that pace)."""
    f_ref: float
    i0: float
    v0: float
    t0: float
    ext: Comb
    chip: Comb | None
    lo: float = -math.inf        # GHz: the window, at i0 and v0...
    hi: float = math.inf
    h_up: float = 0.0            # the hysteresis, sweeping up and down (of a chip resonance's loss at its most, 1)
    h_down: float = 0.0
    offset: float = 0.0          # GHz: a chip resonance's best place from its middle
    pull: float = 0.0            # the mode lasing pulls its chip resonance's best place this part of the way to it
    lag: float = 0.0             # s: the chip's temperature lags the current's heating by this
    w_v: float = 0.0             # GHz/V: ...as the piezo moves it (the grating turned as well as moved)...
    w_i: float = 0.0             # GHz/mA: ...and the current (the gain, with the chip's temperature)
    soft: float = 5.0            # GHz: beyond its edges a mode loses (d / soft)² more, d GHz out (_losses)
    i_th: float | None = None    # mA: the threshold, as the power at the wavemeter says it
    slope: float | None = None   # µW/mA at the wavemeter, above it
    power_v: float | None = None  # µW/V: the power's dependence on the piezo at a given current
    power_dp: float | None = None  # µW per GHz a mode sits above its chip resonance's best place (δ)
    right: float = 0.0           # the fraction of the map's points the model puts on the right mode...
    near: float = 0.0            # ...and of those it misses, within two points of a hop the laser made
    null: float | None = None    # the fraction a model that never hops would put on the right mode (each sweep on its
    #                              first point's mode)
    hops_on: float | None = None  # the share of the map's hops that are whole numbers of its external modes
    points: int = 0
    kind: str = "current"
    pace: float | None = None
    step_s: float | None = None
    step_x: float | None = None
    again_s: float | None = None
    hysteresis: float | None = None
    apart: float | None = None
    notes: list = field(default_factory=list)
    i_span: list | None = None   # mA, V: the currents and piezo voltages its map covered (as the model has them)...
    v_span: list | None = None
    v_set_to_act: float = 0.0    # V: ...the piezo as measured less as set (the model's is the measured one)
    temp: float | None = None    # °C: the diode's temperature, as set, when it was mapped
    alt: dict | None = None      # the rule held the other way (_fit_rule's other search): rule()'s terms by name (RULE,
    #                              WINDOW, dc, ci), and the fraction of the map's points it puts on the right mode
    T0: float | None = None      # °C: the temperature its combs' and its window's terms are from (None: mapped at one,
    #                              temp: no temperature terms)
    w_T: float = 0.0             # GHz/K: its window, as the temperature moves it
    temps: list | None = None    # °C: the temperatures its map was made at
    aliases: dict | None = None  # GHz/K: the temperature's other coefficients its map allows as well ('chip', 'ext':
    #                              whole modes more or less per step), and the steps that would tell ('step', K)
    i_th_T: float | None = None  # mA/K: its threshold, as the temperature moves it
    by_temp: list | None = None  # [[°C, the fraction of the points there it puts on the right mode]], each mapped
    version: int = MODEL_VERSION  # its make (this version's; 1: kept by a version before it said)

    # ----------------------------------------------------------- the rule
    @property
    def single(self) -> bool:
        """One selection filter (no chip comb)."""
        return self.chip is None or self.chip.spacing <= 0

    @property
    def dc(self) -> float:
        """GHz: the chip comb's spacing (one filter: as one resonance SINGLE external modes wide would be)."""
        return SINGLE * self.ext.spacing if self.single else self.chip.spacing

    def rule(self) -> np.ndarray:
        """Its rule's parameters as _select has them: RULE's, its chip comb's spacing and GHz/mA, its window's
        motion and its edges' softness (WINDOW)."""
        return np.array([getattr(self, k) for k in RULE] + [self.dc, self._ci()] + [getattr(self, k) for k in WINDOW],
                        dtype=float)

    def doubts(self) -> list:
        """Why it cannot be the laser's model, if it cannot (its map misled its fit: misread frequencies, too few
        hops): sentences; none, it can."""
        out = []
        L = C_LIGHT / (2 * self.ext.spacing * 1e9) * 1e3
        if not LENGTHS[0][0] <= L <= LENGTHS[0][1]:
            out.append(f"its external modes {self.ext.spacing:.4f} GHz apart would make its cavity {L:.2f} mm long "
                       f"(optically): no ECDL's is ({LENGTHS[0][0]:g} to {LENGTHS[0][1]:g} mm)")
        if not self.single:
            Lc = C_LIGHT / (2 * self.chip.spacing * 1e9) * 1e3
            if not LENGTHS[1][0] <= Lc <= LENGTHS[1][1]:
                out.append(f"its chip's modes {self.chip.spacing:.3f} GHz apart would make its chip {Lc:.3f} mm long "
                           f"(optically, n_g L): no laser diode's is ({LENGTHS[1][0]:g} to {LENGTHS[1][1]:g} mm)")
            elif Lc >= L:
                out.append(f"its chip's modes ({self.chip.spacing:.3f} GHz apart) would make its chip longer than its "
                           f"whole cavity ({Lc:.2f} against {L:.2f} mm)")
        if self.hops_on is not None and self.hops_on < HOPS_ON:
            out.append(f"only {self.hops_on:.0%} of its map's hops are whole numbers of its external modes "
                       f"({self.ext.spacing:.4f} GHz): they are not its modes")
        if self.null is not None and self.right < self.null + 0.05:
            out.append(f"it puts no more of the map's points on the right mode ({self.right:.0%}) than one that never "
                       f"hops would ({self.null:.0%})")
        return out

    @property
    def plausible(self) -> bool:
        """It can be the laser's model (doubts())."""
        return not self.doubts()

    def with_rule(self, rule: dict) -> LaserModel:
        """A copy of it with another rule (as alt has it: rule()'s terms by name)."""
        m = copy.deepcopy(self)
        for k in RULE + WINDOW:
            setattr(m, k, float(rule.get(k, getattr(self, k))))  # (a rule kept before its window's edges were soft)
        if not m.single:
            m.chip.spacing = float(rule["dc"])
        if m.chip is not None:
            m.chip.p_i = float(rule["ci"])
        m.alt = None
        return m

    def _d(self, i, v, t):
        return np.asarray(i, dtype=float) - self.i0, np.asarray(v, dtype=float) - self.v0, \
            (np.asarray(t, dtype=float) - self.t0) / UNIT_T

    def _dT(self, T):
        """K from T0: 0 at T0 (T None or NaN), or with no temperature terms."""
        if T is None or self.T0 is None:
            return 0.0
        dT = np.asarray(T, dtype=float) - self.T0
        return np.where(np.isfinite(dT), dT, 0.0)

    def ext_modes(self, i, v, t, orders, up=None, T=None) -> np.ndarray:
        """GHz: the external modes of those orders (whole numbers) at (i, v, t) and T °C (None: T0), sweeping up or
        down (None: between): points x orders."""
        return self.ext.base(*self._d(i, v, t), up, self._dT(T))[..., None] + np.asarray(orders) * self.ext.spacing

    def chip_base(self, i, v, t, T=None):
        """GHz: a chip resonance's middle at (i, v, t) and T °C (None: T0), as the chip comb has it (the others dc
        apart)."""
        if self.chip is None:
            return np.zeros_like(np.asarray(i, dtype=float))
        return self.chip.base(*self._d(i, v, t), dT=self._dT(T))

    def window(self, i, v, T=None) -> tuple:
        """GHz: (its window's low edge, high edge) at (i, v) and T °C (None: T0)."""
        shift = self.w_v * (np.asarray(v, dtype=float) - self.v0) + self.w_i * (np.asarray(i, dtype=float) - self.i0)
        shift = shift + self.w_T * self._dT(T)
        return self.lo + shift, self.hi + shift

    def with_temps(self, chip: float, ext: float) -> LaserModel:
        """A copy of it whose combs move chip and ext GHz/K with the temperature (others its map allows as well:
        aliases), agreeing with it at the first temperature its map was made at (and so, whole modes more or less, at
        the others)."""
        m = copy.deepcopy(self)
        if self.T0 is None:
            return m
        T1 = (self.temps or [self.T0])[0]
        if m.chip is not None:
            m.chip.p0 += (m.chip.p_T - chip) * (T1 - self.T0)
            m.chip.p_T = float(chip)
        m.ext.p0 += (m.ext.p_T - ext) * (T1 - self.T0)
        m.ext.p_T = float(ext)
        return m

    def at(self, T: float | None) -> LaserModel:
        """A copy of it at T °C (None: as it is): its combs and its window where they are there, its terms from there
        (as a model of a map made at T would have them)."""
        m = copy.deepcopy(self)
        if T is None or self.T0 is None or T == self.T0:
            return m
        dT = float(T) - self.T0
        m.ext.p0 += m.ext.p_T * dT
        if m.chip is not None:
            m.chip.p0 += m.chip.p_T * dT
        m.lo, m.hi = m.lo + m.w_T * dT, m.hi + m.w_T * dT
        if m.alt is not None:
            m.alt = dict(m.alt, lo=m.alt["lo"] + m.w_T * dT, hi=m.alt["hi"] + m.w_T * dT)
        if m.i_th is not None and m.i_th_T is not None:
            m.i_th += m.i_th_T * dT
        m.T0 = m.temp = float(T)
        return m

    def orders(self, margin: float = 60.0) -> np.ndarray:
        """The external orders that can lase about the map's frequencies."""
        lo = self.lo if math.isfinite(self.lo) else -600.0
        hi = self.hi if math.isfinite(self.hi) else 600.0
        e = self.ext
        return np.arange(math.floor((lo - margin - abs(e.p0) - 400) / e.spacing),
                         math.ceil((hi + margin + abs(e.p0) + 400) / e.spacing) + 1)

    def _first(self, modes, base, i, v, T=None) -> int:
        """The mode lasing where a path begins at (i, v) and T °C (as if long there): the best, pulled to it."""
        dc = self.dc
        g = self._loss(modes, base + self.offset, i, v, T)
        k = int(np.argmin(g))
        mid = base + self.offset + self.pull * ((modes[k] - base - self.offset + dc / 2) % dc - dc / 2)
        return int(np.argmin(self._loss(modes, mid, i, v, T)))

    def _firsts(self, modes, base, i, v) -> np.ndarray:
        """_first for many paths at once (modes: paths x orders; base, i, v: each path's), at T0."""
        dc = self.dc
        lo, hi = (np.asarray(x, dtype=float).reshape(-1, 1) for x in self.window(i, v))
        mid = (np.asarray(base, dtype=float) + self.offset).reshape(-1, 1)
        k = np.argmin(_losses(modes, mid, dc, lo, hi, self.soft), axis=1)
        near = np.take_along_axis(modes, k[:, None], 1)
        mid = mid + self.pull * ((near - mid + dc / 2) % dc - dc / 2)
        return np.argmin(_losses(modes, mid, dc, lo, hi, self.soft), axis=1)

    def _loss(self, modes, mid, i, v, T=None):
        lo, hi = self.window(i, v, T)
        return _losses(modes, mid, self.dc, lo, hi, self.soft)

    def _ci(self) -> float:
        return self.chip.p_i if self.chip is not None else 0.0

    def _path(self, i, v, t, up, clock=None, dtype=float, T=None) -> dict:
        """One path (arrays i mA, v V, t s, up: the swept axis rising; T °C, None: T0) as _select's pk; clock: s at
        each point for the chip's lag (None: t)."""
        orders = self.orders()
        modes = self.ext_modes(i, v, t, orders, up, T)
        base = self.chip_base(i, v, t, T)
        s = t if clock is None else np.asarray(clock, dtype=float)
        wt = np.broadcast_to(self.w_T * self._dT(T), np.shape(base))
        arr = [np.asarray(x, dtype=dtype)[None] for x in (modes, base, i, v, s, wt)]
        return {"M": arr[0], "B": arr[1], "A": arr[2], "V": arr[3], "T": arr[4], "WT": arr[5],
                "up": np.asarray(up)[None], "ci0": self._ci(), "i0": self.i0, "v0": self.v0, "modes": modes,
                "base": base}

    def simulate(self, i, v, t, start: float | None = None, legs=None, clock=None, T=None) -> np.ndarray:
        """GHz: the frequency along a path (arrays i mA, v V, t s, in order; T °C, one or each point's, None: T0),
        the mode lasing kept until another is better by the hysteresis (h_up where the swept axis rises, h_down where
        it falls, or as `legs` says: 1 up, 2 down). start: GHz it is on at the first point (None: the best mode
        there, as if long there). clock: s at each point for the chip's lag (None: t)."""
        i, v, t = (np.atleast_1d(np.asarray(x, dtype=float)) for x in (i, v, t))
        n = len(i)
        if n == 0:
            return np.zeros(0)
        x = i if self.kind == "current" else v
        if legs is not None:
            up = np.asarray(legs) == 1
        else:
            up = np.concatenate([[True], np.diff(x) >= 0])
            if n > 1:
                up[0] = up[1]
        if T is not None:
            T = np.broadcast_to(np.asarray(T, dtype=float), i.shape)
        pk = self._path(i, v, t, up, clock, T=T)
        modes = pk["modes"]
        cur = int(np.argmin(np.abs(modes[0] - start))) if start is not None else \
            self._first(modes[0], pk["base"][0], i[0], v[0], None if T is None else T[0])
        k = _select(pk, self.rule(), [cur])
        return modes[np.arange(n), k[0, 0]]

    def step(self, i: float, v: float, t: float, state=None, clock: float | None = None, T=None) -> tuple:
        """One point of a path after another: (GHz it lases at, δ GHz from its chip resonance's best place, the state
        for the next). state: what the last point left (None: a fresh start, as if long there); clock: s for the
        chip's lag (None: t); T: °C, the diode's temperature (None: T0)."""
        e, dc = self.ext, self.dc
        now_t = t if clock is None else clock
        if state is None:
            warm, last_t, last_x, up, order = i, now_t, None, True, None
        else:
            order, warm, last_t, last_x, up = state
        x = i if self.kind == "current" else v
        if last_x is not None and x != last_x:
            up = x > last_x
        if self.lag > 0 and now_t > last_t:
            warm = i + (warm - i) * math.exp(-(now_t - last_t) / self.lag)
        elif self.lag <= 0:
            warm = i
        di, dv, dt = self._d(i, v, t)
        b = float(e.base(di, dv, dt, dT=self._dT(T))) + self.leg_now(i, warm, up)
        lo, hi = self.window(i, v, T)
        lo = lo if math.isfinite(lo) else b - 3 * dc
        hi = hi if math.isfinite(hi) else b + 3 * dc
        orders = np.arange(math.floor((lo - 30 - b) / e.spacing), math.ceil((hi + 30 - b) / e.spacing) + 1)
        modes = b + orders * e.spacing
        base = float(self.chip_base(i, v, t, T)) + self.offset + self._ci() * (warm - i)
        if order is None or not (orders[0] <= order <= orders[-1]):
            k = self._first(modes, base - self.offset, i, v, T)
        else:
            k = int(order - orders[0])
            mid = base + self.pull * ((modes[k] - base + dc / 2) % dc - dc / 2)
            g = self._loss(modes, mid, i, v, T)
            best = int(np.argmin(g))
            if g[k] > g[best] + (self.h_up if up else self.h_down):
                k = best
        mid = base + self.pull * ((modes[k] - base + dc / 2) % dc - dc / 2)
        delta = float((modes[k] - mid + dc / 2) % dc - dc / 2)
        return float(modes[k]), delta, (int(orders[k]), warm, now_t, x, up)

    def leg_now(self, i: float, warm: float, up: bool) -> float:
        """GHz its external modes are off where they would be at rest, the current sweeping (p_leg: down less up, at
        the map's pace): as warm as `warm` mA (its chip lagging), they are that much behind; with no lag known, as
        swept up or down."""
        rate = self.step_x / self.step_s if self.step_x and self.step_s else 0.0
        if self.lag > 0 and rate > 0:
            return -0.5 * self.ext.p_leg * (i - warm) / (rate * self.lag)
        return self.ext.p_leg * (0.5 - float(up))

    def _sweeps(self, d: MapData, most: int | None = None) -> list:
        """The map's sweeps (indices of their clean points) that have two at least (`most`: at most that many,
        spread over the map)."""
        sw = [idx[d.ok[idx]] for idx in d.sweeps()]
        sw = [idx for idx in sw if len(idx) >= 2]
        if not sw:
            raise ValueError("no sweep with two clean points")
        if most and len(sw) > most:
            sw = [sw[k] for k in np.unique(np.linspace(0, len(sw) - 1, most).round().astype(int))]
        return sw

    def _pack(self, d: MapData, sw: list, dtype=float) -> dict:
        """Those sweeps as _select's pk, each from its first clean point, padded to the longest (dtype: of the
        arrays; float32 for a search: twice as fast, and as good for it)."""
        n, L = len(sw), max(len(idx) for idx in sw)
        up_pt = d.leg == 1
        orders = self.orders()
        e = self.ext.base(*self._d(d.i, d.v, d.t), up_pt, self._dT(d.T))  # (order 0's mode at each point: the rest
        f = d.f[d.ok]  # whole spacings from it; only those of the sweeps' points worked out)
        span = float(f.max() - f.min()) if len(f) else 0.0
        reach = 0.45 * span + 15.0 if not self.single else 6 * self.ext.spacing  # (no window reaches further)
        step = orders * self.ext.spacing
        keep = (float(np.max(e[d.ok])) + step > f.min() - reach) & (float(np.min(e[d.ok])) + step < f.max() + reach)
        step = step[keep]
        base = self.chip_base(d.i, d.v, d.t, d.T)
        wt = np.broadcast_to(self.w_T * self._dT(d.T), base.shape)
        K = len(step)
        M, B, A, V, T, W = (np.zeros(shape, dtype=dtype) for shape in ((n, L, K), (n, L), (n, L), (n, L), (n, L),
                                                                        (n, L)))
        meas, valid, up = np.zeros((n, L), dtype=int), np.zeros((n, L), dtype=bool), np.zeros((n, L), dtype=bool)
        for s, idx in enumerate(sw):
            k = len(idx)
            modes = e[idx][:, None] + step
            M[s, :k], M[s, k:] = modes, modes[-1]
            B[s, :k], B[s, k:] = base[idx], base[idx[-1]]
            A[s, :k], A[s, k:] = d.i[idx], d.i[idx[-1]]
            V[s, :k], V[s, k:] = d.v[idx], d.v[idx[-1]]
            T[s, :k], T[s, k:] = d.t[idx], d.t[idx[-1]] + np.arange(1, L - k + 1)
            W[s, :k], W[s, k:] = wt[idx], wt[idx[-1]]
            meas[s, :k] = np.argmin(np.abs(modes - d.f[idx][:, None]), axis=1)
            valid[s, 1:k] = True
            up[s, :] = up_pt[idx[0]]
        return {"M": M, "B": B, "A": A, "V": V, "T": T, "WT": W, "meas": meas, "valid": valid, "up": up,
                "ci0": self._ci(), "i0": self.i0, "v0": self.v0}

    def _score(self, P, pk: dict, forced: bool = False) -> np.ndarray:
        """The fraction right of each candidate rule (P: pop x 12, as rule()) on a packed map (forced: a point at a
        time, from the mode measured at the one before)."""
        return _shared(lambda Q: _select(pk, Q, pk["meas"][:, 0], True, forced), P) / max(int(pk["valid"].sum()), 1)

    def score(self, d: MapData) -> float:
        """The fraction of the map's clean points it puts on the mode the laser was on, each sweep simulated from its
        first clean point (as measured)."""
        return self.accuracy(d)[0]

    def accuracy(self, d: MapData) -> tuple:
        """(the fraction of the map's clean points it puts on the mode the laser was on, the fraction of those it
        misses within two points of a hop the laser made: where the model hops a point or two early or late, the
        fraction a model that never hops would put on it: each sweep on the mode of its first point)."""
        right = total = near = null = 0
        sw = self._sweeps(d)
        for a in range(0, len(sw), CHUNK):
            pk = self._pack(d, sw[a:a + CHUNK])
            k = _select(pk, self.rule(), pk["meas"][:, 0])[0]
            v, meas = pk["valid"], pk["meas"]
            hop = np.zeros_like(v)
            hop[:, 1:] = (meas[:, 1:] != meas[:, :-1]) & v[:, 1:]
            close = hop.copy()
            for s in (1, 2):  # (within two points of one, either side)
                close[:, s:] |= hop[:, :-s]
                close[:, :-s] |= hop[:, s:]
            miss = (k != meas) & v
            right += int(((k == meas) & v).sum())
            total += int(v.sum())
            near += int((miss & close).sum())
            null += int(((meas == meas[:, :1]) & v).sum())
        return right / max(total, 1), near / max(total - right, 1), null / max(total, 1)

    def hysteresis_of(self, lo: float, hi: float) -> tuple:
        """(mA (or V): how far its modes sweeping down are from where they were sweeping up, the part of the range
        where the two are on different modes), swept from lo to hi and back at the map's pace, at i0, v0, t0."""
        step = self.step_x or (hi - lo) / 100.0
        xs = np.arange(lo, hi + step / 2, step)
        n = len(xs)
        if n < 4:
            return None, None
        x = np.concatenate([xs, xs[::-1]])
        i, v = (x, np.full(2 * n, self.v0)) if self.kind == "current" else (np.full(2 * n, self.i0), x)
        up = np.arange(2 * n) < n
        pk = self._path(i, v, np.full(2 * n, self.t0), up, np.arange(2 * n) * (self.step_s or 0.3))
        k = _select(pk, self.rule(), [self._first(pk["modes"][0], pk["base"][0], i[0], v[0])])[0, 0]
        ku, kd = k[:n], k[n:][::-1]
        apart = float(np.mean(ku != kd))
        shifts = range(-n // 2, n // 2 + 1)
        agree = [np.mean(ku[max(0, s):n + min(0, s)] == kd[max(0, -s):n - max(0, s)]) for s in shifts]
        s = list(shifts)[int(np.argmax(agree))]
        return float(s * step), apart

    # --------------------------------------------------------- the facts
    def ideal_feedforward(self) -> float | None:
        """mA/V: the current's change per volt of piezo that keeps the combs aligned (no hop, as far as they decide),
        at v0."""
        if self.chip is None or abs(self.chip.p_i - self.ext.p_i) < 1e-6:
            return None
        return (self.ext.p_v - self.chip.p_v) / (self.chip.p_i - self.ext.p_i)

    def facts(self) -> list:
        """What the model says of the laser, in sentences."""
        e, c, nu = self.ext, self.chip, self.f_ref * 1e3  # (nu in GHz)
        single = self.single
        out = []
        L = C_LIGHT / (2 * e.spacing * 1e9) * 1e3
        out.append(f"its external cavity's modes are {e.spacing:.4f} GHz apart: an optical length of {L:.2f} mm from "
                   f"the chip's back facet to the grating (lens and chip included)")
        if not single:
            Lc = C_LIGHT / (2 * c.spacing * 1e9) * 1e3
            nm = C_LIGHT / (self.f_ref * 1e12) * 1e9
            n_g, what = (GROUP_GAN, "GaN") if nm < 560 else (GROUP_III_V, "AlGaInP or GaAs")
            out.append(f"its chip's own modes are {c.spacing:.3f} GHz apart (an optical length n_g·L of {Lc:.3f} mm: "
                       f"{Lc / n_g:.2f} mm of chip at a group index of {n_g:g}, {what} at {nm:.0f} nm): its front "
                       f"facet reflects, so they decide which external mode lases and where it hops (its "
                       f"anti-reflection coating is poor, or there is none); {c.spacing / e.spacing:.3f} external "
                       f"modes to a chip mode")
            out.append(f"so the light goes some {L - Lc:.1f} mm through air and lens from the chip's front facet to "
                       f"the grating")
        else:
            out.append("one selection filter (the grating's and the chip's gain) decides which external mode lases: "
                       "no chip modes, its front facet's anti-reflection coating is good")
        pm_per_ghz = (C_LIGHT / (nu * 1e9)) ** 2 / C_LIGHT * 1e21  # (λ²/c: pm of wavelength per GHz, here)
        if not single:
            out.append(f"the current moves the chip's modes {c.p_i:+.3f} GHz/mA ({-c.p_i * pm_per_ghz:+.2f} pm/mA: the "
                       f"chip heating) and the external modes {e.p_i:+.3f} GHz/mA")
        else:
            out.append(f"the current moves the external modes {e.p_i:+.3f} GHz/mA and the filter {c.p_i:+.3f} GHz/mA"
                       if c is not None else f"the current moves the external modes {e.p_i:+.3f} GHz/mA")
        out += self._piezo_facts(nu, L)
        if not single and abs(c.p_v) < 0.15 * abs(e.p_v):
            out.append(f"the piezo moves the chip's modes {c.p_v:+.3f} GHz/V: next to nothing (it does not touch the "
                       f"chip)")
        elif c is not None:
            out.append(f"the piezo moves the {'filter' if single else 'chip modes'} {c.p_v:+.3f} GHz/V")
        if math.isfinite(self.lo) and math.isfinite(self.hi):
            mid = self.f_ref + (self.lo + self.hi) / 2e3
            e = self.ext
            moves = []
            if abs(self.w_v) >= 0.15 * max(abs(e.p_v), 0.05):
                moves.append(f"the piezo moves it {self.w_v:+.3f} GHz/V (the grating turned as it is pushed: its "
                             f"external modes move {e.p_v:+.3f})")
            else:
                moves.append(f"the piezo hardly moves it ({self.w_v:+.3f} GHz/V)")
            if abs(self.w_i) >= 0.15 * max(abs(self._ci()), 0.1):
                moves.append(f"the current {self.w_i:+.3f} GHz/mA (the gain, as the chip warms)")
            edges = ("" if self.soft <= 6.0 else
                     f"; beyond those edges its feedback and gain fall off softly: a mode {self.soft:.0f} GHz out "
                     f"loses as much more as one halfway between two chip modes, so a chip mode near an edge can "
                     f"still take over where the one inside is between two of its external modes")
            out.append(f"it lases only within {self.lo:+.0f} … {self.hi:+.0f} GHz of {self.f_ref:.6f} THz at "
                       f"{self.v0:.1f} V and {self.i0:.1f} mA ({self.hi - self.lo:.0f} GHz about {mid:.6f} THz, "
                       f"{C_LIGHT / (mid * 1e12) * 1e9:.4f} nm): its grating's feedback and its gain together; "
                       + ", ".join(moves) + edges)
        ff = self.ideal_feedforward()
        if ff is not None and not single:
            step = (c.spacing - e.spacing * round(c.spacing / e.spacing))
            rel, rel_v = c.p_i - e.p_i, e.p_v - c.p_v

            def every(x, unit):
                return f"every {abs(step / x):.2f} {unit} or so" if abs(x) > 1e-6 else "never"

            out.append(f"it hops where an external mode near another chip mode lines up better: the combs slide past "
                       f"each other {rel:+.3f} GHz/mA, and from one chip mode to the next the alignment changes by "
                       f"{abs(step):.2f} GHz, so sweeping the current it hops {every(rel, 'mA')}, sweeping the piezo "
                       f"{every(rel_v, 'V')}")
        if ff is not None:
            along = e.p_v + e.p_i * ff
            gain = along - (self.w_v + self.w_i * ff)  # (on the window, as that moves too)
            reach = (f" without hopping until the window's edge ({(self.hi - self.lo) / max(abs(gain), 1e-9):.0f} V "
                     f"of piezo, at most)" if math.isfinite(self.hi - self.lo) else "")
            out.append(f"the ideal feedforward: {ff:+.3f} mA/V keeps the combs aligned; with it the laser tunes "
                       f"{along:+.3f} GHz/V{reach}")
        out += self._temp_facts(nu)
        out += self._hysteresis_facts(single)
        out += self._power_facts(single)
        out += self._drift_facts(nu, L)
        null = f"; one that never hops would put {self.null:.0%} there" if self.null is not None else ""
        hops = (f"; {self.hops_on:.0%} of the map's hops are whole numbers of its external modes"
                if self.hops_on is not None else "")
        out.append(f"the model puts {self.right:.0%} of the map's {self.points:,} points on the mode the laser was on "
                   f"(each sweep simulated from its first point){null}; {self.near:.0%} of the rest are within two "
                   f"points of a hop: there it hops a point or two early or late{hops}")
        doubts = self.doubts()
        if doubts:  # (first: what the rest is worth)
            out.insert(0, "NOT THIS LASER'S MODEL: its map misled its fit (misread frequencies, too few hops?), so "
                          "nothing goes by it: " + "; ".join(doubts))
        return out + list(self.notes)

    def _temp_facts(self, nu: float) -> list:
        """What its map's temperatures say (none at one temperature)."""
        if self.T0 is None:
            return []
        e, c = self.ext, self.chip
        nm_per_ghz = (C_LIGHT / (nu * 1e9)) ** 2 / C_LIGHT * 1e18  # (λ²/c: nm of wavelength per GHz, here)
        window = not self.single and math.isfinite(self.lo) and math.isfinite(self.hi)
        if not self.single:
            own = e.spacing / c.spacing * c.p_T
            likely, most, least = (k * nu for k in CHIP_T)
            unlike = "" if most <= c.p_T <= least else ", unlike a diode chip's"
            what = (f"its chip's modes {c.p_T:+.2f} GHz/K ({-c.p_T * nm_per_ghz:+.3f} nm/K: the chip's refractive "
                    f"index and length as it warms{unlike}), its external modes {e.p_T:+.2f} GHz/K ({own:+.2f} of "
                    f"it the chip's share of the cavity, {e.p_T - own:+.2f} the rest: " +
                    ("its mount warming with the diode)" if e.p_T - own < 0 else "the cavity shrinking as it warms?)"))
        else:
            what = (f"its filter {c.p_T:+.2f} GHz/K ({-c.p_T * nm_per_ghz:+.3f} nm/K) and its external modes "
                    f"{e.p_T:+.2f} GHz/K")
        if window:
            what += f", its window {self.w_T:+.2f} GHz/K"
        temps = ", ".join(f"{T:g}" for T in self.temps or [])
        text = f"the diode's temperature (mapped at {temps} °C; the model's terms from {self.T0:g} °C) moves " + what
        if not self.single:
            slide = c.p_T - e.p_T
            text += (f": warming it slides its chip's modes {slide:+.2f} GHz/K past its external modes (a chip mode "
                     f"every {c.spacing / max(abs(slide), 1e-9):.2f} K)" +
                     (f" and {c.p_T - self.w_T:+.2f} GHz/K past its window" if window else ""))
            if self.kind == "current" and abs(c.p_T) > 1e-6:
                text += f"; its current warms the chip {c.p_i / c.p_T:.3f} K/mA (its modes {c.p_i:+.3f} GHz/mA)"
        out = [text]
        a = self.aliases or {}
        others = []
        if a.get("chip"):
            others.append("its chip's modes moving " + " or ".join(f"{x:+.2f}" for x in a["chip"][:2]) + " GHz/K")
        if a.get("ext"):
            others.append("its external modes " + " or ".join(f"{x:+.2f}" for x in a["ext"][:2]) + " GHz/K")
        if others:
            gaps = np.diff(self.temps or [])
            apart = f", {float(np.min(gaps)):g} K apart" if len(gaps) else ""
            out.append(f"its temperatures{apart}, are fitted as well by {' and '.join(others)} (whole modes more or "
                       f"less from one to the next): the ones above are what a diode's chip and an ECDL's cavity do, "
                       f"but between the temperatures mapped the model is only as good as that choice; map in steps "
                       f"of {a.get('step') or 0.25:g} K or less to tell")
        if self.by_temp:
            th = ""
            if self.i_th_T is not None:
                th = f"; its threshold moves {self.i_th_T:+.2f} mA/K" + (
                    "" if self.i_th_T > 0 else " as the power at the wavemeter has it (a diode's rises as it warms: "
                                               "that power tells it badly)")
            out.append("of the points at each temperature, it puts on the mode the laser was on " +
                       ", ".join(f"{r:.0%} at {T:g} °C" for T, r in self.by_temp) + th)
        if not e.drift_known:
            out.append("its temperatures were mapped one after another, nothing swept again: a drift of its cavity "
                       "meanwhile would be taken for the temperature's doing")
        return out

    def _piezo_facts(self, nu: float, L: float) -> list:
        e = self.ext

        def nm(p_v):  # (nm the grating moves along the cavity per volt)
            return abs(p_v / nu * L * 1e6)

        alts = [o["p_v"] for o in e.others] if e.drift_known else []
        towards = "nearer the chip" if e.p_v > 0 else "farther from the chip"
        if self.kind == "current" and self.pace:
            as_mapped = e.p_v + e.p_t * self.pace / UNIT_T
            text = (f"the piezo moves the external modes {as_mapped:+.3f} GHz/V as the map went ({self.pace:.0f} s a "
                    f"volt, any drift in it)")
            if e.drift_known:
                text += (f"; less its drift, {e.p_v:+.3f} GHz/V at {self.v0:.1f} V" +
                         (" (or " + ", ".join(f"{p:+.3f}" for p in alts) + ": the drift is not known, below)"
                          if alts else ""))
            text += (f", {2 * e.p_vv:+.4f} GHz/V more per volt (a piezo is not linear): the grating {nm(e.p_v):.1f} "
                     f"nm" + (" (or " + ", ".join(f"{nm(p):.1f}" for p in alts) + ")" if alts else "") +
                     f" {towards} per volt")
            return [text]
        return [(f"the piezo moves the external modes {e.p_v:+.3f} GHz/V at {self.v0:.1f} V ({2 * e.p_vv:+.4f} "
                 f"GHz/V more per volt: a piezo is not linear): the grating {nm(e.p_v):.1f} nm {towards} per volt")]

    def _hysteresis_facts(self, single: bool) -> list:
        unit = "mA" if self.kind == "current" else "V"
        out = []
        if self.hysteresis is not None and self.apart is not None:
            rate = f" ({self.step_x / self.step_s:.2f} {unit}/s)" if self.step_x and self.step_s else ""
            out.append(f"sweeping up and back down at the map's pace{rate} it is on different modes over "
                       f"{self.apart:.0%} of the range: sweeping down, its modes come {abs(self.hysteresis):.2f} "
                       f"{unit} {'lower' if self.hysteresis >= 0 else 'higher'} than sweeping up (its hysteresis)")
        how = []
        if self.lag > 0.01 and self.step_x and self.step_s and self.kind == "current":
            how.append(f"the chip's temperature lags the current by {self.lag:.2f} s (at that pace the chip is as warm "
                       f"as {self.lag * self.step_x / self.step_s:.2f} mA behind the current)")
        if abs(self.pull) >= 0.02:
            pulls = self.pull > 0
            how.append(f"the mode lasing {'pulls' if pulls else 'pushes'} its chip resonance {abs(self.pull):.0%} of "
                       f"the way {'to' if pulls else 'away from'} itself (so a mode, once lasing, "
                       f"{'holds on' if pulls else 'is pushed on'})")
        past = self.past()
        if past is not None:
            ghz, x = past
            how.append(f"and it lases on past where another lines up as well, by {ghz[0]:.2f} GHz sweeping up and "
                       f"{ghz[1]:.2f} GHz down ({x[0]:.2f} and {x[1]:.2f} {unit}: its hysteresis of its own, "
                       f"{self.h_up:.4f} and {self.h_down:.4f} of {'the filter' if single else 'a chip mode'}'s loss "
                       f"at its most)")
        out.append("in the model: " + "; ".join(how))
        a = self.alt
        if a is not None and a.get("right") is not None:
            window = (f", its window moving {a['w_v']:+.3f} GHz/V and {a['w_i']:+.3f} GHz/mA"
                      if math.isfinite(a["lo"]) and math.isfinite(a["hi"]) else "")
            near = a["right"] >= self.right - 0.05
            out.append(f"another rule does {'nearly as well' if near else 'less well'}: its pull {a['pull']:+.0%}, its "
                       f"hysteresis of its own {a['h_up']:.4f} and {a['h_down']:.4f}, its chip lagging {a['lag']:.2f} "
                       f"s{window}; it puts {a['right']:.0%} of the points on the right mode (this one "
                       f"{self.right:.0%})" + (": the map tells the two apart only so far, so its recovery plans for "
                                               "both" if near else ""))
        return out

    def past(self) -> tuple | None:
        """((GHz sweeping up, down), (mA or V, the same)): how far a mode lases on past where the next lines up as
        well, by its hysteresis of its own (the loss near a resonance's best place: (π δ / dc)²; the next mode a
        vernier step away, or a whole external mode with one filter)."""
        e, c = self.ext, self.chip
        if c is None:
            return None
        step = e.spacing if self.single else abs(c.spacing - e.spacing * round(c.spacing / e.spacing))
        rel = (c.p_i - e.p_i) if self.kind == "current" else (c.p_v - e.p_v)
        if step <= 0 or abs(rel) < 1e-9:
            return None
        per = 2 * step * (math.pi / self.dc) ** 2
        ghz = (self.h_up / per, self.h_down / per)
        return ghz, (ghz[0] / abs(rel), ghz[1] / abs(rel))

    def _power_facts(self, single: bool) -> list:
        out = []
        if self.i_th is not None and self.slope is not None:
            out.append(f"the power at the wavemeter: {self.slope:.3f} µW per mA above a threshold of {self.i_th:.1f} "
                       f"mA" + (f" (at a given current, {self.power_v:+.3f} µW/V with the piezo)"
                                if self.power_v is not None else ""))
        if self.power_dp is not None and self.slope and not single:
            c, e = self.chip, self.ext
            per_x = ((e.p_i - c.p_i) if self.kind == "current" else (e.p_v - c.p_v)) * (1.0 - self.pull)
            unit = "mA" if self.kind == "current" else "V"
            on_mode = (self.slope if self.kind == "current" else (self.power_v or 0.0)) + self.power_dp * per_x
            out.append(f"on a mode its power changes {self.power_dp:+.3f} µW per GHz the mode sits higher on its chip "
                       f"resonance: along a mode it goes {on_mode:+.3f} µW/{unit} (not "
                       f"{self.slope if self.kind == 'current' else self.power_v or 0.0:+.3f}), and jumps back at "
                       f"the hops")
            heat = -c.p_i * self.power_dp  # (> 0: its heat pulls a mode's resonance to it; < 0: pushes it away)
            if abs(self.pull) >= 0.02 and heat * self.pull < 0:
                out.append("if the light it gives off is what cools it, a mode giving less light heats the chip and "
                           f"moves its resonance {'away from' if heat < 0 else 'toward'} the mode: that would "
                           f"{'push it on (it would hop sooner)' if heat < 0 else 'hold it'}, the opposite of what the "
                           f"map shows: what holds a mode is not that heat (more likely the light's own effect on the "
                           f"chip, through its gain and its refractive index)")
            elif abs(self.pull) >= 0.02:
                out.append("a mode giving less light heats the chip, and that moves its resonance the way the map "
                           "shows the pull going: its heat could be what holds a mode")
        return out

    def _drift_facts(self, nu: float, L: float) -> list:
        e, c = self.ext, self.chip
        if not e.drift_known:
            return [("its drift is not told apart from the piezo's effect (no line swept again later): it is in "
                     "the piezo's terms")]
        chip = (f"; its chip's modes {c.p_t:+.2f} GHz per 10 min" if c is not None and c.drift_known and not self.single
                else "")
        if e.others:
            when = f" {self.again_s / 60:.0f} min later" if self.again_s else " later"
            return [f"how much its cavity drifted, this map cannot tell: its first line, swept again{when}, was where "
                    f"{e.p_t:+.2f} GHz per 10 min puts it, and as well " +
                    ", ".join(f"{o['p_t']:+.2f}" for o in e.others) +
                    " (whole external modes more or less by then); sweep its first line again sooner, or more often, "
                    "to tell" + chip]
        return [f"it drifted {e.p_t:+.2f} GHz per 10 min (its cavity: {abs(e.p_t / nu * L * 1e6):.0f} nm per 10 min)"
                + chip]

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> LaserModel:
        d = {"version": 1, **d}  # (kept by a version before it said its make: 1)
        d["ext"] = Comb(**d["ext"])
        d["chip"] = Comb(**d["chip"]) if d.get("chip") else None
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


# ------------------------------------------------------------------ the file
def save(path, model: LaserModel):
    """Keep the model in its map's file (meta 'model', and its facts as 'model_facts')."""
    db = sqlite3.connect(str(path), timeout=30.0)
    try:
        with db:
            db.execute("INSERT OR REPLACE INTO meta VALUES (?, ?)", ("model", json.dumps(model.to_dict())))
            db.execute("INSERT OR REPLACE INTO meta VALUES (?, ?)", ("model_facts", json.dumps(model.facts())))
    finally:
        db.close()


def stored(m) -> LaserModel | None:
    """The model kept in a map's file (lasermap.LaserMap), None if none."""
    d = m.meta.get("model")
    return LaserModel.from_dict(d) if isinstance(d, dict) else None


def outdated(m, model: LaserModel) -> str | None:
    """Why a model kept in a map's file (m) is to be fitted to it again though it can be its laser's: an older
    version fitted it (MODEL_VERSION), or it is of one temperature where the file has several; None if neither."""
    if model.version < MODEL_VERSION:
        return "an older version's"
    if model.T0 is None and len({t for t in map_temps(m).values() if t is not None}) > 1:
        return "of one temperature"
    return None


# ---------------------------------------------------------------------- fit
def fit_power(d: MapData, T0: float | None = None) -> tuple:
    """(threshold mA, µW per mA, µW per V at a given current, mA/K the threshold moves with the temperature) from
    the power at the wavemeter where it lased, None each if it cannot tell (the last, too, at one temperature; T0:
    °C the threshold is at, its temperatures about it)."""
    s = d.ok & ~np.isnan(d.power)
    if s.sum() < 20:
        return None, None, None, None
    i, v, p = d.i[s], d.v[s], d.power[s]
    dT = d.T[s] - T0 if T0 is not None else np.zeros(len(p))
    fin = np.isfinite(dT)
    temps = T0 is not None and bool(fin.any()) and float(np.ptp(dT[fin])) > 0
    dT = np.where(fin, dT, 0.0)
    cols = [i, v - np.median(v), np.ones(len(p))] + ([dT] if temps else [])
    X = np.column_stack(cols)
    keep = np.ones(len(p), dtype=bool)
    sol = None
    for _ in range(5):
        sol, *_ = np.linalg.lstsq(X[keep], p[keep], rcond=None)
        res = p - X @ sol
        sd = np.std(res[keep])
        keep = np.abs(res) < 3 * sd
    slope, per_v, c0 = sol[:3]
    if slope <= 0:
        return None, None, None, None
    return float(-c0 / slope), float(slope), float(per_v), float(-sol[3] / slope) if temps else None


def fit_power_dp(d: MapData, model: LaserModel, jump: float = 1.5) -> float | None:
    """µW per GHz: how a mode's power at the wavemeter changes with where it sits on its chip resonance (δ, from
    its best place): along a mode δ moves as the combs slide (e - c per unit of the swept axis, less the part the
    mode's pull drags its resonance along), so its power's slope there, less the trend across modes, over that."""
    if model.single or model.slope is None:
        return None
    e, c = model.ext, model.chip
    per_x, trend = ((e.p_i - c.p_i), model.slope) if d.kind == "current" else ((e.p_v - c.p_v), model.power_v or 0.0)
    per_x *= 1.0 - model.pull
    if abs(per_x) < 1e-6:
        return None
    out = []
    for idx in d.sweeps():
        x, f, p = d.fast[idx], d.f[idx], d.power[idx]
        ok = d.ok[idx] & ~np.isnan(p)
        for a in range(len(idx) - 1):
            if ok[a] and ok[a + 1] and abs(f[a + 1] - f[a]) < jump and x[a + 1] != x[a]:
                out.append((p[a + 1] - p[a]) / (x[a + 1] - x[a]))
    if len(out) < 20:
        return None
    return float((np.median(out) - trend) / per_x)


def fit(m, current: str = "act", map_n: int | None = None, budget: int | None = None, seed: int = 0,
        log=None, tick=None) -> LaserModel:
    """The model of a map's laser (lasermap.LaserMap; map_n: which of its maps, None the last, or all of them if they
    were made at several temperatures: fit_temps), as the module's docstring says. budget: generations of the rule's
    search (None: BUDGET; fewer: faster, coarser). log(text): progress; tick(text): how far it has got, as it goes
    (each generation of the rule's searches: the longest part)."""
    if map_n is None and len({t for t in map_temps(m).values() if t is not None}) > 1:
        return fit_temps(m, current, budget, seed, log, tick)
    say = log or (lambda text: None)
    d = data_of(m, current, map_n=map_n)
    if d.ok.sum() < 50:
        raise ValueError("too few points with a frequency for a model")
    i0, v0 = float(np.median(d.i[d.ok])), float(np.median(d.v[d.ok]))
    t0 = float(np.median(d.t[d.ok]))
    _, rough, ext, chip = _combs(d, i0, v0, t0, say)
    if chip is None:  # one selection filter: the lasing frequencies' middle, as it moves
        chip = _single_filter(d, i0, v0, t0)
    model = LaserModel(d.f_ref, i0, v0, t0, ext, chip, kind=d.kind)
    temps = d.temps()
    model.temp = temps[0] if len(temps) == 1 else (m.meta.get("start") or {}).get("temp_set")
    _finish(model, m, d, rough, BUDGET if budget is None else budget, seed, tick=tick)
    say(f"right mode at {model.right:.0%} of the points")
    return model


def _combs(d: MapData, i0: float, v0: float, t0: float, say) -> tuple:
    """(GHz per unit of the swept axis on one mode, the external comb's spacing as first found, the external comb,
    the chip comb: None if the hops are not whole numbers of one's spacing) of a map at one temperature, their terms
    from i0, v0, t0 (steps 1 to 3)."""
    rate = onmode_rate(d)
    say(f"on one mode: {rate:+.3f} GHz per {'mA' if d.kind == 'current' else 'V'}")
    # the chip comb, if the hops are whole numbers of a large quantum; the external comb below it
    few = d.thinned(THIN)
    rough, _ = comb_spacing(few, rate, 2.0, 40.0)
    quantum = hop_quantum(d, rough)
    chip = None
    if quantum is not None and quantum > 4 * rough:
        chip_rate = _chip_rate(few, quantum)
        dc, _ = comb_spacing(few, chip_rate, 0.97 * quantum, 1.03 * quantum)
        chip = fit_comb(d, dc, chip_rate, i0, v0, t0, legs=False, drift=False)
        say(f"chip comb: {chip.spacing:.3f} GHz, {chip.p_i:+.3f} GHz/mA")
        de, _ = comb_spacing(few, rate, 2.0, min(chip.spacing / 4.5, 40.0))
        small = min(1.0, 0.4 * rough)
        on, on_rough = hops_on(d, de, small, 0.25 * min(de, rough)), hops_on(d, rough, small)
        if on is not None and on_rough is not None and on_rough > on + 0.1:  # (its hops say which is its comb)
            say(f"external comb: {de:.4f} GHz has {on:.0%} of the hops on it, {rough:.4f} GHz {on_rough:.0%}: that")
            de = rough
    else:
        de = rough
    ext = fit_comb(d, de, rate, i0, v0, t0)
    others = ", ".join(f"{o['p_t']:+.2f}" for o in ext.others)
    say(f"external comb: {ext.spacing:.4f} GHz, {ext.p_i:+.3f} GHz/mA, {ext.p_v:+.3f} GHz/V, drift {ext.p_t:+.2f} "
        f"GHz per 10 min" + (f" (or {others})" if others else ""))
    return rate, rough, ext, chip


def _finish(model: LaserModel, m, d: MapData, rough: float, budget: int, seed: int, window=None, most: int = 120,
            tick=None):
    """The rest of a model whose combs are fitted (fit, fit_temps): what its map says besides, its rule (window: the
    window's guess, as _window_guess has it; None: that; at several temperatures, its chip's modes' temperature
    coefficient again from where it hops at each: _chip_temp), how well it does, its power, its hysteresis.
    tick(text): how far it has got."""
    tick = tick or (lambda text: None)
    model.pace, model.step_s, model.step_x, model.again_s = pace_of(d)
    model.i_span = [float(np.min(d.i[d.ok])), float(np.max(d.i[d.ok]))]
    model.v_span = [float(np.min(d.v[d.ok])), float(np.max(d.v[d.ok]))]
    model.v_set_to_act = _v_set_to_act(m)
    ext = model.ext
    if ext.jump > 0.42:
        slow = [ln["slow"] for ln in d.lines if not ln.get("repeat")]
        step = float(np.median(np.abs(np.diff(slow)))) if len(slow) > 1 else 0.0
        unit = "V" if d.kind == "current" else "mA"
        model.notes.append(f"its lines are far apart for this: from one to the next its external modes moved up to "
                           f"{ext.jump:.0%} of their spacing (half and it cannot tell which way), so how its "
                           f"{'piezo' if unit == 'V' else 'current'} "
                           f"tunes them could be another, a whole mode per line more or less ({ext.spacing / step:.2f} "
                           f"GHz/{unit} apart): map with smaller steps of it to be sure" if step else
                           "its lines are far apart for this: map with smaller steps to be sure")
    _fit_rule(model, d, budget, seed, most, window=window, tick=tick)
    if model.T0 is not None and len(d.temps()) > 1:
        tick("its chip's modes at each temperature, from where it hops")
        _chip_temp(model, d)
    tick("how well it does, over the whole map")
    model.right, model.near, model.null = model.accuracy(d)
    model.hops_on = hops_on(d, ext.spacing, min(1.0, 0.4 * rough), 0.25 * min(ext.spacing, rough))
    if model.alt is not None:
        model.alt["right"] = float(model.with_rule(model.alt).accuracy(d)[0])
    model.points = int(d.ok.sum())
    model.i_th, model.slope, model.power_v, model.i_th_T = fit_power(d, model.T0)
    model.power_dp = fit_power_dp(d, model)
    fast = d.fast[d.ok]
    model.hysteresis, model.apart = model.hysteresis_of(float(np.percentile(fast, 2)), float(np.percentile(fast, 98)))


def fit_temps(m, current: str = "act", budget: int | None = None, seed: int = 0, log=None, tick=None) -> LaserModel:
    """The model of a laser mapped at several temperatures (a map at each, as lasermap.MapPlan's t_step makes them),
    as the module's docstring says: each temperature's combs fitted alone, the temperature's coefficients from where
    they are at each (_chip_coef, _ext_coef), the combs fitted again over all of them at once (_joint_comb); the
    window at each, and how it moves; the rule over every temperature's sweeps at once. With fewer than two
    temperatures whose combs agree: the last map's model."""
    say = log or (lambda text: None)
    d = data_of(m, current, map_n="all")
    if d.ok.sum() < 50:
        raise ValueError("too few points with a frequency for a model")
    i0, v0 = float(np.median(d.i[d.ok])), float(np.median(d.v[d.ok]))
    t0 = float(np.median(d.t[d.ok]))
    temps = {n: float(T) for n, T in map_temps(m).items() if T is not None}
    exts, chips, roughs, each = {}, {}, {}, {}
    for n, T in sorted(temps.items()):
        dn = data_of(m, current, map_n=n)
        if dn.ok.sum() < 50:
            say(f"{T:.3f} °C: too few points with a frequency, left out")
            continue
        say(f"{T:.3f} °C (map {n + 1}):")
        _, roughs[n], exts[n], chip = _combs(dn, i0, v0, t0, say)
        each[n] = dn
        if chip is not None:
            chips[n] = chip
    de = float(np.median([e.spacing for e in exts.values()])) if exts else 0.0
    exts = {n: e for n, e in exts.items() if abs(e.spacing - de) < 0.01 * de}  # (one comb: they agree to 0.1 % or so)
    single = len([n for n in chips if n in exts]) <= len(exts) / 2  # (most temperatures' hops say a chip comb, or not)
    if not single:
        dc = float(np.median([chips[n].spacing for n in exts if n in chips]))
        chips = {n: c for n, c in chips.items() if n in exts and abs(c.spacing - dc) < 0.02 * dc}
        exts = {n: e for n, e in exts.items() if n in chips}
    if len({temps[n] for n in exts}) < 2:
        last = max(each) if each else None
        say("fewer than two temperatures whose combs agree: the last map's model")
        return fit(m, current, last, budget, seed, log)
    Tn = {n: temps[n] for n in exts}
    T0 = round(0.5 * (min(Tn.values()) + max(Tn.values())), 3)
    nu = d.f_ref * 1e3
    if single:
        filters = {n: _single_filter(each[n], i0, v0, t0) for n in exts}
        x = np.array([Tn[n] - T0 for n in filters])
        c_T = float(np.polyfit(x, [filters[n].p0 for n in filters], 1)[0])
        c_alts, share = [], None
        chip = _single_filter(d, i0, v0, t0, T0)
    else:
        c_T, c_alts = _chip_coef({n: chips[n].p0 for n in chips}, Tn, dc, nu)
        chip = _joint_comb(d, chips, c_T, T0, i0, v0, t0, legs=False, drifts=False)
        share = de / dc
    e_T, e_alts = _ext_coef({n: exts[n].p0 for n in exts}, Tn, de, nu, None if single else c_T, share)
    ext = _joint_comb(d, exts, e_T, T0, i0, v0, t0)
    say(f"the temperature: the chip's modes {chip.p_T:+.2f} GHz/K, the external modes {ext.p_T:+.2f} GHz/K"
        + (f" (or the chip's {', '.join(f'{c:+.2f}' for c in c_alts[:2])}, the external "
           f"{', '.join(f'{c:+.2f}' for c in e_alts[:2])}: whole modes more or less per step)" if c_alts or e_alts
           else ""))
    model = LaserModel(d.f_ref, i0, v0, t0, ext, chip, kind=d.kind, T0=T0, temp=T0)
    model.temps = sorted(set(Tn.values()))
    model.aliases = {"chip": c_alts, "ext": e_alts, "step": _telling_step(de, None if single else dc, nu, share)}
    window = None
    if not single:  # (where it lased at each temperature: the window there; how that moves with it)
        r = np.array([(Tn[n] - T0, *_window_guess(each[n], model)) for n in exts])
        A = np.column_stack([np.ones(len(r)), r[:, 0]])
        (lo0, w_lo), (hi0, w_hi) = (np.linalg.lstsq(A, r[:, k], rcond=None)[0] for k in (1, 2))
        model.w_T = float(0.5 * (w_lo + w_hi))
        window = (float(lo0), float(hi0), float(np.median(r[:, 3])), float(np.median(r[:, 4])))
        say(f"the window: {model.w_T:+.2f} GHz/K")
    keep = np.isin(d.maps(), list(exts))
    d = d.only(keep)
    _finish(model, m, d, min(roughs[n] for n in exts), BUDGET if budget is None else budget, seed, window,
            max(120, 40 * len(Tn)), tick)
    model.by_temp = [[T, float(model.score(d.only(d.T == T)))] for T in model.temps]
    say(f"right mode at {model.right:.0%} of the points (" +
        ", ".join(f"{r:.0%} at {T:g} °C" for T, r in model.by_temp) + ")")
    return model


def _chip_temp(model: LaserModel, d: MapData, most: int = 60, grid: int = 96):
    """Its chip's modes' temperature coefficient again, from where it hops (its rule fitted over every temperature):
    at each temperature, its chip's modes moved (its rule's offset, a chip mode around: grid steps, then finer about
    the best) to where that temperature's sweeps put most points on the mode the laser was on; the coefficient
    corrected by the line through those, its middle taken into the offset (the rule held the other way's too).
    Where the lasing frequencies bunch on the chip's modes (fit_temps' first guess) is biased near the window's
    edges, and differently at each temperature, as the chip's modes move past the window."""
    dc, R = model.dc, model.rule()
    rows = []
    for T in d.temps():
        dt_ = d.only(d.T == T)
        if dt_.ok.sum() < 50:
            continue
        pk = model._pack(dt_, model._sweeps(dt_, most), np.float32)
        shifts = np.linspace(-dc / 2, dc / 2, grid, endpoint=False)
        for width in (dc / grid, dc / grid / 8):  # (a chip mode around, then finer about the best)
            P = np.repeat(R[None], len(shifts), axis=0)
            P[:, 4] += shifts
            sc = model._score(P, pk)
            top = shifts[int(np.argmax(sc))]
            if width > dc / grid / 2:
                shifts = top + np.linspace(-width, width, 17)
        rows.append((T - model.T0, float(top), float(sc.max())))
    if len(rows) < 2:
        return
    x, y = np.array([r[0] for r in rows]), np.array([r[1] for r in rows])
    b, a = np.polyfit(x, y, 1)
    if model.chip is not None:
        model.chip.p_T += float(b)
    model.offset += float(a)
    if model.alt is not None:
        model.alt["offset"] = model.alt["offset"] + float(a)


def _aliases(p0s: dict, temps: dict, spacing: float, lo: float, hi: float) -> list:
    """GHz/K: the coefficients from lo to hi that put a comb's offsets at its temperatures (p0s, temps: by map; each
    known only to whole spacings) on one line as well as the best does, or nearly (how well: |mean exp(2πi (p0 - c
    T) / spacing)|, 1 at best, within 0.05 of the best's); [[c, how well]], in order."""
    keys = list(p0s)
    T = np.array([temps[n] for n in keys], dtype=float)
    p = np.array([p0s[n] for n in keys], dtype=float)
    T -= T.mean()
    width = spacing / max(float(np.ptp(T)), 1e-6)  # (about a peak's width)
    cs = np.arange(lo, hi, width / 400)
    z = np.abs(np.exp(2j * np.pi * (p[None, :] - cs[:, None] * T[None, :]) / spacing).mean(axis=1))
    peaks = np.flatnonzero((z[1:-1] >= z[:-2]) & (z[1:-1] > z[2:])) + 1
    top = float(z.max()) if len(z) else 0.0
    return [[float(cs[k]), float(z[k])] for k in peaks if z[k] >= top - 0.05]


def _chip_coef(p0s: dict, temps: dict, dc: float, nu: float) -> tuple:
    """(GHz/K: how its chip's modes move with the temperature, the others its map allows as well, nearest first):
    of the coefficients that put its temperatures' chip combs on one line (_aliases), the nearest what a diode chip's
    modes do (CHIP_T: of nu GHz, its frequency)."""
    likely, most = CHIP_T[0] * nu, CHIP_T[1] * nu
    found = sorted((c for c, _ in _aliases(p0s, temps, dc, 2 * most, -most)), key=lambda c: abs(c - likely))
    if not found:
        return likely, []
    return found[0], found[1:]


def _ext_coef(p0s: dict, temps: dict, de: float, nu: float, c_T: float | None, share: float | None) -> tuple:
    """(GHz/K: how its external modes move with the temperature, the others its map allows as well, nearest first):
    of the coefficients that put its temperatures' external combs on one line (_aliases), the nearest the chip's
    share of the cavity (share: de/dc, of c_T), the rest of the cavity no more than an aluminium mount at the
    diode's temperature would expand it (MOUNT_T), nor shrinking as it warms; with one selection filter (its chip's
    share not known) the nearest the middle of that, a quarter of the cavity the chip's."""
    if c_T is not None and share:
        own, rest = share * c_T, MOUNT_T * nu * (1.0 - share)
        aim = own
    else:
        own, rest = 0.25 * CHIP_T[0] * nu, 0.75 * MOUNT_T * nu
        aim = own - 0.5 * rest
    period = de / max(float(np.ptp(list(temps.values()))) / max(len(set(temps.values())) - 1, 1), 1e-6)
    found = [c for c, _ in _aliases(p0s, temps, de, own - rest - 2 * period, own + 2 * period)]
    if not found:
        return aim, []
    band = [c for c in found if own - rest - 0.05 * rest <= c <= own + 0.05 * rest]
    best = min(band or found, key=lambda c: abs(c - aim))
    return best, sorted((c for c in found if c != best), key=lambda c: abs(c - best))


def _telling_step(de: float, dc: float | None, nu: float, share: float | None) -> float:
    """K: the temperature's steps in which only one coefficient of each comb is what a diode's chip and an ECDL's
    cavity can do (_chip_coef, _ext_coef): a little finer than its spacing over the range of those."""
    rest = MOUNT_T * nu * (1.0 - (share or 0.25))
    steps = [de / (1.1 * rest)]
    if dc:
        steps.append(dc / ((CHIP_T[2] - CHIP_T[1]) * nu))
    return math.floor(0.8 * min(steps) * 20) / 20


def _line_through(p0s: dict, temps: dict, spacing: float, coef: float, T0: float) -> float:
    """GHz: at T0, where the line coef GHz/K through a comb's offsets at its temperatures (each known only to whole
    spacings) is."""
    z = np.mean([np.exp(2j * np.pi * (p0s[n] - coef * (temps[n] - T0)) / spacing) for n in p0s])
    return float(np.angle(z) / (2 * np.pi) * spacing)


def _joint_comb(d: MapData, combs: dict, coef: float, T0: float, i0: float, v0: float, t0: float,
                legs: bool = True, drifts: bool = True) -> Comb:
    """One comb over a map's temperatures (d: all of them): each point's mode number from its own temperature's comb
    (combs: by map, each fitted to that map alone), whole modes more or less as the line coef GHz/K puts their
    offsets on has them (_line_through); then fitted again over all of them at once by least squares, with
    fit_comb's terms and p_T (GHz/K, from T0). Its drift, if a line swept again told it (the last temperature's),
    is kept as that told it, not fitted again: the temperatures were mapped one after another, so over them a drift
    and the temperature's doing are one and the same. drifts: whether it drifts at all (not the chip's modes: its
    temperature is held, and the drift its own fit finds is where frequencies bunch on it, not where it is)."""
    spacing = float(np.median([c.spacing for c in combs.values()]))
    maps = d.maps()
    Tm = {n: float(np.nanmedian(d.T[maps == n])) for n in combs}
    a = _line_through({n: c.p0 for n, c in combs.items()}, Tm, spacing, coef, T0)
    di, dv, dt = d.i - i0, d.v - v0, (d.t - t0) / UNIT_T
    up = d.leg == 1
    s = d.ok & np.isin(maps, list(combs))
    m_num = np.full(len(d.f), np.nan)
    for n, c in combs.items():
        sel = s & (maps == n)
        base = c.base(di[sel], dv[sel], dt[sel] if drifts else 0.0, up[sel] if legs else None) - c.p0 + a + \
            coef * (Tm[n] - T0)
        m_num[sel] = np.round((d.f[sel] - base) / spacing)  # (its own drift in it, if it knew one)
    known = [c.p_t for c in combs.values() if c.drift_known] if drifts else []
    p_t = float(np.median(known)) if known else 0.0
    dd = copy.copy(d)
    dd.f = d.f - p_t * dt  # (its drift taken off, as told)
    dT = np.where(np.isfinite(d.T), d.T - T0, 0.0)
    extra = [di, di * di, dv, dv * dv, dT]
    leg = np.where(d.leg == 2, 0.5, -0.5) if legs else np.zeros(len(d.f))
    sol, ss, n, _, bunch = _lsq(dd, s, m_num, extra, leg)
    return Comb(float(sol[0]), p0=float(sol[1]), p_i=float(sol[2]), p_ii=float(sol[3]), p_v=float(sol[4]),
                p_vv=float(sol[5]), p_T=float(sol[6]), p_t=p_t, p_leg=float(sol[-1]) if legs else 0.0,
                rms=float(math.sqrt(ss / max(n, 1))), bunching=bunch, points=n, drift_known=bool(known),
                jump=max(c.jump for c in combs.values()))


def _v_set_to_act(m) -> float:
    """V: the piezo voltage as measured less as set, over a map's points (0 where it was not measured)."""
    diff = [q["v_act"] - q["v"] for q in m.points if q.get("v_act") is not None and q.get("v") is not None]
    return float(np.median(diff)) if diff else 0.0


def _chip_rate(d: MapData, quantum: float) -> float:
    """GHz per unit of the swept axis the chip's modes move: where the frequencies, that taken off, bunch best at the
    quantum, line by line."""
    rates = np.linspace(-15.0, 15.0, 601)
    s = d.ok
    x0 = d.fast[s] - np.median(d.fast[s])
    order, starts = _grouped(d.line[s])
    f, x0 = d.f[s][order], x0[order]
    b = np.concatenate([_bunched(f[None, :] - rates[a:a + 64, None] * x0[None, :], starts, 1.0 / quantum, len(f))
                        for a in range(0, len(rates), 64)])
    return float(rates[int(np.argmax(b))])


def _single_filter(d: MapData, i0, v0, t0, T0: float | None = None) -> Comb:
    """One selection filter: the lasing frequencies' middle, quadratic in current and piezo (and linear in the
    temperature from T0, if given), by least squares (a comb of spacing 0)."""
    s = d.ok
    di, dv = d.i[s] - i0, d.v[s] - v0
    cols = [np.ones(s.sum()), di, di * di, dv, dv * dv]
    if T0 is not None:
        cols.append(np.where(np.isfinite(d.T[s]), d.T[s] - T0, 0.0))
    A = np.column_stack(cols)
    sol, *_ = np.linalg.lstsq(A, d.f[s], rcond=None)
    res = d.f[s] - A @ sol
    return Comb(0.0, float(sol[0]), float(sol[1]), float(sol[3]), float(sol[4]), 0.0, float(np.sqrt(np.mean(res ** 2))),
                1.0, int(s.sum()), False, float(sol[2]), p_T=float(sol[5]) if T0 is not None else 0.0)


HOLDS = (((-0.5, 0.25), 0.05), ((0.15, 0.85), 0.015))  # the rule's searches: (its pull, its own hysteresis at most):
#                                                         held by a hysteresis of its own, and pulled hard instead


def _window_guess(d: MapData, model: LaserModel) -> tuple:
    """(its low edge and high edge at i0 and v0 (GHz), GHz/V, GHz/mA): where the window is and how it moves, guessed
    from where the laser lased: each line's range (its 2nd to 98th percentile) against the slow axis, and within
    lines the frequencies against the swept axis (each line's own mean taken off)."""
    s = d.ok
    f, fast, line = d.f[s], d.fast[s], d.line[s]
    count = np.maximum(np.bincount(line), 1)
    dx = fast - (np.bincount(line, fast) / count)[line]
    w_fast = float((dx * (f - (np.bincount(line, f) / count)[line])).sum() / max(float((dx * dx).sum()), 1e-9))
    rows = []
    for L, ln in enumerate(d.lines):
        sl = s & (d.line == L)
        if not ln.get("repeat") and sl.sum() >= 10:
            rows.append((ln["slow"], *np.percentile(d.f[sl], [2, 98])))
    slow0 = model.v0 if d.kind == "current" else model.i0
    if len(rows) >= 3 and np.ptp([r[0] for r in rows]) > 0:
        r = np.array(rows)
        A = np.column_stack([np.ones(len(r)), r[:, 0] - slow0])
        (lo, w_lo), (hi, w_hi) = (np.linalg.lstsq(A, r[:, k], rcond=None)[0] for k in (1, 2))
        w_slow = 0.5 * (w_lo + w_hi)
    else:
        (lo, hi), w_slow = np.percentile(f, [1, 99]), 0.0
    w_v, w_i = (w_slow, w_fast) if d.kind == "current" else (w_fast, w_slow)
    return float(lo), float(hi), float(w_v), float(w_i)


def _fit_rule(model: LaserModel, d: MapData, budget: int, seed: int, most: int = 120, popsize: int = 15,
              window=None, tick=None):
    """The rule (RULE: the window, the hysteresis each way, the chip resonances' best place, its pull to the mode
    lasing, the chip's lag; WINDOW: how the window moves, how soft its edges), and the chip comb's spacing and current
    tuning again (by
    where it hops: where the frequencies lie on its resonances misleads, near the window's edges, as a mode lases
    only on their inner side there), by differential evolution, every candidate of a generation and every sweep at
    once (`most` sweeps at most, spread over the map), in two stages: the rule that makes what the laser did
    likeliest, a point at a time (_likelihood: smooth, and every hop counts, so the search finds where the rule is),
    then from there and from a plain guess (both among its first population) the rule that puts most points on the
    mode the laser was on, each sweep simulated from its first point (what the model is for). A hysteresis of its
    own and a mode's pull on its chip resonance both hold a mode: such a search settles on one or the other as it
    goes, so it is searched twice, once for each (HOLDS), the better kept. window: the window's guess (as
    _window_guess has it; None: that). The edges' softness is searched as its logarithm (hard and soft alike tried:
    most maps cannot tell much of it). tick(text): how far it has got, a generation at a time. Its candidates are
    worked out on THREADS threads at once (_shared)."""
    from scipy.optimize import differential_evolution

    tick = tick or (lambda text: None)
    searches = 1 + len(HOLDS)

    def going(k, n):  # (search k's progress, a generation at a time)
        done = [0]
        tick(f"the rule: search {k} of {searches}, beginning")

        def callback(x, convergence=None):
            done[0] += 1
            tick(f"the rule: search {k} of {searches}, generation {done[0]} of {n}")
        return callback

    sw = model._sweeps(d, most)
    pk = model._pack(d, sw, np.float32)
    pk_likely = model._pack(d, sw[::2], np.float32) if len(sw) >= 80 else pk  # (the first, coarse search: on every
    #                                                                           other sweep, twice as fast)
    lo0, hi0, wv0, wi0 = _window_guess(d, model) if window is None else window
    span = max(hi0 - lo0, model.ext.spacing)
    half = 0.5 * model.ext.spacing
    dc0, ci0 = model.dc, model._ci()
    ev = abs(model.ext.p_v)
    bounds = [(lo0 - 0.4 * span, lo0 + 0.3 * span), (hi0 - 0.3 * span, hi0 + 0.4 * span), (0.0, 0.05), (0.0, 0.05),
              (-half, half), (-0.5, 0.85), (0.0, max(2.0, 6 * (model.step_s or 0.3))),
              (0.98 * dc0, 1.02 * dc0), (ci0 - 0.1 * abs(ci0) - 0.1, ci0 + 0.1 * abs(ci0) + 0.1),
              (wv0 - 0.5 * ev - 0.2, wv0 + 0.5 * ev + 0.2), (wi0 - 0.5 * abs(ci0) - 0.3, wi0 + 0.5 * abs(ci0) + 0.3),
              (math.log(2.0), math.log(max(10.0, 2.0 * span)))]  # (its edges hard, or soft as far as twice its width)
    guess = [lo0 - 0.05 * span, hi0 + 0.05 * span, 0.005, 0.005, 0.0, 0.2, model.step_s or 0.3, dc0, ci0, wv0, wi0,
             math.log(5.0)]
    if model.single:  # (one filter: no window, its width as it is)
        bounds[0], bounds[1], bounds[7] = (-1e5, -1e5 + 1), (1e5 - 1, 1e5), (dc0 * (1 - 1e-9), dc0 * (1 + 1e-9))
        bounds[9], bounds[10], bounds[11] = (-1e-9, 1e-9), (-1e-9, 1e-9), (math.log(5.0), math.log(5.0) + 1e-9)
        guess[0], guess[1], guess[9], guess[10] = bounds[0][0], bounds[1][1], 0.0, 0.0

    def rules(P):  # (P: parameters x candidates; the softness as its logarithm)
        Q = np.array(P, dtype=float).T
        Q[:, 11] = np.exp(Q[:, 11])
        return Q

    def cost(P):
        return -model._score(rules(P), pk)

    how = {"tol": 1e-6, "polish": False, "init": "sobol", "vectorized": True, "updating": "deferred"}

    def unlikely(P):
        return -_shared(lambda Q: _likelihood(pk_likely, Q), rules(P))

    likely = differential_evolution(unlikely, bounds + [(1e-4, 0.02)], maxiter=max(budget // 2, 5),
                                    popsize=max(popsize - 3, 5), seed=seed, x0=guess + [0.003],
                                    callback=going(1, max(budget // 2, 5)), **how)
    rng = np.random.default_rng(seed)
    found = []
    for k, (pulls, most_h) in enumerate(HOLDS):  # (each its own population: the likeliest rule, the guess among it)
        part = list(bounds)
        part[2] = part[3] = (0.0, most_h)
        part[5] = pulls
        lows, highs = np.array(part).T
        first = lows + (highs - lows) * rng.random((popsize * len(part), len(part)))
        first[0], first[1] = np.clip(likely.x[:12], lows, highs), np.clip(guess, lows, highs)
        found.append(differential_evolution(cost, part, maxiter=budget, popsize=popsize, seed=seed + k,
                                            callback=going(k + 2, budget), **dict(how, init=first)))
    res, *others = sorted(found, key=lambda r: r.fun)
    model.alt = None
    if others:  # (the other way of holding a mode: kept as well)
        x = [float(v) for v in others[0].x]
        x[11] = math.exp(x[11])
        model.alt = dict(zip(RULE + ("dc", "ci") + WINDOW, x))
        if model.single:
            model.alt.update(lo=-math.inf, hi=math.inf, dc=model.dc, w_v=0.0, w_i=0.0, soft=5.0)
    for k, x in zip(RULE, res.x):
        setattr(model, k, float(x))
    model.w_v, model.w_i, model.soft = float(res.x[9]), float(res.x[10]), float(math.exp(res.x[11]))
    if model.single:
        model.lo, model.hi, model.w_v, model.w_i, model.soft = -math.inf, math.inf, 0.0, 0.0, 5.0
    else:
        model.chip.spacing = float(res.x[7])
    if model.chip is not None:
        model.chip.p_i = float(res.x[8])
