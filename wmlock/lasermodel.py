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
- A window, the grating's feedback and the chip's gain together, outside which nothing lases (lo .. hi GHz).
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
4. the rule (window, hysteresis, offset, lag, pull), and the chip comb's spacing and current tuning again (where
   it hops tells them better than where its frequencies lie): every sweep simulated from its first point, scored
   by how many of its points the model puts on the mode the laser was on (differential evolution, all candidates
   and sweeps at once), from the rule that makes what the laser did likeliest a point at a time (_likelihood),
   searched once where its hysteresis of its own holds a mode, once where its pull does (PULLS);
5. the power at the wavemeter: linear in the current above a threshold, and how it changes along a mode (with δ).

LaserModel.facts() says it all as sentences; LaserModel.simulate() predicts the frequency along any path of current,
piezo and time, LaserModel.step() one point after another (the simulator's lasers can be made of it: sim.ModelLaser).
Needs numpy and scipy.
"""
from __future__ import annotations

import json
import math
import sqlite3
from dataclasses import asdict, dataclass, field

import numpy as np

C_LIGHT = 299792458.0  # m/s
N_GROUP = 2.6          # the group index of a diode chip, roughly (GaN, GaAs: 2.5 to 3.7): for its length in mm
UNIT_T = 600.0         # s: drift is fitted per this much time (said per 10 min)
SINGLE = 50.0          # one selection filter: its loss as a chip resonance this many external modes apart would have
RULE = ("lo", "hi", "h_up", "h_down", "offset", "pull", "lag")
THIN = 12000           # points enough for the combs' spacings (a larger map: thinned for those searches)
MAX_DRIFT = 10.0       # GHz per UNIT_T: the most a comb is taken to drift (more: free-running ECDLs don't)
BUDGET = 40            # generations of each of the rule's searches
CHUNK = 100            # sweeps scored at once (memory: sweeps x points x modes)


# --------------------------------------------------------------------- data
@dataclass
class MapData:
    """A map's points as arrays (one entry a point, in the order measured): line (its index among the lines, in the
    order swept), leg (1 up, 2 down), i (mA, as it was), v (V), t (s), f (GHz from f_ref; NaN: no frequency), ok (a
    clean frequency: not spread over two modes), power (µW, NaN: none); lines: the map's lines (axis, slow value,
    repeat, t0), f_ref THz; kind: which axis was swept ('current' or 'piezo')."""
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

    @property
    def fast(self) -> np.ndarray:
        return self.i if self.kind == "current" else self.v

    def sweeps(self):
        """(indices of a sweep's points, in order), sweep by sweep."""
        key = self.line * 4 + self.leg
        cut = np.flatnonzero(np.diff(key)) + 1
        return np.split(np.arange(len(key)), cut)

    def thinned(self, most: int) -> MapData:
        """At most about `most` of its points, evenly through each sweep (for searches that need no more)."""
        k = max(1, math.ceil(len(self.f) / max(most, 1)))
        if k == 1:
            return self
        idx = np.concatenate([sw[::k] for sw in self.sweeps()]) if len(self.f) else np.zeros(0, dtype=int)
        return MapData(self.line[idx], self.leg[idx], self.i[idx], self.v[idx], self.t[idx], self.f[idx],
                       self.ok[idx], self.power[idx], self.lines, self.f_ref, self.kind)


def data_of(m, current: str = "act", spread: float = 0.3, map_n: int | None = None) -> MapData:
    """One map's points (lasermap.LaserMap: its legs up and down; descents left out), with its first line swept
    again at the end if that was this map's: map_n (None: the last map, the one swept again). The current as
    LaserMap.sweeps has it ('act': as read at each point, else the set current and what it was besides where it
    began). spread: GHz between a point's readings above which its frequency is no one mode's."""
    start = m.meta.get("start") or {}
    off = (start.get("current_act") or 0.0) - (start.get("current_set") or 0.0)
    f_ref = next((q["f"] for q in m.points if q["f"] is not None), 0.0)
    mains = [ln for ln in m.lines if not ln.get("repeat")]
    if not mains:
        raise ValueError("no lines in the map")
    last = max(ln["map"] for ln in mains)
    n = last if map_n is None else map_n
    first = next((ln for ln in mains if ln["map"] == n), None)
    if first is None:
        raise ValueError(f"no map {n + 1} in the file")
    take = {ln["id"] for ln in mains if ln["map"] == n}
    if n == last:  # (the first line swept again comes after the last map, and is its)
        take |= {ln["id"] for ln in m.lines if ln.get("repeat") and ln["axis"] == first["axis"]
                 and ln["temp"] == first["temp"]}
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
            rows.append((k, leg, i, v, q["t"], f, clean, q["power"] if q["power"] is not None else math.nan))
    a = np.array(rows, dtype=float) if rows else np.zeros((0, 8))
    lines = [by_id[lid] for lid, _ in sorted(order.items(), key=lambda kv: kv[1])]
    return MapData(a[:, 0].astype(int), a[:, 1].astype(int), a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6] > 0.5,
                   a[:, 7], lines, f_ref, first["axis"])


def pace_of(d: MapData) -> tuple:
    """(s per unit of the slow axis from line to line, s from point to point, the swept axis's step, s from the
    first line to that line swept again (None: not swept again)), as the map went."""
    mains = [ln for ln in d.lines if not ln.get("repeat")]
    per = [(b["t0"] - a["t0"]) / (b["slow"] - a["slow"]) for a, b in zip(mains, mains[1:])
           if b["slow"] != a["slow"] and a.get("t0") is not None and b.get("t0") is not None]
    dts, dxs = [], []
    for idx in d.sweeps():
        if len(idx) > 1:
            dts.append(np.diff(d.t[idx]))
            dxs.append(np.abs(np.diff(d.fast[idx])))
    rep = next((ln for ln in d.lines if ln.get("repeat")), None)
    again = rep["t0"] - mains[0]["t0"] if rep is not None and mains and rep.get("t0") is not None else None
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
    out = np.zeros(len(spacings))
    total = len(x)
    for g in np.unique(groups):
        s = groups == g
        z = np.exp(2j * np.pi * x[s][None, :] / spacings[:, None]).sum(axis=1)
        out += np.abs(z)
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
    grid = np.exp(np.linspace(math.log(lo), math.log(hi), int(4000 * math.log(hi / lo)) + 1))
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
    grid = np.linspace(2.5 * small, min(sizes.max(), 500.0), 4000)
    score = np.cos(2 * np.pi * sizes[None, :] / grid[:, None]).mean(axis=1)
    k = _largest_peak(score)
    return float(grid[k]) if score[k] > 0.5 else None


@dataclass
class Comb:
    """A comb of modes: spacing GHz apart (0: one selection filter, no comb), at frequencies m * spacing + p0 + p_i dI
    + p_ii dI² + p_v dV + p_vv dV² + p_t dt (dI, dV, dt from the model's i0, v0, t0; dt in units of UNIT_T), m whole,
    and + p_leg / 2 sweeping down, - p_leg / 2 up; its rms residual (GHz), how strongly the frequencies bunched
    (0..1), how many points; drift_known: a line swept again later told drift from piezo (else p_t is 0 and the
    drift is in p_v, p_vv); others: the drifts the map allows nearly as well (whole modes more or less by the time it
    was swept again), each {p0, p_v, p_vv, p_t, p_leg}."""
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

    def base(self, di, dv, dt, up=None):
        out = self.p0 + (self.p_i + self.p_ii * di) * di + (self.p_v + self.p_vv * dv) * dv + self.p_t * dt
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
             legs: bool = True) -> Comb:
    """One comb fitted to the map: each line's offset (where its frequencies bunch, `rate` GHz per unit of the swept
    axis taken off), unwrapped from line to line; each point's mode number from that; then frequency = mode number
    * spacing + quadratic in current and piezo, linear in time, offset by the way it was swept, by least squares
    (iterated: the mode numbers again from the fit). The first line swept again at the end (a line at a piezo
    voltage swept before, later) tells drift from piezo, to whole modes: each of the nearest numbers of modes is
    fitted (as many as a drift of MAX_DRIFT could be), the one fitting best kept, those as good as it within what
    the residuals can tell (they go together along a mode) kept as others. No line swept again: no drift term.
    legs: an offset by the way it was swept (not for the chip comb: sweeping up and down at a current the laser is
    mostly on different chip modes, which that would take for a change of their spacing)."""
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
    drift_known = bool(len(again)) and main.sum() >= 3
    guess = {}
    if drift_known:  # (where the main lines put a line swept again at its piezo voltage, had nothing drifted)
        a = np.polyfit(slow[main], unwrapped[main], 2)
        guess = {L: phase[L] + spacing * round((np.polyval(a, slow[L]) - phase[L]) / spacing) for L in again}
    extra = [di, di * di, dv, dv * dv] + ([dt] if drift_known else [])
    tried = []
    span = max((float(np.max(d.t[s & (d.line == L)])) for L in again), default=0.0) - float(np.min(d.t[s]))
    most = int(MAX_DRIFT * span / UNIT_T / spacing) if drift_known else 0
    for k in range(-most, most + 1):
        u = unwrapped.copy()
        for L in again:
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
def _losses(modes, mid, dc, lo, hi):
    """Each mode's loss: sin²(π δ / dc), δ its distance from its chip resonance's best place (mid, the others dc
    apart), and far more outside the window (lo .. hi)."""
    return np.sin(np.pi * ((modes - mid + dc / 2) % dc - dc / 2) / dc) ** 2 + \
        ((np.maximum(modes - hi, 0.0) + np.maximum(lo - modes, 0.0)) / 5.0) ** 2


def _select(M, B, A, T, up, cur, ci0: float, i0: float, P, meas=None, valid=None, forced: bool = False):
    """The rule run along sweeps, all at once, for candidates all at once. M (n, L, K): the modes' GHz along each of
    n sweeps of L points (K modes); B (n, L): the chip comb's middle (GHz), as it moves ci0 GHz/mA with the current
    from i0; A (n, L) mA; T (n, L) s; up (n, L): the swept axis rising there; cur (n,): the mode each sweep begins on
    (an index into K). P (pop, 9): each candidate's rule (RULE), its chip comb's spacing and GHz/mA. Returns the
    modes chosen (pop, n, L), or given meas (n, L, the modes measured) and valid (n, L): how many valid points each
    candidate put on the mode measured (pop,); forced: each point from the mode measured at the one before (a point
    at a time: one wrong hop does not take the rest of its sweep with it)."""
    P = np.atleast_2d(np.asarray(P, dtype=M.dtype))
    lo, hi, h_up, h_down, offset, pull, lag, dc, ci = (P[:, k][:, None] for k in range(9))
    dc3 = dc[:, :, None]
    npop, (n, L, _) = len(P), M.shape
    now = np.broadcast_to(np.asarray(cur, dtype=int), (npop, n)).copy()
    warm = np.broadcast_to(A[:, 0], (npop, n)).copy()  # the current the chip is as warm as (it lags)
    lo3, hi3 = lo[:, :, None], hi[:, :, None]
    ar = np.arange(n)[None, :]
    right = np.zeros(npop)
    chosen = np.empty((npop, n, L), dtype=int) if meas is None else None
    for j in range(L):
        if j:
            fade = np.exp(-(T[:, j] - T[:, j - 1])[None, :] / np.maximum(lag, 1e-9))
            warm = A[None, :, j] + (warm - A[None, :, j]) * fade
        base = B[None, :, j] + (ci - ci0) * (A[None, :, j] - i0) + offset + ci * (warm - A[None, :, j])
        mj = M[None, :, j, :]
        if j and forced:
            now = np.broadcast_to(meas[None, :, j - 1], (npop, n)).copy()
        fc = M[ar, j, now]
        mid = base + pull * ((fc - base + dc / 2) % dc - dc / 2)
        g = _losses(mj, mid[:, :, None], dc3, lo3, hi3)
        if j:  # (at its first point it is on the mode it began on)
            best = np.argmin(g, axis=2)
            h = np.where(up[None, :, j], h_up, h_down)
            g_now = np.take_along_axis(g, now[:, :, None], 2)[:, :, 0]
            g_best = np.take_along_axis(g, best[:, :, None], 2)[:, :, 0]
            now = np.where(g_now > g_best + h, best, now)
        if meas is None:
            chosen[:, :, j] = now
        else:
            right += ((now == meas[None, :, j]) & valid[None, :, j]).sum(axis=1)
    return chosen if meas is None else right


def _likelihood(M, B, A, T, up, ci0: float, i0: float, P, meas, valid, eps: float = 0.02) -> np.ndarray:
    """The mean log likelihood of what the laser did, a point at a time from the mode measured at the one before,
    for candidates all at once (P: pop x 10, _select's and a temperature, of a chip mode's loss): it goes on lasing,
    or hops to another mode as much as that one is better than the hysteresis asks (a softmax at that temperature),
    or, with eps, anything at all (what the rule cannot tell: a reading put on the wrong mode, a flicker). Smooth,
    and every hop counts: what a search needs to find where the rule is (_fit_rule)."""
    P = np.atleast_2d(np.asarray(P, dtype=M.dtype))
    lo, hi, h_up, h_down, offset, pull, lag, dc, ci, temp = (P[:, k][:, None] for k in range(10))
    npop, (n, L, K) = len(P), M.shape
    warm = np.broadcast_to(A[:, 0], (npop, n)).copy()
    ar = np.arange(n)
    out = np.zeros(npop)
    for j in range(1, L):
        fade = np.exp(-(T[:, j] - T[:, j - 1])[None, :] / np.maximum(lag, 1e-9))
        warm = A[None, :, j] + (warm - A[None, :, j]) * fade
        base = B[None, :, j] + (ci - ci0) * (A[None, :, j] - i0) + offset + ci * (warm - A[None, :, j])
        was = meas[:, j - 1]
        fc = M[ar, j, was][None, :]
        mid = base + pull * ((fc - base + dc / 2) % dc - dc / 2)
        g = _losses(M[None, :, j, :], mid[:, :, None], dc[:, :, None], lo[:, :, None], hi[:, :, None])
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
    lo: float = -math.inf        # GHz: the window
    hi: float = math.inf
    h_up: float = 0.0            # the hysteresis, sweeping up and down (of a chip resonance's loss at its most, 1)
    h_down: float = 0.0
    offset: float = 0.0          # GHz: a chip resonance's best place from its middle
    pull: float = 0.0            # the mode lasing pulls its chip resonance's best place this part of the way to it
    lag: float = 0.0             # s: the chip's temperature lags the current's heating by this
    i_th: float | None = None    # mA: the threshold, as the power at the wavemeter says it
    slope: float | None = None   # µW/mA at the wavemeter, above it
    power_v: float | None = None  # µW/V: the power's dependence on the piezo at a given current
    power_dp: float | None = None  # µW per GHz a mode sits above its chip resonance's best place (δ)
    right: float = 0.0           # the fraction of the map's points the model puts on the right mode...
    near: float = 0.0            # ...and of those it misses, within two points of a hop the laser made
    points: int = 0
    kind: str = "current"
    pace: float | None = None
    step_s: float | None = None
    step_x: float | None = None
    again_s: float | None = None
    hysteresis: float | None = None
    apart: float | None = None
    notes: list = field(default_factory=list)

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
        """Its rule's parameters as _select has them: RULE's, its chip comb's spacing and GHz/mA."""
        return np.array([getattr(self, k) for k in RULE] + [self.dc, self._ci()], dtype=float)

    def _d(self, i, v, t):
        return np.asarray(i, dtype=float) - self.i0, np.asarray(v, dtype=float) - self.v0, \
            (np.asarray(t, dtype=float) - self.t0) / UNIT_T

    def ext_modes(self, i, v, t, orders, up=None) -> np.ndarray:
        """GHz: the external modes of those orders (whole numbers) at (i, v, t), sweeping up or down (None: between):
        points x orders."""
        return self.ext.base(*self._d(i, v, t), up)[..., None] + np.asarray(orders) * self.ext.spacing

    def chip_base(self, i, v, t):
        """GHz: a chip resonance's middle at (i, v, t), as the chip comb has it (the others dc apart)."""
        if self.chip is None:
            return np.zeros_like(np.asarray(i, dtype=float))
        return self.chip.base(*self._d(i, v, t))

    def orders(self, margin: float = 60.0) -> np.ndarray:
        """The external orders that can lase about the map's frequencies."""
        lo = self.lo if math.isfinite(self.lo) else -600.0
        hi = self.hi if math.isfinite(self.hi) else 600.0
        e = self.ext
        return np.arange(math.floor((lo - margin - abs(e.p0) - 400) / e.spacing),
                         math.ceil((hi + margin + abs(e.p0) + 400) / e.spacing) + 1)

    def _first(self, modes, base) -> int:
        """The mode lasing where a path begins (as if long there): the best, pulled to it."""
        dc, P = self.dc, self.rule()
        g = self._loss(modes, base + P[4], P)
        k = int(np.argmin(g))
        mid = base + P[4] + self.pull * ((modes[k] - base - P[4] + dc / 2) % dc - dc / 2)
        return int(np.argmin(self._loss(modes, mid, P)))

    def _loss(self, modes, mid, P=None):
        P = self.rule() if P is None else P
        return _losses(modes, mid, self.dc, P[0], P[1])

    def simulate(self, i, v, t, start: float | None = None, legs=None, clock=None) -> np.ndarray:
        """GHz: the frequency along a path (arrays i mA, v V, t s, in order), the mode lasing kept until another is
        better by the hysteresis (h_up where the swept axis rises, h_down where it falls, or as `legs` says: 1 up,
        2 down). start: GHz it is on at the first point (None: the best mode there, as if long there). clock: s at
        each point for the chip's lag (None: t)."""
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
        orders = self.orders()
        modes = self.ext_modes(i, v, t, orders, up)
        base = self.chip_base(i, v, t)
        cur = int(np.argmin(np.abs(modes[0] - start))) if start is not None else self._first(modes[0], base[0])
        T = t if clock is None else np.asarray(clock, dtype=float)
        k = _select(modes[None], base[None], i[None], T[None], up[None], [cur], self._ci(), self.i0, self.rule())
        return modes[np.arange(n), k[0, 0]]

    def _ci(self) -> float:
        return self.chip.p_i if self.chip is not None else 0.0

    def step(self, i: float, v: float, t: float, state=None, clock: float | None = None) -> tuple:
        """One point of a path after another: (GHz it lases at, δ GHz from its chip resonance's best place, the state
        for the next). state: what the last point left (None: a fresh start, as if long there); clock: s for the
        chip's lag (None: t)."""
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
        b = float(e.base(di, dv, dt, up))
        lo = self.lo if math.isfinite(self.lo) else b - 3 * dc
        hi = self.hi if math.isfinite(self.hi) else b + 3 * dc
        orders = np.arange(math.floor((lo - 30 - b) / e.spacing), math.ceil((hi + 30 - b) / e.spacing) + 1)
        modes = b + orders * e.spacing
        base = float(self.chip_base(i, v, t)) + self.offset + self._ci() * (warm - i)
        if order is None or not (orders[0] <= order <= orders[-1]):
            k = self._first(modes, base - self.offset)
        else:
            k = int(order - orders[0])
            mid = base + self.pull * ((modes[k] - base + dc / 2) % dc - dc / 2)
            g = self._loss(modes, mid)
            best = int(np.argmin(g))
            if g[k] > g[best] + (self.h_up if up else self.h_down):
                k = best
        mid = base + self.pull * ((modes[k] - base + dc / 2) % dc - dc / 2)
        delta = float((modes[k] - mid + dc / 2) % dc - dc / 2)
        return float(modes[k]), delta, (int(orders[k]), warm, now_t, x, up)

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
        """Those sweeps as arrays for _select, each from its first clean point, padded to the longest (dtype: of
        the arrays; float32 for a search: twice as fast, and as good for it)."""
        n, L = len(sw), max(len(idx) for idx in sw)
        up_pt = d.leg == 1
        orders = self.orders()
        modes = self.ext_modes(d.i, d.v, d.t, orders, up_pt)
        f = d.f[d.ok]
        span = float(f.max() - f.min()) if len(f) else 0.0
        reach = 0.3 * span + 60.0 if not self.single else 6 * self.ext.spacing
        keep = (modes[d.ok].max(axis=0) > f.min() - reach) & (modes[d.ok].min(axis=0) < f.max() + reach)
        modes = modes[:, keep]
        base = self.chip_base(d.i, d.v, d.t)
        K = modes.shape[1]
        M, B, A, T = (np.zeros(shape, dtype=dtype) for shape in ((n, L, K), (n, L), (n, L), (n, L)))
        meas, valid, up = np.zeros((n, L), dtype=int), np.zeros((n, L), dtype=bool), np.zeros((n, L), dtype=bool)
        for s, idx in enumerate(sw):
            k = len(idx)
            M[s, :k], M[s, k:] = modes[idx], modes[idx[-1]]
            B[s, :k], B[s, k:] = base[idx], base[idx[-1]]
            A[s, :k], A[s, k:] = d.i[idx], d.i[idx[-1]]
            T[s, :k], T[s, k:] = d.t[idx], d.t[idx[-1]] + np.arange(1, L - k + 1)
            meas[s, :k] = np.argmin(np.abs(modes[idx] - d.f[idx][:, None]), axis=1)
            valid[s, 1:k] = True
            up[s, :] = up_pt[idx[0]]
        return {"M": M, "B": B, "A": A, "T": T, "meas": meas, "valid": valid, "up": up}

    def _score(self, P, pk: dict, forced: bool = False) -> np.ndarray:
        """The fraction right of each candidate rule (P: pop x 9, as rule()) on a packed map (forced: a point at a
        time, from the mode measured at the one before)."""
        return _select(pk["M"], pk["B"], pk["A"], pk["T"], pk["up"], pk["meas"][:, 0], self._ci(), self.i0, P,
                       pk["meas"], pk["valid"], forced) / max(int(pk["valid"].sum()), 1)

    def score(self, d: MapData) -> float:
        """The fraction of the map's clean points it puts on the mode the laser was on, each sweep simulated from its
        first clean point (as measured)."""
        return self.accuracy(d)[0]

    def accuracy(self, d: MapData) -> tuple:
        """(the fraction of the map's clean points it puts on the mode the laser was on, the fraction of those it
        misses within two points of a hop the laser made: where the model hops a point or two early or late)."""
        right = total = near = 0
        sw = self._sweeps(d)
        for a in range(0, len(sw), CHUNK):
            pk = self._pack(d, sw[a:a + CHUNK])
            k = _select(pk["M"], pk["B"], pk["A"], pk["T"], pk["up"], pk["meas"][:, 0], self._ci(), self.i0,
                        self.rule())[0]
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
        return right / max(total, 1), near / max(total - right, 1)

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
        orders = self.orders()
        modes = self.ext_modes(i, v, self.t0, orders, up)
        base = self.chip_base(i, v, np.full(2 * n, self.t0))
        T = np.arange(2 * n) * (self.step_s or 0.3)
        k = _select(modes[None], base[None], i[None], T[None], up[None], [self._first(modes[0], base[0])],
                    self._ci(), self.i0, self.rule())[0, 0]
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
            out.append(f"its chip's own modes are {c.spacing:.3f} GHz apart (an optical length n_g·L of {Lc:.3f} mm: "
                       f"{Lc / N_GROUP:.2f} mm of chip at a group index of {N_GROUP:g}): its front facet reflects, so "
                       f"they decide which external mode lases and where it hops (its anti-reflection coating is "
                       f"poor, or there is none); {c.spacing / e.spacing:.3f} external modes to a chip mode")
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
            out.append(f"it lases only within {self.lo:+.0f} … {self.hi:+.0f} GHz of {self.f_ref:.6f} THz "
                       f"({self.hi - self.lo:.0f} GHz about {mid:.6f} THz, {C_LIGHT / (mid * 1e12) * 1e9:.4f} nm): "
                       f"its grating's feedback and its gain together, not moved by the piezo")
        ff = self.ideal_feedforward()
        if ff is not None and not single:
            step = (c.spacing - e.spacing * round(c.spacing / e.spacing))
            rel = c.p_i - e.p_i
            out.append(f"it hops where an external mode near another chip mode lines up better: the combs slide past "
                       f"each other {rel:+.3f} GHz/mA, and from one chip mode to the next the alignment changes by "
                       f"{abs(step):.2f} GHz, so sweeping the current it hops every {abs(step / rel):.2f} mA or so, "
                       f"sweeping the piezo every {abs(step / (e.p_v - c.p_v)):.2f} V")
        if ff is not None:
            along = e.p_v + e.p_i * ff
            reach = (f" without hopping until the window's edge ({(self.hi - self.lo) / max(abs(along), 1e-9):.0f} V "
                     f"of piezo, at most)" if math.isfinite(self.hi - self.lo) else "")
            out.append(f"the ideal feedforward: {ff:+.3f} mA/V keeps the combs aligned; with it the laser tunes "
                       f"{along:+.3f} GHz/V{reach}")
        out += self._hysteresis_facts(single)
        out += self._power_facts(single)
        out += self._drift_facts(nu, L)
        out.append(f"the model puts {self.right:.0%} of the map's {self.points:,} points on the mode the laser was on "
                   f"(each sweep simulated from its first point); {self.near:.0%} of the rest are within two points of "
                   f"a hop: there it hops a point or two early or late")
        return out + list(self.notes)

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
        d = dict(d)
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


# ---------------------------------------------------------------------- fit
def fit_power(d: MapData) -> tuple:
    """(threshold mA, µW per mA, µW per V at a given current) from the power at the wavemeter where it lased, None
    each if it cannot tell."""
    s = d.ok & ~np.isnan(d.power)
    if s.sum() < 20:
        return None, None, None
    i, v, p = d.i[s], d.v[s], d.power[s]
    keep = np.ones(len(p), dtype=bool)
    sol = None
    for _ in range(5):
        A = np.column_stack([i[keep], v[keep] - np.median(v), np.ones(keep.sum())])
        sol, *_ = np.linalg.lstsq(A, p[keep], rcond=None)
        res = p - np.column_stack([i, v - np.median(v), np.ones(len(p))]) @ sol
        sd = np.std(res[keep])
        keep = np.abs(res) < 3 * sd
    slope, per_v, c0 = sol
    if slope <= 0:
        return None, None, None
    return float(-c0 / slope), float(slope), float(per_v)


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
        log=None) -> LaserModel:
    """The model of a map's laser (lasermap.LaserMap; map_n: which of its maps, None the last), as the module's
    docstring says. budget: generations of the rule's search (None: BUDGET; fewer: faster, coarser). log(text):
    progress."""
    say = log or (lambda text: None)
    budget = BUDGET if budget is None else budget
    d = data_of(m, current, map_n=map_n)
    if d.ok.sum() < 50:
        raise ValueError("too few points with a frequency for a model")
    i0, v0 = float(np.median(d.i[d.ok])), float(np.median(d.v[d.ok]))
    t0 = float(np.median(d.t[d.ok]))
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
        chip = fit_comb(d, dc, chip_rate, i0, v0, t0, legs=False)
        say(f"chip comb: {chip.spacing:.3f} GHz, {chip.p_i:+.3f} GHz/mA")
        de, _ = comb_spacing(few, rate, 2.0, chip.spacing / 4.5)
    else:
        de = rough
    ext = fit_comb(d, de, rate, i0, v0, t0)
    others = ", ".join(f"{o['p_t']:+.2f}" for o in ext.others)
    say(f"external comb: {ext.spacing:.4f} GHz, {ext.p_i:+.3f} GHz/mA, {ext.p_v:+.3f} GHz/V, drift {ext.p_t:+.2f} "
        f"GHz per 10 min" + (f" (or {others})" if others else ""))
    if chip is None:  # one selection filter: the lasing frequencies' middle, as it moves
        chip = _single_filter(d, i0, v0, t0)
    model = LaserModel(d.f_ref, i0, v0, t0, ext, chip, kind=d.kind)
    model.pace, model.step_s, model.step_x, model.again_s = pace_of(d)
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
    _fit_rule(model, d, budget, seed)
    model.right, model.near = model.accuracy(d)
    model.points = int(d.ok.sum())
    model.i_th, model.slope, model.power_v = fit_power(d)
    model.power_dp = fit_power_dp(d, model)
    fast = d.fast[d.ok]
    model.hysteresis, model.apart = model.hysteresis_of(float(np.percentile(fast, 2)), float(np.percentile(fast, 98)))
    say(f"right mode at {model.right:.0%} of the points")
    return model


def _chip_rate(d: MapData, quantum: float) -> float:
    """GHz per unit of the swept axis the chip's modes move: where the frequencies, that taken off, bunch best at the
    quantum, line by line."""
    rates = np.linspace(-15.0, 15.0, 601)
    s = d.ok
    x0 = d.fast[s] - np.median(d.fast[s])
    best, at = -1.0, 0.0
    for r in rates:
        b = _bunching(d.f[s] - r * x0, d.line[s], np.array([quantum]))[0]
        if b > best:
            best, at = b, r
    return float(at)


def _single_filter(d: MapData, i0, v0, t0) -> Comb:
    """One selection filter: the lasing frequencies' middle, quadratic in current and piezo, by least squares (a
    comb of spacing 0)."""
    s = d.ok
    di, dv = d.i[s] - i0, d.v[s] - v0
    A = np.column_stack([np.ones(s.sum()), di, di * di, dv, dv * dv])
    sol, *_ = np.linalg.lstsq(A, d.f[s], rcond=None)
    res = d.f[s] - A @ sol
    return Comb(0.0, float(sol[0]), float(sol[1]), float(sol[3]), float(sol[4]), 0.0, float(np.sqrt(np.mean(res ** 2))),
                1.0, int(s.sum()), False, float(sol[2]))


PULLS = ((-0.5, 0.25), (0.15, 0.85))  # the rule's searches: a mode pushed on or barely held, and one pulled hard


def _fit_rule(model: LaserModel, d: MapData, budget: int, seed: int, most: int = 120, popsize: int = 15):
    """The rule (RULE: the window, the hysteresis each way, the chip resonances' best place, its pull to the mode
    lasing, the chip's lag), and the chip comb's spacing and current tuning again (by where it hops: where the
    frequencies lie on its resonances misleads, near the window's edges, as a mode lases only on their inner side
    there), by differential evolution, every candidate of a generation and every sweep at once (`most` sweeps at
    most, spread over the map), in two stages: the rule that makes what the laser did likeliest, a point at a time
    (_likelihood: smooth, and every hop counts, so the search finds where the rule is), then from there and from a
    plain guess (both among its first population) the rule that puts most points on the mode the laser was on, each
    sweep simulated from its first point (what the model is for). A hysteresis of its own and a mode's pull on its
    chip resonance both hold a mode: such a search settles on one or the other as it goes, so it is searched twice,
    once for each (PULLS), the better kept."""
    from scipy.optimize import differential_evolution

    pk = model._pack(d, model._sweeps(d, most), np.float32)
    f = d.f[d.ok]
    lo0, hi0 = float(np.percentile(f, 1)), float(np.percentile(f, 99))
    span = max(hi0 - lo0, model.ext.spacing)
    half = 0.5 * model.ext.spacing
    dc0, ci0 = model.dc, model._ci()
    bounds = [(lo0 - 0.3 * span, lo0 + 0.1 * span), (hi0 - 0.1 * span, hi0 + 0.3 * span), (0.0, 0.05), (0.0, 0.05),
              (-half, half), (-0.5, 0.85), (0.0, max(2.0, 6 * (model.step_s or 0.3))),
              (0.98 * dc0, 1.02 * dc0), (ci0 - 0.1 * abs(ci0) - 0.1, ci0 + 0.1 * abs(ci0) + 0.1)]
    if model.single:  # (one filter: no window, its width as it is)
        bounds[0], bounds[1], bounds[7] = (-1e5, -1e5 + 1), (1e5 - 1, 1e5), (dc0 * (1 - 1e-9), dc0 * (1 + 1e-9))

    def cost(P):  # (P: parameters x candidates)
        return -model._score(np.asarray(P).T, pk)

    guess = [lo0 - 0.05 * span, hi0 + 0.05 * span, 0.005, 0.005, 0.0, 0.2, model.step_s or 0.3, dc0, ci0]
    if model.single:
        guess[0], guess[1] = bounds[0][0], bounds[1][1]
    how = {"tol": 1e-6, "polish": False, "init": "sobol", "vectorized": True, "updating": "deferred"}

    def unlikely(P):
        return -_likelihood(pk["M"], pk["B"], pk["A"], pk["T"], pk["up"], model._ci(), model.i0, np.asarray(P).T,
                            pk["meas"], pk["valid"])

    soft = differential_evolution(unlikely, bounds + [(1e-4, 0.02)], maxiter=max(budget // 2, 5),
                                  popsize=max(popsize - 3, 5), seed=seed, x0=guess + [0.003], **how)
    rng = np.random.default_rng(seed)
    res = None
    for k, pulls in enumerate(PULLS):  # (each its own population, the likeliest rule and the guess among its first)
        part = list(bounds)
        part[5] = pulls
        lows, highs = np.array(part).T
        first = lows + (highs - lows) * rng.random((popsize * len(part), len(part)))
        first[0], first[1] = np.clip(soft.x[:9], lows, highs), np.clip(guess, lows, highs)
        r = differential_evolution(cost, part, maxiter=budget, popsize=popsize, seed=seed + k,
                                   **dict(how, init=first))
        if res is None or r.fun < res.fun:
            res = r
    for k, x in zip(RULE, res.x):
        setattr(model, k, float(x))
    if model.single:
        model.lo, model.hi = -math.inf, math.inf
    else:
        model.chip.spacing = float(res.x[7])
    if model.chip is not None:
        model.chip.p_i = float(res.x[8])
