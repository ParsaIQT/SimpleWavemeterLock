"""Where to park a laser for a frequency, by its own model (lasermodel.py, fitted to its map; a map at several
temperatures makes the temperature part of it): on the mode that holds most widely there, power not counted.

At temperatures across those its map was made at (finer than it was), piezo voltages and currents across what it
covered, the model at rest (its chip as warm as its current, its external modes between where sweeping up and down
put them) is swept along the current up and down at every piezo voltage, by its rule and by the rule held the other
way (its map tells them apart only so far). Where a mode is on target, the same both ways and by both rules, the spot:

- its core: the currents about it where the laser is on that mode sweeping up and sweeping down alike (from either
  side it comes to that mode, whatever it was on before), and the spot's place in it;
- held: parked there, on it, how far the current can move each way before it hops (its hysteresis with it);
- the window: how far the target is inside where it lases at all.

The spots are ranked by the narrower side of their core (a wide mode, and the spot in its middle), then refined
about the best (the temperature, the piezo, five times finer). A laser whose modes sweeping up and down are seldom
the same (a large hysteresis) has few such spots: after them come those on target one way only (come to sweeping
the current up, or down), ranked by how far parked there the current can go on that way. For the best few besides:
the piezo voltages it holds over, the current kept; the drift of its cavity a lock can take up (the frequency held
by the piezo, the current moved with it as a feedforward has it: none, the controller's, the ideal) before it hops;
and with a model of several temperatures, the best spot near it with the temperature's other coefficients (whole
modes more or less per step of its map: the ones it allows as well), the worst of them: what the spot is worth if
those were the laser's.

park() finds them (Spot), say_spot() says one. Needs numpy.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .lasermodel import LaserModel, _select

REST = 1e6        # s between the points of a path at rest (the chip as warm as its current at each)
REACH_V = 12.0    # V: how far the piezo is moved each way for its margins
REACH_I = 4.0     # mA: how far the current is moved each way for its margins
MIN_HOLD = 0.1    # mA: a spot whose mode, parked on, holds less than this either way is none
REACH_DRIFT = 15.0  # GHz: how far a cavity's drift is followed by the lock, each way
NEAR_V, NEAR_I = 4.0, 3.0  # V, mA: a spot this near, with the temperature's other coefficients, is near enough
BEYOND = 3.0      # mA: the sweeps begin this far beyond the currents searched (what they come from there, as it would)


@dataclass
class Spot:
    """Where to park (T °C as set, None: the model's one temperature; v V of piezo as measured, v_set as set; i mA,
    measured) and how widely its mode holds there: order (its external mode, the model's numbering); way: 'either'
    (the same mode sweeping up and down, by both rules), or 'up' or 'down' (on it only come to that way: its core
    nothing); core (mA below and above the spot: the same mode sweeping up and down, by both rules); held (mA down
    and up, parked on it, by both rules: how far the current can go each way before it hops); window (GHz
    the target is above its window's low edge, below its high one); edges: which of those margins reached the end of
    what was searched (core low, core high, held down, held up: at least that); piezo (V
    down and up it holds over, the current kept); lock ({feedforward mA/V: (GHz of drift down, up)} a lock holding
    the frequency by the piezo takes up before it hops); worst (mA: with each of the temperature's other
    coefficients, the best spot near it (NEAR_V, NEAR_I): its core's narrower side, the worst of them; None: not
    asked, or one temperature); score (mA: its core's narrower side; one way, its held on that way)."""
    T: float | None
    v: float
    v_set: float
    i: float
    order: int
    core: tuple
    held: tuple
    window: tuple
    edges: tuple = (False, False, False, False)
    piezo: tuple | None = None
    lock: dict = field(default_factory=dict)
    worst: float | None = None
    worst_of: list = field(default_factory=list)  # [(chip GHz/K, external GHz/K, the best spot near: its score mA, V,
    #                                                mA)]
    way: str = "either"
    entry: float | None = None   # mA: one way, where its mode came on, as the current came that way...
    start: float | None = None   # ...and where to begin the current from, to come to it so (two hops' worth before)

    @property
    def score(self) -> float:
        """mA: its core's narrower side; one way, how far parked it holds the current going on that way."""
        return min(self.core) if self.way == "either" else self.held[1 if self.way == "up" else 0]

    @property
    def back(self) -> float:
        """mA: one way, how far parked it holds the current going back (its hysteresis then the other way's)."""
        return min(self.held) if self.way == "either" else self.held[0 if self.way == "up" else 1]


def _grid(lo: float, hi: float, step: float) -> np.ndarray:
    n = max(int(round((hi - lo) / step)), 0)
    return lo + step * np.arange(n + 1)


def _rules(model: LaserModel) -> np.ndarray:
    """Its rule and the one held the other way (as _select has them)."""
    rules = [model.rule()]
    if model.alt is not None:
        rules.append(model.with_rule(model.alt).rule())
    return np.array(rules)


def _span(m: LaserModel, P: np.ndarray) -> int:
    """External modes either side of one that a path's modes must reach (the window, and a chip mode beyond)."""
    de = m.ext.spacing
    width = (m.hi - m.lo) if math.isfinite(m.hi - m.lo) else 6 * de
    return min(int(math.ceil((width + 2 * float(np.max(P[:, 7])) + 2 * float(np.max(P[:, 11]))) / de)) + 2, 120)


def _sweeps(m: LaserModel, P: np.ndarray, cur: np.ndarray, pz: np.ndarray, target: float) -> tuple:
    """The model at rest swept along the currents cur (in order) up, and back down, at each piezo voltage of pz: (the
    modes it is on, by each rule: rules x lines x currents, up and down; their frequencies, GHz: lines x currents x
    modes; their orders: lines x modes)."""
    n, L = len(pz), len(cur)
    ii = np.broadcast_to(cur[None, :], (n, L))
    vv = np.broadcast_to(pz[:, None], (n, L))
    e = m.ext.base(ii - m.i0, vv - m.v0, 0.0)
    half = _span(m, P)
    k0 = np.round((target - e.mean(axis=1)) / m.ext.spacing).astype(int)  # (about the target's mode, each line)
    orders = k0[:, None] + np.arange(-half, half + 1)[None, :]
    M = e[:, :, None] + orders[:, None, :] * m.ext.spacing
    B = m.chip_base(ii, vv, m.t0)
    rest = np.broadcast_to(np.arange(L, dtype=float)[None, :] * REST, (n, L))
    out = []
    for rising in (True, False):
        sl = slice(None) if rising else slice(None, None, -1)
        pk = {"M": np.ascontiguousarray(M[:, sl]), "B": np.ascontiguousarray(B[:, sl]),
              "A": np.ascontiguousarray(ii[:, sl]), "V": vv, "T": rest, "up": np.full((n, L), rising),
              "ci0": m._ci(), "i0": m.i0, "v0": m.v0}
        first = m._firsts(pk["M"][:, 0], pk["B"][:, 0], pk["A"][:, 0], pz)
        k = _select(pk, P, first)
        out.append(k if rising else k[:, :, ::-1])
    return out[0], out[1], M, orders


def _runs(key: np.ndarray) -> tuple:
    """For each point of each row (rows x points, ints): the first and last index of the run of equal values it is
    in."""
    n, L = key.shape
    idx = np.broadcast_to(np.arange(L), (n, L))
    new = np.ones((n, L), dtype=bool)
    new[:, 1:] = key[:, 1:] != key[:, :-1]
    end = np.ones((n, L), dtype=bool)
    end[:, :-1] = key[:, :-1] != key[:, 1:]
    lo = np.maximum.accumulate(np.where(new, idx, 0), axis=1)
    hi = np.minimum.accumulate(np.where(end, idx, L - 1)[:, ::-1], axis=1)[:, ::-1]
    return lo, hi


def _spots(m: LaserModel, P: np.ndarray, T, cur: np.ndarray, pz: np.ndarray, target: float, within=None) -> list:
    """The spots on target at T (m: the model there) over piezo voltages pz and currents cur (in order; the spots
    only within (lo, hi) mA of them, if given: the sweeps begin beyond, so that at its edges they have come from
    somewhere, not begun there)."""
    ku, kd, M, orders = _sweeps(m, P, cur, pz, target)
    L = len(cur)
    inside = np.ones(L - 1, dtype=bool) if within is None else \
        (cur[:-1] >= within[0] - 1e-9) & (cur[1:] <= within[1] + 1e-9)
    agree = np.all(ku == ku[:1], axis=0) & np.all(kd == ku[:1], axis=0)
    key = np.where(agree, ku[0], -1)
    c_lo, c_hi = _runs(key)
    held = [(_runs(ku[r])[1], _runs(kd[r])[0]) for r in range(len(P))]
    F = np.take_along_axis(M, ku[0][:, :, None], axis=2)[:, :, 0]
    x = F - target
    hit = (key[:, :-1] >= 0) & (key[:, :-1] == key[:, 1:]) & (x[:, :-1] * x[:, 1:] <= 0) & (x[:, :-1] != x[:, 1:]) & \
        inside[None, :]
    out = []
    for s, j in zip(*np.nonzero(hit)):
        i = float(cur[j] + (cur[j + 1] - cur[j]) * x[s, j] / (x[s, j] - x[s, j + 1]))
        lo_j, hi_j = c_lo[s, j], c_hi[s, j]
        up = min(float(cur[h[0][s, j]]) for h in held) - i
        down = i - max(float(cur[h[1][s, j]]) for h in held)
        w_lo, w_hi = (float(w) for w in m.window(i, pz[s]))
        out.append(Spot(T, float(pz[s]), float(pz[s]) - m.v_set_to_act, i, int(orders[s, key[s, j]]),
                        (i - float(cur[lo_j]), float(cur[hi_j]) - i), (max(down, 0.0), max(up, 0.0)),
                        (target - w_lo, w_hi - target),
                        (lo_j == 0, hi_j == L - 1, max(h[1][s, j] for h in held) == 0,
                         min(h[0][s, j] for h in held) == L - 1)))  # (the rule that holds least: at the end too)
    for way, kk in (("up", ku), ("down", kd)):  # (on target one way only: come to that way)
        one = np.where(np.all(kk == kk[:1], axis=0), kk[0], -1)
        lo1, hi1 = _runs(one)
        f1 = np.take_along_axis(M, kk[0][:, :, None], axis=2)[:, :, 0] - target
        hit1 = (one[:, :-1] >= 0) & (one[:, :-1] == one[:, 1:]) & (f1[:, :-1] * f1[:, 1:] <= 0) & \
            (f1[:, :-1] != f1[:, 1:]) & ~agree[:, :-1] & ~agree[:, 1:] & inside[None, :]
        for s, j in zip(*np.nonzero(hit1)):
            i = float(cur[j] + (cur[j + 1] - cur[j]) * f1[s, j] / (f1[s, j] - f1[s, j + 1]))
            a, b = lo1[s, j], hi1[s, j]
            w_lo, w_hi = (float(w) for w in m.window(i, pz[s]))
            out.append(Spot(T, float(pz[s]), float(pz[s]) - m.v_set_to_act, i, int(orders[s, one[s, j]]), (0.0, 0.0),
                            (i - float(cur[a]), float(cur[b]) - i), (target - w_lo, w_hi - target),
                            (False, False, a == 0, b == L - 1), way=way,
                            entry=float(cur[a]) if way == "up" else float(cur[b])))
    return out


def _distinct(spots: list, most: int, robust: bool = False, dT: float = 0.15, dV: float = 3.0) -> list:
    """The best spots (best first: by their score, or robust: by the worst of it and theirs with the temperature's
    other coefficients), none within dT K and dV V of a better one on the same mode."""
    out = []
    for sp in sorted(spots, key=lambda q: (q.way != "either",
                                           -(min(q.score, q.worst) if robust and q.worst is not None else q.score))):
        if all(q.order != sp.order or abs(q.v - sp.v) > dV or
               (q.T is not None and sp.T is not None and abs(q.T - sp.T) > dT) for q in out):
            out.append(sp)
            if len(out) >= most:
                break
    return out


def _holds(m: LaserModel, P: np.ndarray, ii: np.ndarray, vv: np.ndarray, shift: np.ndarray, start: np.ndarray,
           up) -> np.ndarray:
    """Along paths at rest (rows: ii mA, vv V, shift GHz added to its external modes), each from its first point on
    the mode at start GHz there: how many steps each holds on it, by every rule (P), its hysteresis as `up` has it
    (each path's, or all of them alike): rules x paths."""
    n, L = ii.shape
    e = m.ext.base(ii - m.i0, vv - m.v0, 0.0) + shift
    half = _span(m, P)
    k0 = np.round((start - e[:, 0]) / m.ext.spacing).astype(int)
    orders = k0[:, None] + np.arange(-half, half + 1)[None, :]
    pk = {"M": e[:, :, None] + orders[:, None, :] * m.ext.spacing, "B": m.chip_base(ii, vv, m.t0), "A": ii, "V": vv,
          "T": np.broadcast_to(np.arange(L, dtype=float)[None, :] * REST, (n, L)),
          "up": np.broadcast_to(np.asarray(up, dtype=bool).reshape(-1, 1), (n, L)),
          "ci0": m._ci(), "i0": m.i0, "v0": m.v0}
    k = _select(pk, P, np.full(n, half))
    moved = k != half
    return np.where(moved.any(axis=2), np.argmax(moved, axis=2), L)


def _margins(m: LaserModel, P: np.ndarray, sp: Spot, target: float, ffs: list, v_range: tuple, i_range: tuple,
             dv: float = 0.05, di: float = 0.02):
    """sp's margins at rest, parked on its mode, by the model there (m): the current either way, the piezo kept,
    its hysteresis as the current goes (sp.held: one way, exactly, where the sweeps had only its least); the piezo
    either way, the current kept (sp.piezo); the drift of its cavity a lock takes up before it hops, by each
    feedforward (sp.lock). The current kept, its hysteresis is the way it came to it (one way), or the lesser of both
    (either way)."""
    cs = np.arange(int(REACH_I / di) + 1) * di
    steps = np.arange(int(REACH_V / dv) + 1) * dv
    rows = [(sp.i - cs, np.full(len(cs), sp.v), np.zeros(len(cs)), False),  # (I, V, shift, the current rising)
            (sp.i + cs, np.full(len(cs), sp.v), np.zeros(len(cs)), True)]
    came = None if sp.way == "either" else sp.way == "up"  # (the current kept: the way it came to it holds on)
    for sign in (-1, 1):
        rows.append((np.full(len(steps), sp.i), sp.v + sign * steps, np.zeros(len(steps)), came))
    e0 = float(m.ext.base(sp.i - m.i0, sp.v - m.v0, 0.0))
    for ff in ffs:
        tune = m.ext.p_v + m.ext.p_i * ff  # (GHz/V the external modes move along it, about)
        reach = min(REACH_DRIFT / max(abs(tune), 1e-3), 4 * REACH_V)
        dvs = np.arange(int(reach / dv) + 1) * dv
        for sign in (-1, 1):
            v = sp.v + sign * dvs
            i = sp.i + ff * sign * dvs
            shift = -(m.ext.base(i - m.i0, v - m.v0, 0.0) - e0)  # (its external modes held: the lock's doing)
            rows.append((i, v, shift, came if ff == 0 else bool(ff * sign > 0)))
    L = max(len(r[0]) for r in rows)
    ii, vv, S = (np.array([np.concatenate([r[c], np.full(L - len(r[c]), r[c][-1])]) for r in rows]) for c in range(3))
    ups = [r[3] for r in rows]
    held = np.minimum(_holds(m, P, ii, vv, S, np.full(len(rows), target), [u is not False for u in ups]),
                      _holds(m, P, ii, vv, S, np.full(len(rows), target), [u is True for u in ups])).min(axis=0)
    inside = (vv >= v_range[0] - 1e-9) & (vv <= v_range[1] + 1e-9) & (ii >= i_range[0] - 1e-9) & \
        (ii <= i_range[1] + 1e-9)
    stop = np.argmin(np.concatenate([inside, np.zeros((len(rows), 1), dtype=bool)], axis=1), axis=1)
    last = np.minimum(held, stop) - 1  # (the last step on it, within what was searched)
    k = np.maximum(last, 0)
    edge = stop <= held  # (what was searched ended before it hopped: at least that)
    sp.held = (float(sp.i - ii[0, k[0]]), float(ii[1, k[1]] - sp.i))
    sp.edges = (sp.edges[0], sp.edges[1], bool(edge[0]), bool(edge[1]))
    sp.piezo = tuple(float(abs(vv[r, k[r]] - sp.v)) for r in (2, 3))
    sp.lock = {}
    for f, ff in enumerate(ffs):
        a, b = 4 + 2 * f, 5 + 2 * f
        drift = [float(S[r, k[r]]) for r in (a, b)]  # (GHz its cavity drifted, taken up)
        sp.lock[ff] = (min(drift), max(drift))


def park(model: LaserModel, target_thz: float, temps: tuple | None = None, currents: tuple | None = None,
         piezo: tuple | None = None, t_step: float = 0.05, v_step: float = 0.5, i_step: float = 0.05,
         feedforward: float | None = None, top: int = 8, refine: bool = True, aliases: bool = True,
         robust: bool = False, log=None) -> list:
    """The best spots to park the laser at target_thz (THz) by its model, best first (Spot; the module's docstring
    says how): over temperatures temps (°C, lo and hi; None: those its map was made at), currents (mA) and piezo
    voltages (V, as measured: the model's) across what its map covered unless given, in steps of t_step, v_step and
    i_step, refined about the best (refine); top: how many, none within 0.15 K and 3 V of a better one on its mode.
    feedforward: mA/V, the controller's (its lock's margins said for it too, with none and the ideal). aliases: the
    best again with the temperature's other coefficients; robust: every spot so, and ranked by the worst of it and
    those (what its map can vouch for, whichever the laser's are). log(text): progress."""
    say = log or (lambda text: None)
    target = (target_thz - model.f_ref) * 1e3
    P = _rules(model)
    if model.T0 is not None and model.temps:
        t_lo, t_hi = temps if temps is not None else (min(model.temps), max(model.temps))
        Ts = list(_grid(t_lo, t_hi, t_step))
    else:
        Ts = [None]
    i_range = tuple(currents) if currents is not None else tuple(model.i_span or (model.i0 - 5, model.i0 + 5))
    v_range = tuple(piezo) if piezo is not None else tuple(model.v_span or (model.v0 - 20, model.v0 + 20))
    cur, pz = _grid(i_range[0] - BEYOND, i_range[1] + BEYOND, i_step), _grid(*v_range, v_step)
    others = _other_models(model) if aliases or robust else []
    alt_rules = [_rules(alt) for _, alt in others]

    def scan(T, volts) -> list:  # (the spots at T over those piezo voltages; robust: each one's worst besides)
        found = _spots(model.at(T), P, T, cur, volts, target, i_range)
        if robust and found:
            wide = _grid(max(float(volts[0]) - NEAR_V, v_range[0]), min(float(volts[-1]) + NEAR_V, v_range[1]),
                         volts[1] - volts[0] if len(volts) > 1 else v_step)
            for ((c_T, e_T), alt), Q in zip(others, alt_rules):
                _worst(found, c_T, e_T, _spots(alt.at(T), Q, T, cur, wide, target, i_range))
        return found

    spots = []
    for k, T in enumerate(Ts):
        say(f"where it parks: {k + 1} of {len(Ts)} temperatures" if T is not None else "where it parks")
        spots += scan(T, pz)
    say(f"{len(spots):,} spots on target over {len(Ts)} temperatures, {len(pz)} piezo voltages, {len(cur)} currents"
        + (f", each with the temperature's {len(others)} other coefficients" if robust and others else ""))
    if refine and spots:
        best = _distinct(spots, max(3, top // 2), robust)
        for k, sp in enumerate(best):
            say(f"where it parks: about the best, {k + 1} of {len(best)}, finer")
            fine_T = [None] if sp.T is None else \
                [T for T in _grid(sp.T - t_step, sp.T + t_step, t_step / 5) if Ts[0] - 1e-9 <= T <= Ts[-1] + 1e-9]
            fine_V = _grid(max(sp.v - 2 * v_step, v_range[0]), min(sp.v + 2 * v_step, v_range[1]), v_step / 5)
            for T in fine_T:
                spots += scan(T, fine_V)
        say(f"refined about the best {len(best)}: {len(spots):,} spots")
    ff_ideal = model.ideal_feedforward()
    ffs = [0.0] + [f for f in (feedforward, ff_ideal) if f is not None and abs(f) > 1e-9]
    period = _hop_period(model)
    best = _distinct(spots, 3 * top, robust)
    for k, sp in enumerate(best):  # (its margins parked there, exactly: ranked again by them, those that hold
        say(f"how far each holds: {k + 1} of {len(best)}")  # nothing left out)
        _margins(model.at(sp.T), P, sp, target, ffs, v_range, i_range)
        if sp.way != "either":
            sp.start = max(sp.entry - 2 * period, i_range[0]) if sp.way == "up" else \
                min(sp.entry + 2 * period, i_range[1])
    out = _distinct([sp for sp in best if sp.score >= MIN_HOLD], top, robust)
    for k, sp in enumerate(out):
        say(f"the temperature's other coefficients: {k + 1} of {len(out)}")
        if aliases and not sp.worst_of:
            near_v = _grid(max(sp.v - NEAR_V, v_range[0]), min(sp.v + NEAR_V, v_range[1]), v_step / 2)
            for ((c_T, e_T), alt), Q in zip(others, alt_rules):
                _worst([sp], c_T, e_T, _spots(alt.at(sp.T), Q, sp.T, cur, near_v, target, i_range))
    return out


def _hop_period(m: LaserModel) -> float:
    """mA: how often it hops sweeping the current (from one chip mode to the next: the vernier), 1.5 if not
    known."""
    e, c = m.ext, m.chip
    if c is None or m.single or abs(c.p_i - e.p_i) < 1e-6:
        return 1.5
    step = abs(c.spacing - e.spacing * round(c.spacing / e.spacing)) or e.spacing
    return float(min(max(step / abs(c.p_i - e.p_i), 0.2), 5.0))


def _worst(spots: list, c_T: float, e_T: float, alts: list):
    """With the temperature's coefficients c_T, e_T (GHz/K: the chip's, the external modes'), the best of alts (the
    spots then) near each of spots (NEAR_V, NEAR_I): kept in its worst_of, its worst the least of those."""
    if alts:
        av, ai = np.array([q.v for q in alts]), np.array([q.i for q in alts])
        sc = np.array([q.score for q in alts])
    for sp in spots:
        k = None
        if alts:
            near = np.flatnonzero((np.abs(av - sp.v) <= NEAR_V + 1e-9) & (np.abs(ai - sp.i) <= NEAR_I + 1e-9))
            k = int(near[np.argmax(sc[near])]) if len(near) else None
        q = alts[k] if k is not None else None
        sp.worst_of.append((c_T, e_T, q.score if q else 0.0, q.v if q else None, q.i if q else None))
        sp.worst = min(w[2] for w in sp.worst_of)


def _other_models(model: LaserModel) -> list:
    """[((chip GHz/K, external GHz/K), the model with them)]: its temperature's other coefficients its map allows as
    well (the nearest of each comb's, and both together), each agreeing with it at the temperatures mapped."""
    a = model.aliases or {}
    if model.T0 is None or not model.temps:
        return []
    c, e = (model.chip.p_T if model.chip is not None else 0.0), model.ext.p_T
    cs = [c] + list(a.get("chip") or [])[:1]
    es = [e] + list(a.get("ext") or [])[:1]
    return [((ct, et), model.with_temps(ct, et)) for ct in cs for et in es if (ct, et) != (c, e)]


def say_spot(sp: Spot) -> str:
    """One spot, in words."""
    def least(x, edge):
        return f"{x:.2f}" + ("+" if edge else "")

    where = (f"{sp.T:.2f} °C, " if sp.T is not None else "") + \
        f"piezo {sp.v_set:.2f} V set ({sp.v:.2f} V measured), {sp.i:.2f} mA"
    if sp.way == "either":
        text = (f"{where}: its mode (order {sp.order:+d}) the same sweeping up and down from "
                f"{sp.i - sp.core[0]:.2f} to {sp.i + sp.core[1]:.2f} mA (-{least(sp.core[0], sp.edges[0])}/"
                f"+{least(sp.core[1], sp.edges[1])} mA about it); parked on it, it holds from "
                f"{sp.i - sp.held[0]:.2f} to {sp.i + sp.held[1]:.2f} mA (-{least(sp.held[0], sp.edges[2])}/"
                f"+{least(sp.held[1], sp.edges[3])})")
    else:
        text = (f"{where}: its mode (order {sp.order:+d}) only come to sweeping the current {sp.way} (from "
                f"{'below' if sp.way == 'up' else 'above'}: the other way it is on another); parked on it, it holds "
                f"from {sp.i - sp.held[0]:.2f} to {sp.i + sp.held[1]:.2f} mA "
                f"(-{least(sp.held[0], sp.edges[2])}/+{least(sp.held[1], sp.edges[3])})")
    if sp.piezo is not None:
        text += f"; the current kept, the piezo -{sp.piezo[0]:.1f}/+{sp.piezo[1]:.1f} V"
    if sp.lock:
        text += "; a lock on the piezo takes up a drift of its cavity of " + ", ".join(
            f"{lo:+.1f}/{hi:+.1f} GHz " + ("with no feedforward" if ff == 0 else f"at {ff:+.3f} mA/V")
            for ff, (lo, hi) in sp.lock.items())
    if all(math.isfinite(w) for w in sp.window):
        text += f"; {sp.window[0]:.1f} GHz above its window's low edge, {sp.window[1]:.1f} below its high one"
    if sp.worst_of:
        text += ("; were the temperature's coefficients others its map allows as well (the chip's, the external "
                 "modes' GHz/K), the best spot near it would hold " + ", ".join(
                     f"±{w:.2f} mA at {v:.1f} V, {i:.2f} mA ({c:+.2f}, {e:+.2f})" if v is not None else
                     f"none ({c:+.2f}, {e:+.2f})" for c, e, w, v, i in sp.worst_of))
    return text
