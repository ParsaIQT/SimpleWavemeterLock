"""Mode-hop recovery by a laser's own model (lasermodel.py, fitted to its map): where to put the laser, and how to
come to it, so that it lands on a mode on target, with margins, the first time.

A Planner holds the model, its frequencies made GHz from the target, anchored to the laser as it is now:
- to where the laser was on target before it hopped (ref): its right mode is on target there, which places the
  external comb (the model's own drift terms are left out), and the target is inside its window there (it lased);
- to what the laser does in the recovery (observe: every reading, sweep by sweep). fit() puts the external comb where
  the readings have it (it may have drifted since the ref), then finds where its chip's modes are (phi GHz from where
  its map had them: their temperature, their age) and where its window is (slide GHz: the gain, with the chip's
  temperature), as most of the readings say: each sweep simulated from its first reading, the modes it lases on
  those it was read on; and every other phi and slide that does as well, the readings not telling them apart; for
  its rule and for the one held the other way (the model's alt: its map tells the two apart only so far), the one
  that does better the model's from then on, the other kept as well if it does nearly as well.
plan() then finds, for piezo voltages across its range, the current that puts a mode on target there (any of the
external modes: the one it was on, or one a few orders away at another piezo voltage, whichever holds best), and
comes to it from either end of the window as the model says the laser would (to the far end and back first, as the
recovery does: lasermodel's rule, its hysteresis, its chip lagging the current, all of it), keeping those where it
arrives on that mode, with margins: how far the current may then go on, or back, before it hops. Not by the model as
fitted alone, but by its members (members(): its chip's modes and its window as far either way as the readings
allow, a little either way at least): the plan most of them have arrive and hold, with the widest margins, so that
what the model cannot know does not take the laser off its mode; its path is what each reading on the way should be.
A plan that did not hold is not made again (avoid).

Units: currents mA as measured, the piezo in volts as set (the model's, as its map measured it, is v_set_to_act
off), frequencies GHz from the target. Needs numpy.
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field

import numpy as np

from .lasermodel import RULE, WINDOW, LaserModel, _losses, _select

RAMP_STEP = 0.2   # mA: a quick move's steps (the recovery's STEP_MAX)
RAMP_DT = 0.05    # s each (no reading)
FINE = 0.02       # mA: the steps margins are found in...
REACH = 1.5       # mA: ...as far as this
V_STEP = 0.5      # V between the piezo voltages a plan looks at (then finer about the best)
PHI_MIN = 0.02    # of the chip's mode spacing: the least its modes are taken to be uncertain by, either way
WINDOW_SLIP = 3.0  # GHz: the least its window is taken to be uncertain by, either way (a tenth of its width at most)
WINDOW_IN = 1.0   # GHz: the target at least this far inside its window where it was on target (a tenth at most)
SLIDES = 24       # window positions fit() tries, at most...
WINDOW_DRIFT = 0.1  # ...as far as this much of its width from where the model has it (a stable diode's drifts no more)
MEMBER_GAP = 0.08  # the rule held the other way kept as members if it puts this much less of the readings right at most
SPREAD = 3        # members either side where the readings allow its window further than WINDOW_SLIP from the model's
AVOID_V = 1.5     # V: a plan this near one that did not hold (the same mode, the same way) is not made again


@dataclass
class Plan:
    """Where to put the laser: a mode (its order) on target at piezo v (V set) and current i (mA measured), come
    to sweeping up from the window's bottom or down from its top (start, mA measured); margins (mA) the model gives
    it there, the median of its members that have it arrive: ahead (on the way it came) and behind, before it hops;
    path: [(mA, GHz)] the first of those expects on the way, a reading a step; share: of the model's members
    (Planner.members()), those that have it arrive and hold with the margins asked for; robust: the least margin of
    theirs (0: one has it arrive on another mode)."""
    order: int
    v: float
    i: float
    up: bool
    start: float
    ahead: float
    behind: float
    score: float = 0.0
    path: list = field(default_factory=list)
    share: float = 1.0
    robust: float = 0.0

    @property
    def margin(self) -> float:
        return min(self.ahead, self.behind)

    @property
    def way(self) -> str:
        return "up" if self.up else "down"


@dataclass
class Seen:
    seg: int      # which sweep
    t: float      # s
    i: float      # mA measured
    v: float      # V set
    up: bool      # the current rising
    ghz: float    # GHz from the target (NaN: no valid reading)


class Planner:
    def __init__(self, model: LaserModel, target_thz: float, ref=None, v_off: float | None = None):
        """model: the laser's (its frequencies are made GHz from the target here); ref: (mA measured, V set) where
        it was on target on its right mode, or None; v_off: the model's piezo voltage less the one set (None: the
        model's v_set_to_act)."""
        m = copy.deepcopy(model)
        shift = (m.f_ref - target_thz) * 1e3
        m.f_ref = target_thz
        m.ext.p0 += shift
        m.ext.p_t, m.ext.others = 0.0, []
        if m.chip is not None:
            m.chip.p0 += shift
            m.chip.p_t = 0.0
        rules = [m.rule()]
        if model.alt is not None:
            alt = {"soft": model.soft, **model.alt}  # (kept before its window's edges were soft: as its own)
            rules.append(np.array([alt[k] for k in RULE + ("dc", "ci") + WINDOW], dtype=float))
        for P in rules:  # (their windows' frequencies too)
            if math.isfinite(P[0]) and math.isfinite(P[1]):
                P[0] += shift
                P[1] += shift
        self.m = m
        self.rules = rules       # its rule and the one held the other way (as _select has them; the window moved
        #                          to have the target in it where it was on target)
        self.which = 0           # the rule the readings say best (fit())
        self.v_off = m.v_set_to_act if v_off is None else v_off
        self.phi = 0.0           # GHz: its chip's modes from where the model had them...
        self.slide = 0.0         # ...and its window
        self.ensemble = []       # [(rule, phi, slide)]: its members' (members())
        self.drift = 0.0         # GHz: its external comb, moved to where the readings have it
        self.slid = 0.0          # GHz its window was moved by to have the target in it where it was on target
        self.right = None        # the right mode's order
        self.seen = []           # [Seen]
        self.agree = None        # the share of the readings fit() put on their modes...
        self.null = None         # ...and of those on the mode their sweep began on (a model that never hops)...
        self.on_comb = None      # ...and of all of them, those on its external comb
        self._seg = 0
        _apply(m, rules[0])
        if ref is not None:
            self.anchor(*ref)

    # ------------------------------------------------------------- the model
    def _dv(self, v):
        return np.asarray(v, dtype=float) + self.v_off

    def base(self, i, v, up=None):
        """GHz: the external comb's order-0 mode at (i mA, v V set)."""
        m = self.m
        di, dv, _ = m._d(i, self._dv(v), m.t0)
        return m.ext.base(di, dv, 0.0, up)

    def ghz(self, order, i, v, up=None):
        """GHz from the target: a mode's frequency at (i, v)."""
        return self.base(i, v, up) + order * self.m.ext.spacing

    def order_of(self, i, v, ghz) -> int:
        """The order of the mode a reading (GHz) at (i, v) is on."""
        return round(float((ghz - self.base(i, v)) / self.m.ext.spacing))

    def kv(self, i, v) -> float:
        """MHz/V: how the piezo tunes a mode at (i, v)."""
        e, dv = self.m.ext, float(self._dv(v)) - self.m.v0
        return 1e3 * (e.p_v + 2 * e.p_vv * dv)

    def slope(self, i, v) -> float:
        """MHz/mA: how the current tunes a mode at (i, v)."""
        e, di = self.m.ext, float(i) - self.m.i0
        return 1e3 * (e.p_i + 2 * e.p_ii * di)

    def current_for(self, order: int, v: float, i_guess: float) -> float:
        """mA: the current that puts that mode on target at v (Newton's, the comb being quadratic in it)."""
        i = float(i_guess)
        for _ in range(4):
            f = float(self.ghz(order, i, v))
            d = self.slope(i, v) / 1e3
            if abs(d) < 1e-9:
                return math.nan
            i -= f / d
        return i

    def anchor(self, i: float, v: float, ghz: float = 0.0):
        """The right mode on target (ghz from it) at (i, v): the external comb placed so, its order kept; and the
        target inside its window there by each rule (it lased there: the window moved so, if it was not, WINDOW_IN
        inside)."""
        m = self.m
        e = m.ext
        b = float(self.base(i, v))
        n = round((ghz - b) / e.spacing)
        e.p0 += ghz - (b + n * e.spacing)
        self.right = int(n)
        for r, P in enumerate(self.rules):
            if math.isfinite(P[0]) and math.isfinite(P[1]):
                shift = P[9] * (float(self._dv(v)) - m.v0) + P[10] * (i - m.i0)
                lo, hi = P[0] + shift, P[1] + shift
                edge = min(WINDOW_IN, 0.1 * _width(P)) - _beyond(P)
                slid = max(ghz - (hi - edge), 0.0) + min(ghz - (lo + edge), 0.0)
                P[0] += slid
                P[1] += slid
                if r == 0:
                    self.slid = slid
        _apply(m, self._rule(self.which, self.phi, self.slide))

    def _rule(self, r: int, phi: float, slide: float) -> np.ndarray:
        """Rule r (as _select has it) with its chip's modes phi and its window slide GHz from the model's."""
        P = self.rules[r].copy()
        P[4] += phi
        if math.isfinite(P[0]) and math.isfinite(P[1]):
            P[0] += slide
            P[1] += slide
        return P

    def _bounded(self) -> bool:
        return math.isfinite(self.rules[0][0]) and math.isfinite(self.rules[0][1])

    def members(self) -> np.ndarray:
        """The rules a plan must hold for (as _select has them), the model as fitted first: its chip's modes and its
        window wherever the readings allow, by its rule and the one held the other way if that does nearly as well
        (fit()), and PHI_MIN of their spacing, WINDOW_SLIP, either way at least. Before a fit: by each rule, those
        least ways about where the model has them."""
        if self.ensemble:
            return np.array([self._rule(r, phi, slide) for r, phi, slide in self.ensemble])
        out = []
        for r in range(len(self.rules)):
            d = PHI_MIN * self.rules[r][7]
            out += [(r, self.phi, self.slide), (r, self.phi + d, self.slide), (r, self.phi - d, self.slide)]
            if self._bounded():
                w = min(WINDOW_SLIP, 0.1 * _width(self.rules[r]))
                out += [(r, self.phi, self.slide + w), (r, self.phi, self.slide - w)]
        if self.which:
            out.sort(key=lambda x: x[0] != self.which)  # (the model's own first)
        return np.array([self._rule(r, phi, slide) for r, phi, slide in out])

    # --------------------------------------------------------- what it did
    def segment(self) -> int:
        """A new sweep's number (its readings observed under it)."""
        self._seg += 1
        return self._seg

    def observe(self, seg: int, t: float, i: float, v: float, up: bool, ghz: float):
        self.seen.append(Seen(seg, t, float(i), float(v), bool(up), float(ghz)))

    def _paths(self, rows, orders: np.ndarray) -> dict:
        """Paths as _select's pk: rows [(mA, V set, s, up)] of arrays each; padded to the longest."""
        m = self.m
        n, L = len(rows), max(len(r[0]) for r in rows)
        K = len(orders)
        M, B, A, V, T = (np.zeros(shape) for shape in ((n, L, K), (n, L), (n, L), (n, L), (n, L)))
        up = np.zeros((n, L), dtype=bool)
        for s, (i, v, t, u) in enumerate(rows):
            i, t = np.asarray(i, dtype=float), np.asarray(t, dtype=float)
            k = len(i)
            u = np.broadcast_to(np.asarray(u, dtype=bool), (k,))
            mv = np.broadcast_to(self._dv(v), (k,)).astype(float)
            modes = m.ext_modes(i, mv, np.full(k, m.t0), orders, u)
            base = m.chip_base(i, mv, np.full(k, m.t0))
            for arr, val in ((M, modes), (B, base), (A, i), (V, mv)):
                arr[s, :k] = val
                arr[s, k:] = val[-1]
            T[s, :k], T[s, k:] = t, t[-1] + np.arange(1, L - k + 1)
            up[s, :k], up[s, k:] = u, u[-1]
        return {"M": M, "B": B, "A": A, "V": V, "T": T, "up": up, "ci0": m._ci(), "i0": m.i0, "v0": m.v0}

    def _orders(self, lo: float, hi: float, base_lo: float, base_hi: float) -> np.ndarray:
        """The orders whose modes can come within lo..hi GHz while their comb's order 0 is within base_lo..base_hi."""
        e = self.m.ext.spacing
        return np.arange(math.floor((lo - base_hi) / e) - 1, math.ceil((hi - base_lo) / e) + 2)

    def _window(self, i, v):
        lo, hi = self.m.window(np.asarray(i, dtype=float), self._dv(v))
        return lo, hi

    def fit(self, grid: int = 160) -> float | None:
        """From what it did: its external comb where the readings have it, then its chip's modes (phi) and its
        window (slide) where most readings are on the modes they were on (each sweep simulated from its first
        reading), by its rule and by the one held the other way; the share they then are, by the rule that puts
        more of them right, the model's from then on (also self.agree; None: nothing to fit; self.null: the share a
        model that never hops would have right; self.on_comb: of all the readings, those on its external comb,
        within a fifth of its spacing of one of its modes, where readings off it would be two in five). Of the phi
        and slide that do as well: the window as near where the model has it as they allow, phi the middle of the
        widest stretch of them there; and as members, the furthest either way of both, and between, by each rule
        that does nearly as well (members())."""
        m = self.m
        segs = {}
        for s in self.seen:
            if math.isfinite(s.ghz):
                segs.setdefault(s.seg, []).append(s)
        segs = [ss for ss in segs.values() if len(ss) >= 2]
        if not segs:
            self.agree = None
            return None
        self._recomb(segs)
        rows = [tuple(np.array([getattr(s, k) for s in ss]) for k in ("i", "v", "t", "up")) for ss in segs]
        ghz = [np.array([s.ghz for s in ss]) for ss in segs]
        allb = np.concatenate([self.base([s.i for s in ss], [s.v for s in ss]) for ss in segs])
        allf = np.concatenate(ghz)
        orders = self._orders(float(allf.min()) - 5, float(allf.max()) + 5, float(allb.min()), float(allb.max()))
        pk = self._paths(rows, orders)
        n, L = pk["A"].shape
        meas, valid = np.zeros((n, L), dtype=int), np.zeros((n, L), dtype=bool)
        for k, (ss, f) in enumerate(zip(segs, ghz)):
            modes = pk["M"][k, :len(ss)]
            near = np.argmin(np.abs(modes - f[:, None]), axis=1)
            meas[k, :len(ss)] = near
            valid[k, 1:len(ss)] = np.abs(modes[np.arange(len(ss)), near] - f)[1:] < 0.35 * m.ext.spacing
        pk["meas"], pk["valid"] = meas, valid
        total = max(int(valid.sum()), 1)
        off = [np.min(np.abs(pk["M"][k, :len(ss)] - f[:, None]), axis=1) for k, (ss, f) in enumerate(zip(segs, ghz))]
        self.on_comb = float(np.mean(np.concatenate(off) < 0.2 * m.ext.spacing))
        self.null = int(((meas == meas[:, :1]) & valid).sum()) / total
        fits = []
        for r in range(len(self.rules)):
            dc = float(self.rules[r][7])
            phis = np.linspace(-dc / 2, dc / 2, grid, endpoint=False)
            slides = self._slides(segs, r)
            P = np.array([self._rule(r, phi, slide) for slide in slides for phi in phis])
            score = (_select(pk, P, meas[:, 0], scored=True) / total).reshape(len(slides), grid)
            best = float(score.max())
            good = score >= best - 0.5 / total
            ok = np.flatnonzero(good.any(axis=1))
            r0 = int(ok[np.argmin(np.abs(slides[ok]))])  # (its window as near where the model has it as can be)
            phi, spread = _middle(good[r0], dc)
            slide = float(slides[r0])
            d = max(spread, PHI_MIN * dc)
            members = [(r, phi, slide), (r, phi + d, slide), (r, phi - d, slide)]
            if self._bounded():
                w = min(WINDOW_SLIP, 0.1 * _width(self.rules[r]))
                for side, sign in ((ok[slides[ok] <= slide - w], -1.0), (ok[slides[ok] >= slide + w][::-1], 1.0)):
                    if len(side):  # (as far as the readings allow, and between; its chip's modes as they say there)
                        pick = np.unique(np.round(np.linspace(0, len(side) - 1, min(SPREAD, len(side)))).astype(int))
                        members += [(r, _middle(good[side[k]], dc)[0], float(slides[side[k]])) for k in pick]
                    else:
                        members.append((r, phi, slide + sign * w))
            fits.append((best, r, phi, slide, members))
        top = max(fits, key=lambda f: (f[0], -f[1]))
        self.agree, self.which, self.phi, self.slide = top[0], top[1], top[2], top[3]
        self.ensemble = top[4] + [x for f in fits if f is not top and f[0] >= top[0] - MEMBER_GAP for x in f[4]]
        _apply(m, self._rule(self.which, self.phi, self.slide))
        return self.agree

    def _recomb(self, segs):
        """Its external comb moved to where the readings have it (the mean of each reading's offset from the mode
        nearest it, around the comb's spacing; it may have drifted since it was anchored)."""
        e = self.m.ext
        x = np.array([(s.ghz - float(self.base(s.i, s.v, s.up))) / e.spacing for ss in segs for s in ss])
        z = complex(np.exp(2j * np.pi * x).mean())
        if abs(z) > 0.5:  # (the readings on one comb)
            d = math.atan2(z.imag, z.real) / (2 * math.pi) * e.spacing
            e.p0 += d
            self.drift += d

    def _slides(self, segs, r: int = 0) -> np.ndarray:
        """GHz: where its window may be by rule r, from where the model has it: each reading inside it (2 GHz out at
        most, and as far into soft edges as a mode lases: _beyond), and no further than a stable diode's window drifts
        (WINDOW_DRIFT of its width) from where the model
        has it, or from as near as the readings allow; a chip mode's eighth apart (SLIDES at most), and where the
        model has it if that is one."""
        m, P = self.m, self.rules[r]
        if not (math.isfinite(P[0]) and math.isfinite(P[1])):
            return np.zeros(1)
        f = np.array([s.ghz for ss in segs for s in ss])
        i = np.array([s.i for ss in segs for s in ss])
        v = np.array([s.v for ss in segs for s in ss])
        shift = P[9] * (self._dv(v) - m.v0) + P[10] * (i - m.i0)
        out_by = 2.0 + _beyond(P)
        a = float(np.max(f - (P[1] + shift))) - out_by  # (the least that has every reading below its top)
        b = float(np.min(f - (P[0] + shift))) + out_by  # (the most that has every reading above its bottom)
        if a > b:
            return np.array([0.5 * (a + b)])
        d = WINDOW_DRIFT * _width(P)
        a, b = (max(a, -d), min(b, d)) if a <= 0.0 <= b else (max(a, b - d), b) if b < 0.0 else (a, min(b, a + d))
        out = np.linspace(a, b, min(SLIDES, max(1, math.ceil((b - a) / (P[7] / 8)))) + 1)
        if a <= 0.0 <= b:
            out = np.append(out, 0.0)
        return np.unique(out)

    # -------------------------------------------------------------- the plan
    def plan(self, i_lo: float, i_hi: float, v_lo: float, v_hi: float, step: float, i_nom: float, v_now: float,
             step_s: float = 0.2, want: float = 0.5, orders=None, fine: bool = True, avoid=()) -> Plan | None:
        """The best way onto a mode on target (orders: those to consider; None: every one that can be on target
        within i_lo..i_hi and v_lo..v_hi): for piezo voltages from v_lo to v_hi (V_STEP apart, then finer about the
        best), the current putting that mode on target; that within i_lo..i_hi, come to from either end of the
        window (by way of the other end first, as the recovery's come_to does, quickly), in `step` mA, a reading
        each (step_s s). Kept where any of its members has it arrive on that mode, and not near one that did not
        hold (avoid: [(V, mA, up)]); the best by the share of its members that have it arrive and hold (want: the
        margins asked for), then by their margins, little piezo and the current near i_nom. None: no way."""
        if orders is None:
            ns = [self.order_of(i, v, 0.0) for i in (i_lo, i_hi) for v in (v_lo, v_hi)]
            orders = range(min(ns) - 1, max(ns) + 2)
        vs = np.arange(v_lo, v_hi + 1e-9, V_STEP)
        cands = []
        for n in orders:
            for v in vs:
                i = self.current_for(n, float(v), i_nom)
                if math.isfinite(i) and i_lo + FINE <= i <= i_hi - FINE:
                    cands.append((n, float(v), i))
        best = self._weigh(cands, i_lo, i_hi, step, i_nom, v_now, step_s, want, avoid)
        if best is not None and fine:  # (finer about it)
            around = np.arange(best.v - V_STEP, best.v + V_STEP + 1e-9, 0.1)
            more = [(best.order, float(v), self.current_for(best.order, float(v), best.i)) for v in around
                    if v_lo <= v <= v_hi]
            more = [c for c in more if math.isfinite(c[2]) and i_lo + FINE <= c[2] <= i_hi - FINE]
            better = self._weigh(more, i_lo, i_hi, step, i_nom, v_now, step_s, want, avoid)
            if better is not None and better.score > best.score:
                best = better
        return best

    def _way(self, i: float, up: bool, i_lo: float, i_hi: float, step: float, step_s: float):
        """The way to i, coming up from the window's bottom (or down from its top) by way of the other end, quickly:
        (mA, s, up) arrays, and where the sweep that reads begins (its index)."""
        start, other = (i_lo, i_hi) if up else (i_hi, i_lo)
        pre, app = _grid(other, start, RAMP_STEP), _grid(start, i, step)[1:]
        t = np.concatenate([np.arange(len(pre)) * RAMP_DT, len(pre) * RAMP_DT + np.arange(1, len(app) + 1) * step_s])
        return np.concatenate([pre, app]), t, np.concatenate([np.full(len(pre), not up), np.full(len(app), up)]), \
            len(pre)

    def _weigh(self, cands, i_lo, i_hi, step, i_nom, v_now, step_s, want, avoid=()) -> Plan | None:
        """The candidates (order, V set, mA) each come to both ways, and on past (ahead) or back (behind) in FINE
        steps, by each of the members' rules: the best Plan (plan())."""
        def avoided(v, i, up=None):
            return any(abs(v - av) < AVOID_V and abs(i - ai) < 0.5 and up in (None, au) for av, ai, au in avoid)

        cands = [c for c in cands if not (avoided(c[1], c[2], True) and avoided(c[1], c[2], False))]
        if not cands:
            return None
        m = self.m
        rows, keys = [], []
        for c, (_, v, i) in enumerate(cands):
            for up in (True, False):
                ii, t, uu, npre = self._way(i, up, i_lo, i_hi, step, step_s)
                for ahead in (True, False):
                    tail = _grid(i, i + (REACH if up == ahead else -REACH), FINE)[1:]
                    rows.append((np.concatenate([ii, tail]), v,
                                 np.concatenate([t, t[-1] + step_s * np.arange(1, len(tail) + 1)]),
                                 np.concatenate([uu, np.full(len(tail), up == ahead)])))
                    keys.append((c, up, ahead, npre, len(ii)))
        ii_all = np.array([c[2] for c in cands])
        vv_all = np.array([c[1] for c in cands])
        bs = self.base(ii_all, vv_all)
        P = self.members()
        if self._bounded():  # (all its members' windows)
            shift = [Q[9] * (self._dv(vv_all) - m.v0) + Q[10] * (ii_all - m.i0) for Q in P]
            lo_f = min(float(np.min(Q[0] + x)) for Q, x in zip(P, shift))
            hi_f = max(float(np.max(Q[1] + x)) for Q, x in zip(P, shift))
        else:
            lo_f, hi_f = float(np.min(bs)) - 3 * m.dc, float(np.max(bs)) + 3 * m.dc
        more = abs(self.slope(i_nom, v_now) / 1e3) * (i_hi - i_lo + 2 * REACH) + 5.0  # (the window moves too)
        orders = self._orders(lo_f - more, hi_f + more, float(np.min(bs)) - more, float(np.max(bs)) + more)
        got = {}
        for a in range(0, len(rows), 200):  # (in parts: paths x points x modes)
            part = rows[a:a + 200]
            chosen = _select(self._paths(part, orders), P, self._firsts(part, orders, P))
            for k in range(len(part)):
                got[keys[a + k][:3]] = (chosen[:, k], keys[a + k][3], keys[a + k][4])
        out = []
        for c, (n, v, i) in enumerate(cands):
            idx = int(n - orders[0])
            if not 0 <= idx < len(orders):
                continue
            for up in (True, False):
                if avoided(v, i, up):
                    continue
                ga, npre, end = got[(c, up, True)]
                gb = got[(c, up, False)][0]
                end -= 1  # (the operating point)
                there = np.flatnonzero(ga[:, end] == idx)  # (the members that have it arrive on that mode)
                if not len(there):
                    continue
                ahead = np.array([(_held(ga[p, end:], idx) - 1) * FINE for p in there])
                behind = np.array([(_held(gb[p, end:], idx) - 1) * FINE for p in there])
                held = np.zeros(len(P))
                held[there] = np.minimum(ahead, behind)
                ahead, behind = float(np.median(ahead)), float(np.median(behind))
                share = float(np.mean(held >= want - 1e-9))
                score = (share + 0.5 * float(np.mean(np.minimum(held, 2 * want))) / (2 * want)
                         - 0.004 * abs(v - v_now) - 0.02 * abs(i - i_nom)
                         + 0.02 * min(abs(i - (i_lo if up else i_hi)), 2 * want) / want)
                ii = self._way(i, up, i_lo, i_hi, step, step_s)[0]
                path = [(float(x), float(self.ghz(orders[g], x, v))) for x, g in zip(ii[npre:end + 1],
                                                                                    ga[there[0], npre:end + 1])]
                out.append(Plan(int(n), v, i, up, i_lo if up else i_hi, ahead, behind, score, path, share,
                                float(held.min())))
        return max(out, key=lambda p: p.score) if out else None

    def _firsts(self, rows, orders, P) -> np.ndarray:
        """(rules, paths): the mode each path begins on by each rule (P, as _select has them), as if long at its
        first point (the far end of the window)."""
        m = self.m
        out = np.empty((len(P), len(rows)), dtype=int)
        for k, (i, v, _, u) in enumerate(rows):
            mv = float(self._dv(v))
            modes = m.ext_modes(i[0], mv, m.t0, orders, bool(u[0]))
            base = float(m.chip_base(i[0], mv, m.t0))
            for p, Q in enumerate(P):
                out[p, k] = _first_of(modes, base, Q, float(i[0]) - m.i0, mv - m.v0, m._ci())
        return out


def _beyond(P) -> float:
    """GHz beyond its window's edges a mode may lase by rule P, its edges softer than the old hard ones (5 GHz): where
    it loses a sixth of what one halfway between chip modes does more (_losses)."""
    return 0.4 * max(float(P[11]) - 5.0, 0.0)


def _width(P) -> float:
    """GHz: rule P's window as wide as hard edges would make it (its soft edges counted as far as _beyond)."""
    return float(P[1] - P[0]) + 2 * _beyond(P)


def _apply(m: LaserModel, P):
    """The model's rule made P (as _select has it)."""
    m.lo, m.hi, m.h_up, m.h_down, m.offset, m.pull, m.lag = (float(x) for x in P[:7])
    if not m.single:
        m.chip.spacing = float(P[7])
    if m.chip is not None:
        m.chip.p_i = float(P[8])
    m.w_v, m.w_i, m.soft = float(P[9]), float(P[10]), float(P[11])


def _first_of(modes, base: float, P, di: float, dv: float, ci0: float) -> int:
    """The mode lasing where a path begins (as if long there) by rule P (as _select has it; its chip comb's middle
    there base, as ci0 GHz/mA has it; di, dv from the model's i0, v0): the best, pulled to it (LaserModel._first)."""
    lo, hi, _, _, offset, pull, _, dc, ci, w_v, w_i, soft = (float(x) for x in P)
    shift = w_v * dv + w_i * di
    mid = base + (ci - ci0) * di + offset
    k = int(np.argmin(_losses(modes, mid, dc, lo + shift, hi + shift, soft)))
    mid += pull * ((modes[k] - mid + dc / 2) % dc - dc / 2)
    return int(np.argmin(_losses(modes, mid, dc, lo + shift, hi + shift, soft)))


def _grid(a: float, b: float, step: float) -> np.ndarray:
    """a to b (both included) in steps of at most `step`."""
    n = max(1, math.ceil(abs(b - a) / step - 1e-9))
    return a + (b - a) * np.arange(n + 1) / n


def _held(orders: np.ndarray, idx: int) -> int:
    """How many steps along a path it stays on mode idx (from the first)."""
    off = np.flatnonzero(orders != idx)
    return int(off[0]) if len(off) else len(orders)


def _middle(good: np.ndarray, dc: float) -> tuple:
    """(GHz, GHz): of phis around the circle (good: those that do as well, len(good) of them from -dc/2), the middle
    of the widest stretch, and half its width."""
    grid = len(good)
    runs, start = [], None
    for k in range(2 * grid):
        if good[k % grid] and start is None:
            start = k
        if (not good[k % grid] or k == 2 * grid - 1) and start is not None:
            runs.append((start, k - 1))
            start = None
    runs = [r for r in runs if r[0] < grid and r[1] - r[0] < grid]
    if not runs:  # (all of them, or none)
        return 0.0, (0.5 * dc if good.all() else 0.0)
    a, b = max(runs, key=lambda r: r[1] - r[0])
    return float(-dc / 2 + dc * (0.5 * (a + b) % grid) / grid), 0.5 * dc * (b - a + 1) / grid
