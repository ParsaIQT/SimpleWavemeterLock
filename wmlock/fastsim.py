"""Many model lasers at once: lasermodel.LaserModel's rule (step(): which external mode lases at each point of a path,
kept until another is better by its hysteresis, its chip lagging the current's heat and pulled by the mode lasing)
for a whole batch of lasers and paths in lockstep, in numpy.

What it is for: the learned recovery (learned.py). Its training needs millions of simulated manoeuvres on lasers
that differ from one another (each a LaserModel of its own), and at run time it looks at a few hundred manoeuvres on
the laser's own model at once; a point at a time through LaserModel.step, in Python, that is minutes, here a fraction
of a second. The same rule, to rounding (tests/test_learned.py holds it against sim.ModelLaser along random paths);
only current sweeps (LaserModel.kind 'current'), at one time and temperature (no drift terms: a path of seconds).

    b = Batch.of([model, ...])            # one row a laser (or the same model many times: Batch.of(model, n))
    b.start(i, v)                         # each as if long at (i, v): the mode it would lase on there
    b.put(order)                          # ...or on that external order (as a laser that has hopped is)
    ghz = b.step(i, v, dt)                # dt s later, at (i, v): GHz each lases at
    down, up = b.margins(i, v)            # mA each can go before its mode hops, slowly (as LaserModel.margins)

Needs numpy.
"""
from __future__ import annotations

import math

import numpy as np

from .lasermodel import SINGLE

FIELDS = ("i0", "v0", "de", "e0", "e_i", "e_ii", "e_v", "e_vv", "e_leg", "c0", "c_i", "c_ii", "c_v", "c_vv", "dc",
          "lo", "hi", "h_up", "h_down", "offset", "pull", "lag", "w_v", "w_i", "soft", "rate")


def terms(model) -> dict:
    """A LaserModel's terms by the names used here (FIELDS)."""
    e, c = model.ext, model.chip
    return {
        "i0": model.i0, "v0": model.v0, "de": e.spacing, "e0": e.p0, "e_i": e.p_i, "e_ii": e.p_ii, "e_v": e.p_v,
        "e_vv": e.p_vv, "e_leg": e.p_leg,
        "c0": c.p0 if c is not None else 0.0, "c_i": c.p_i if c is not None else 0.0,
        "c_ii": c.p_ii if c is not None else 0.0, "c_v": c.p_v if c is not None else 0.0,
        "c_vv": c.p_vv if c is not None else 0.0,
        "dc": SINGLE * e.spacing if model.single else c.spacing,
        "lo": model.lo, "hi": model.hi, "h_up": model.h_up, "h_down": model.h_down, "offset": model.offset,
        "pull": model.pull, "lag": model.lag, "w_v": model.w_v, "w_i": model.w_i, "soft": model.soft,
        "rate": model.step_x / model.step_s if model.step_x and model.step_s else 0.0,
    }


class Batch:
    """n lasers, each with its own terms (arrays of n: FIELDS) and its own state: the external order it lases on,
    how warm its chip is (mA: the current its temperature is that of), the way its current last moved, and δ (GHz
    its mode sits above its chip resonance's best place). sided: its hysteresis by the edge of its mode it is at
    (sim.ModelLaser's sided), each laser's own."""

    def __init__(self, terms_: dict, n: int, sided=False):
        self.n = n
        for k in FIELDS:
            setattr(self, k, np.broadcast_to(np.asarray(terms_[k], dtype=float), (n,)).copy())
        self.sided = np.broadcast_to(np.asarray(sided, dtype=bool), (n,)).copy()
        self.lo = np.where(np.isfinite(self.lo), self.lo, -1e9)
        self.hi = np.where(np.isfinite(self.hi), self.hi, 1e9)
        self.order = np.zeros(n, dtype=np.int64)
        self.warm = np.zeros(n)
        self.x = np.full(n, np.nan)   # the current at its last point (NaN: none yet)
        self.up = np.ones(n, dtype=bool)
        self.delta = np.zeros(n)
        self.ghz = np.zeros(n)
        self._span = None

    @classmethod
    def of(cls, models, n: int | None = None, sided=False) -> "Batch":
        """From LaserModels: a list (a row each), or one model n times."""
        if n is not None:
            return cls(terms(models), n, sided)
        rows = [terms(m) for m in models]
        return cls({k: np.array([r[k] for r in rows]) for k in FIELDS}, len(rows), sided)

    def take(self, idx) -> "Batch":
        """The lasers at idx (their terms and their states), as a batch of their own."""
        idx = np.asarray(idx)
        out = Batch.__new__(Batch)
        out.n = len(idx)
        for k in FIELDS + ("sided", "order", "warm", "x", "up", "delta", "ghz"):
            setattr(out, k, getattr(self, k)[idx].copy())
        out._span = None
        return out

    def copy(self) -> "Batch":
        return self.take(np.arange(self.n))

    def repeat(self, times: int) -> "Batch":
        """Each laser `times` times over (its copies next to one another)."""
        return self.take(np.repeat(np.arange(self.n), times))

    # ------------------------------------------------------------- the rule
    def _combs(self, i, v, warm, up):
        """(b: GHz of each laser's external order 0, chip: its chip resonance's best place before the pull, its
        window's low and high edge) at (i, v), its chip as warm as `warm`."""
        di, dv = i - self.i0, v - self.v0
        b = self.e0 + (self.e_i + self.e_ii * di) * di + (self.e_v + self.e_vv * dv) * dv
        lagging = (self.lag > 0) & (self.rate > 0)
        safe = np.where(lagging, self.rate * self.lag, 1.0)
        b = b + np.where(lagging, -0.5 * self.e_leg * (i - warm) / safe, self.e_leg * (0.5 - up.astype(float)))
        chip = (self.c0 + (self.c_i + self.c_ii * di) * di + (self.c_v + self.c_vv * dv) * dv + self.offset
                + self.c_i * (warm - i))
        shift = self.w_v * dv + self.w_i * di
        return b, chip, self.lo + shift, self.hi + shift

    def _modes(self, b, lo, hi):
        """(the first order looked at, the last, GHz of each: n x m, which of them count)."""
        first = np.floor((lo - 30.0 - b) / self.de).astype(np.int64)
        last = np.ceil((hi + 30.0 - b) / self.de).astype(np.int64)
        m = int((last - first).max()) + 1
        k = np.arange(m)
        modes = b[:, None] + (first[:, None] + k[None, :]) * self.de[:, None]
        return first, last, modes, k[None, :] <= (last - first)[:, None]

    def _loss(self, modes, mid, lo, hi, counts):
        g = np.sin(np.pi * (modes - mid[:, None]) / self.dc[:, None]) ** 2 + \
            ((np.maximum(modes - hi[:, None], 0.0) + np.maximum(lo[:, None] - modes, 0.0)) / self.soft[:, None]) ** 2
        return np.where(counts, g, np.inf)

    def _fold(self, d):
        return (d + self.dc / 2) % self.dc - self.dc / 2

    def start(self, i, v) -> np.ndarray:
        """Each as if long at (i, v): the mode it lases on there (LaserModel.step with no state)."""
        i, v = (np.broadcast_to(np.asarray(a, dtype=float), (self.n,)) for a in (i, v))
        self.warm, self.x, self.up = i.copy(), i.copy(), np.ones(self.n, dtype=bool)
        b, chip, lo, hi = self._combs(i, v, self.warm, self.up)
        first, _, modes, counts = self._modes(b, lo, hi)
        self.order = first + self._first(modes, chip, lo, hi, counts)
        return self._settle(b, chip)

    def _first(self, modes, chip, lo, hi, counts):
        rows = np.arange(self.n)
        k = np.argmin(self._loss(modes, chip, lo, hi, counts), axis=1)
        mid = chip + self.pull * self._fold(modes[rows, k] - chip)
        return np.argmin(self._loss(modes, mid, lo, hi, counts), axis=1)

    def _settle(self, b, chip):
        mode = b + self.order * self.de
        mid = chip + self.pull * self._fold(mode - chip)
        self.delta = self._fold(mode - mid)
        self.ghz = mode
        return mode

    def put(self, order, i=None, v=None, warm=None, up=None, delta=None):
        """Each on that external order (and, if given, at that current, as warm, its current last moved that way,
        its δ: the state a laser is in after a path)."""
        self.order = np.broadcast_to(np.asarray(order, dtype=np.int64), (self.n,)).copy()
        for name, val, kind in (("x", i, float), ("warm", warm if warm is not None else i, float), ("up", up, bool),
                                ("delta", delta, float)):
            if val is not None:
                setattr(self, name, np.broadcast_to(np.asarray(val, dtype=kind), (self.n,)).copy())
        if v is not None and i is not None and delta is None:
            b, chip, _, _ = self._combs(self.x, np.broadcast_to(np.asarray(v, dtype=float), (self.n,)), self.warm,
                                        self.up)
            self._settle(b, chip)

    def step(self, i, v, dt) -> np.ndarray:
        """dt s on (each laser's own, or one for all; inf: until its chip is as warm as its current), at (i, v):
        GHz each lases at then (LaserModel.step; sided: sim.ModelLaser._step)."""
        n = self.n
        i, v, dt = (np.broadcast_to(np.asarray(a, dtype=float), (n,)) for a in (i, v, dt))
        moved = (i != self.x) & ~np.isnan(self.x)
        slide = self.c_i - self.e_i
        up = np.where(self.sided, self.delta * slide < 0, np.where(moved, i > self.x, self.up))
        with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
            decay = np.where(self.lag > 0, np.exp(-dt / np.where(self.lag > 0, self.lag, 1.0)), 0.0)
        warm = np.where(dt > 0, i + (self.warm - i) * decay, np.where(self.lag > 0, self.warm, i))
        b, chip, lo, hi = self._combs(i, v, warm, up)
        first, last, modes, counts = self._modes(b, lo, hi)
        rows = np.arange(n)
        inside = (self.order >= first) & (self.order <= last)
        k = np.clip(self.order - first, 0, modes.shape[1] - 1)
        mid = chip + self.pull * self._fold(modes[rows, k] - chip)
        g = self._loss(modes, mid, lo, hi, counts)
        best = np.argmin(g, axis=1)
        hop = g[rows, k] > g[rows, best] + np.where(up, self.h_up, self.h_down)
        k = np.where(hop, best, k)
        if not inside.all():  # (off the orders looked at: afresh, the best there)
            k = np.where(inside, k, self._first(modes, chip, lo, hi, counts))
        self.order, self.warm, self.x, self.up = first + k, warm, i.copy(), up
        return self._settle(b, chip)

    def run(self, i, v, dt) -> np.ndarray:
        """A path a laser: i, each laser's currents along it (n x points); v, its piezo all the way (each laser's, or
        one for all); dt, s from each point to the next (points, or one for all). Stepped through; GHz at its end."""
        i = np.asarray(i, dtype=float)
        dt = np.broadcast_to(np.asarray(dt, dtype=float), (i.shape[-1],))
        for k in range(i.shape[-1]):
            self.step(i[..., k], v, dt[k])
        return self.ghz

    def margins(self, i, v, reach: float = 1.5, fine: float = 0.02) -> tuple:
        """(mA down, mA up) each laser's current can go from (i, v), where its state has it, before the mode it
        lases on hops: slowly (its chip as warm as its current), in steps of `fine` as far as `reach` (inf: not
        within that). LaserModel.margins, sim.ModelLaser.holds. Its state is left as it was."""
        i, v = (np.broadcast_to(np.asarray(a, dtype=float), (self.n,)) for a in (i, v))
        out = []
        for way in (-1.0, 1.0):
            b = self.copy()
            b.x, b.warm = i.copy(), i.copy()
            got = np.full(self.n, np.inf)
            on = np.ones(self.n, dtype=bool)
            for k in range(1, int(round(reach / fine)) + 1):
                b.step(i + way * k * fine, v, np.inf)
                off = on & (b.order != self.order)
                got[off] = (k - 1) * fine
                on &= ~off
                if not on.any():
                    break
            out.append(got)
        return out[0], out[1]

    def mode_ghz(self, order, i, v) -> np.ndarray:
        """GHz of that external order at (i, v), each laser at rest there (its chip as warm as its current)."""
        i, v = (np.broadcast_to(np.asarray(a, dtype=float), (self.n,)) for a in (i, v))
        b, _, _, _ = self._combs(i, v, i, self.up)
        return b + np.asarray(order) * self.de

    def piezo_for(self, i, i_ref, v_ref) -> np.ndarray:
        """V: the piezo that has the external mode on target at (i_ref, v_ref) on target at the current i too (its
        mode's frequency kept: its cavity's part of what the current does made up for), each laser at rest."""
        i, i_ref, v_ref = (np.broadcast_to(np.asarray(a, dtype=float), (self.n,)) for a in (i, i_ref, v_ref))
        di, dr, dvr = i - self.i0, i_ref - self.i0, v_ref - self.v0
        need = (self.e_i + self.e_ii * dr) * dr + (self.e_v + self.e_vv * dvr) * dvr - (self.e_i + self.e_ii * di) * di
        # (e_v + e_vv dv) dv = need: the root next to v_ref
        a, b_ = self.e_vv, self.e_v
        lin = need / np.where(b_ != 0, b_, 1.0)
        disc = b_ * b_ + 4 * a * need
        quad = np.where((np.abs(a) > 1e-12) & (disc >= 0),
                        (-b_ + np.sign(b_) * np.sqrt(np.maximum(disc, 0.0))) / np.where(np.abs(a) > 1e-12, 2 * a, 1.0),
                        lin)
        return self.v0 + quad


def manoeuvre(i_from, i_turn, i_end, rate: float, hold_turn, hold_end, tick: float = 0.02) -> tuple:
    """The currents of a fast manoeuvre a tick at a time: from i_from to its turning point at `rate` mA/s, held there
    hold_turn s, on to its end, held there hold_end s (each an array of n, or one for all): (i: n x points, dt: the
    tick, points). A path that is over sooner than the longest stays where it ended."""
    i_from, i_turn, i_end, hold_turn, hold_end = np.broadcast_arrays(
        *(np.atleast_1d(np.asarray(a, dtype=float)) for a in (i_from, i_turn, i_end, hold_turn, hold_end)))
    t1 = np.abs(i_turn - i_from) / rate
    t2 = t1 + hold_turn
    t3 = t2 + np.abs(i_end - i_turn) / rate
    t4 = t3 + hold_end
    points = max(1, int(math.ceil(float(t4.max()) / tick - 1e-9)))
    t = (np.arange(1, points + 1) * tick)[None, :]
    leg1 = i_from[:, None] + (i_turn - i_from)[:, None] * np.clip(t / np.maximum(t1, 1e-12)[:, None], 0.0, 1.0)
    leg2 = (i_end - i_turn)[:, None] * np.clip((t - t2[:, None]) / np.maximum(t3 - t2, 1e-12)[:, None], 0.0, 1.0)
    return leg1 + leg2, np.full(points, tick), t4
