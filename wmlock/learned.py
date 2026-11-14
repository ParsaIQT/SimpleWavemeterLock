"""Mode-hop recovery, learned: a policy that has seen a great many simulated hops of this laser, and says at once
which fast manoeuvre of its current brings it back onto its right mode (HopParams.method 'learned').

What is learned, and from what. A laser with a model (lasermodel.LaserModel, fitted to its map) hops when its chip's
modes have drifted to where its right mode gives way to another. The way back is a manoeuvre of its current: the
piezo to where its right mode will be on target, the current quickly to a turning point (or none), held there while
its chip warms to it, on to its final current, held again. Which turning point and which final current work depends
on the laser as it really is (its hysteresis, its pull, how far its window's edges are, how its chip lags), which its
model has only roughly. So, for one laser:

- train(model) makes thousands of lasers like the model, not it (randomized: each term of its rule spread as fits of
  one map differ, and more; its hysteresis by the way its current moved, or by the edge of its mode it is at), puts
  each on its mode at an operating point, lets its chip's modes drift one way or the other until it hops, and has an
  oracle fly every manoeuvre of a grid on each of them (fastsim.Batch: the model's rule for all at once), noting for
  each whether it ends on its right mode and with what margin; then again from where a manoeuvre that failed has
  left it. A small network (numpy) is fitted to that: from the situation (where it ran, which mode it is on now and
  how it got there) to the chance that each manoeuvre of the grid ends on its right mode, and the margin to expect.
  That is imitation of an oracle that knows each laser as it is, by a policy that sees only what the program sees.
- Policy.choose(situation, window) is one pass through that network: the manoeuvre within the current's window with
  the best prospects. A fraction of a millisecond.
- What its own recoveries then find (a manoeuvre that worked, one that did not) is kept with the laser (Memory) and
  added to the network's map from then on: the real laser's word against its model's, hop by hop. A recapture by
  hand, recorded (handrec.py), is taken the same way.

Why this, and not learning on the laser: a try on the laser costs seconds and hops are rare; the simulator makes a
million tries in a minute, and its lasers are as its map's model has them. What the model has wrong, the memory
mends, a hop at a time. modehop.Recenter.by_learning flies what the policy chooses, by the recovery's own requests
(every safety rule as ever), reads where that left it, and goes on to the methods of old if it does not get there.

    python -m wmlock.learned train blue --out blue.npz      # a policy for the trials' 493 nm laser (or: red, a seed)
    python -m wmlock.learned train --map MAP.sqlite --out laser.npz   # for a laser's own model, kept in its map

Needs numpy.
"""
from __future__ import annotations

import hashlib
import json
import math
import time

import numpy as np

from . import fastsim
from .fastsim import Batch

VERSION = 2
TURNS = np.concatenate([[np.nan], np.arange(-2.5, 2.5 + 1e-9, 0.5)])  # mA from where it ran (NaN: no turn)
WAYS = (1.0, -1.0)            # the way its current is ramped to catch its right mode: up, down
DEPTHS = (0.25, 0.4, 0.55)    # mA past where it came onto its right mode: where it is left
RATE = 5.0        # mA/s: how fast its current goes where nothing is looked for (modehop.Runner.RATE's most)
PER_READ = 0.2    # mA a reading: how fast it goes where its right mode is looked for (catching)
CATCH = (0.1, 2.5)  # mA/s: ...within these
HOLD = (2.5, 0.1, 1.0)  # held at its turn and at its end: this many of its chip's lags, within these s
TICK = 0.025      # s a point, simulated
MARGIN = 0.6      # mA: a margin counts up to this (more is no better)
COME_FROM = 2.0   # mA above (or below): where a laser is brought onto its mode from, as by hand
SLOWEST = 10.0    # s: a manoeuvre's time as the network has it is a share of this


def actions() -> tuple:
    """The manoeuvres: (turning point, mA from where it ran, NaN: none; the way it then catches, +1 up, -1 down; how
    deep into its right mode it is left, mA): three arrays."""
    t, w, d = np.meshgrid(TURNS, np.array(WAYS), np.array(DEPTHS), indexing="ij")
    return t.ravel(), w.ravel(), d.ravel()


def hold_s(model) -> float:
    """s a manoeuvre is held at its turn and at its end: its chip's lag, HOLD times."""
    return float(min(max(HOLD[0] * (model.lag or 0.0), HOLD[1]), HOLD[2]))


def catch_rate(gap: float) -> float:
    """mA/s its current is ramped at while its right mode is looked for: PER_READ a reading (gap: s between the
    wavemeter's readings of it)."""
    return float(min(max(PER_READ / max(gap, 1e-3), CATCH[0]), CATCH[1]))


def entered(i_seen, way, rate, lag, gap):
    """Where it came onto its right mode (mA), by the first reading that had it there (i_seen: the set current
    then): its chip is `lag` s behind its current, and the reading came half a `gap` after, on average."""
    return i_seen - way * rate * (lag + 0.5 * gap)


# ------------------------------------------------------------- simulated lasers
def randomized(model, n: int, rng, spread=(0.5, 2.0), sided: float = 0.5, alt: float = 0.3) -> Batch:
    """n lasers like the model, not it: its rule's terms spread as fits of one map differ (trials.unlike: its
    hysteresis by a factor e^0.4 either way, its pull by 0.05, its lag by e^0.3, its window's edges by 3 % of its
    width, how it moves by 15 %), times `spread` (each laser's own, from that range), and besides how soft its
    window's edges are (e^0.2) and how its combs tune (1.5 %: its map has them to a percent or two); its chip's modes
    anywhere. sided: this part of them with their hysteresis by the edge of their mode they are at (sim.ModelLaser);
    alt: this part made from the rule its fit held the other way (LaserModel.alt), if it kept one."""
    t = {k: np.full(n, float(v)) for k, v in fastsim.terms(model).items()}
    if model.alt is not None and alt > 0:
        other = fastsim.terms(model.with_rule(model.alt))
        pick = rng.random(n) < alt
        for k in fastsim.FIELDS:
            t[k] = np.where(pick, other[k], t[k])
    k = rng.uniform(spread[0], spread[1], size=n)
    t["h_up"] *= np.exp(rng.normal(0, 0.4, n) * k)
    t["h_down"] *= np.exp(rng.normal(0, 0.4, n) * k)
    t["pull"] = np.clip(t["pull"] + rng.normal(0, 0.05, n) * k, -0.4, 0.8)
    t["lag"] *= np.exp(rng.normal(0, 0.3, n) * k)
    w = t["hi"] - t["lo"]
    t["lo"] += rng.normal(0, 0.03, n) * k * w
    t["hi"] += rng.normal(0, 0.03, n) * k * w
    t["w_v"] *= 1 + rng.normal(0, 1, n) * np.minimum(0.15 * k, 0.4)
    t["w_i"] *= 1 + rng.normal(0, 1, n) * np.minimum(0.15 * k, 0.4)
    t["soft"] *= np.exp(rng.normal(0, 0.2, n) * k)
    for name in ("c_i", "e_i", "e_v"):
        t[name] *= 1 + rng.normal(0, 0.015, n) * k
    t["offset"] += rng.uniform(-0.5, 0.5, n) * t["dc"]
    return Batch(t, n, sided=rng.random(n) < sided)


def situations(b: Batch, i_ref, v_ref, rng, way=None) -> tuple:
    """Each laser of the batch put on its mode at (i_ref, v_ref) (coming down from COME_FROM mA above, as by hand; a
    part of them coming up instead), its chip's modes then drifting one way or the other (way: +1, -1, each laser's;
    None: either) until it hops: where along the drift that is, found by a coarse look over one chip mode and by
    halving. Returns (the right order, the way of the drift, which of them did hop); the batch is left as each is
    just after its hop, still at (i_ref, v_ref)."""
    n = b.n
    above = np.where(rng.random(n) < 0.8, 1.0, -1.0) * COME_FROM
    b.start(i_ref + above, v_ref)
    for k in range(1, 21):
        b.step(i_ref + above * (1 - k / 20), v_ref, 0.15)
    b.step(i_ref, v_ref, np.inf)
    right = b.order.copy()
    way = np.where(rng.random(n) < 0.5, 1.0, -1.0) if way is None else np.broadcast_to(np.asarray(way, float), (n,))
    off0 = b.offset.copy()

    def hops(frac):
        t = b.copy()
        t.offset = off0 + way * frac * b.dc
        t.put(right, i=i_ref, v=v_ref)  # (where its mode sits on its chip's resonance there: the edge it is at)
        t.step(i_ref, v_ref, np.inf)
        return t.order != right

    looks = np.linspace(0.0, 1.0, 129)[1:]
    first = np.full(n, np.nan)
    for f in looks:
        h = hops(np.full(n, f)) & np.isnan(first)
        first[h] = f
    keep = ~np.isnan(first)
    lo, hi = np.where(keep, first - looks[0], 0.0), np.where(keep, first, 0.0)
    for _ in range(12):
        mid = 0.5 * (lo + hi)
        h = hops(mid)
        hi, lo = np.where(h, mid, hi), np.where(h, lo, mid)
    b.offset = off0 + way * (hi + rng.uniform(0, 0.0004, n)) * b.dc
    b.put(right, i=i_ref, v=v_ref)
    b.step(i_ref, v_ref, np.inf)
    return right, way, keep & (b.order != right)


def fly(b: Batch, right, i_ref, v_ref, turn, way, depth, lo, hi, hold, gap, lag, rng) -> tuple:
    """Each laser of the batch flown through its manoeuvre, from where it is, as modehop.Recenter.by_learning flies
    it: its current to its turning point at RATE and held there `hold` s (turn: mA from i_ref; NaN: none); then
    ramped `way` (+1 up, -1 down) at the catching rate towards the end of its window (lo, hi: mA from i_ref), a
    reading every `gap` s; at the first reading on its right order, to where it came onto it (entered(): by `lag`,
    its chip's as the program has it) and `depth` mA on, at RATE; held `hold` s. Not seen on it by its window's
    end: held there. The piezo goes along all the way, to where its right mode stays on target (the mode on target
    at i_ref, v_ref). Returns (where its current ends, mA from i_ref; the s it took; whether it was seen on its
    right mode on the way)."""
    n = b.n
    f = lambda a: np.broadcast_to(np.asarray(a, dtype=float), (n,)).copy()  # noqa: E731
    i_ref, v_ref, turn, way, depth, lo, hi, hold, gap, lag = (f(a) for a in (i_ref, v_ref, turn, way, depth, lo, hi,
                                                                          hold, gap, lag))
    rate = np.clip(PER_READ / np.maximum(gap, 1e-3), CATCH[0], CATCH[1])
    none = np.isnan(turn)
    i = b.x.copy()
    at_turn = i_ref + np.clip(np.where(none, 0.0, turn), lo, hi)
    limit = i_ref + np.where(way > 0, hi, lo)
    goal = limit.copy()
    stage = np.where(none, 2, 0)   # 0: to its turn, 1: held there, 2: catching, 3: to where it is left, 4: held, 5: over
    clock, t, seen = np.zeros(n), np.zeros(n), np.zeros(n, dtype=bool)
    reading = rng.uniform(0, 1, n) * gap
    for _ in range(int(80.0 / TICK)):
        live = stage < 5
        if not live.any():
            break
        to = np.where(stage == 0, at_turn, np.where(stage == 2, limit, np.where(stage == 3, goal, i)))
        pace = np.where(stage == 2, rate, RATE) * TICK
        i = i + np.clip(to - i, -pace, pace)
        b.step(np.where(live, i, b.x), b.piezo_for(i, i_ref, v_ref), np.where(live, TICK, 0.0))
        t, clock = np.where(live, t + TICK, t), clock + TICK
        due = live & (t >= reading)
        reading = np.where(due, reading + gap, reading)
        got = due & (stage == 2) & (b.order == right)
        there = np.abs(to - i) < 1e-9
        nxt = stage.copy()
        nxt = np.where((stage == 0) & there, 1, nxt)
        nxt = np.where((stage == 1) & (clock >= hold), 2, nxt)
        nxt = np.where((stage == 2) & there, 4, nxt)
        nxt = np.where(got, 3, nxt)
        nxt = np.where((stage == 3) & there, 4, nxt)
        nxt = np.where((stage == 4) & (clock >= hold), 5, nxt)
        goal = np.where(got, np.clip(entered(i, way, rate, lag, gap) + way * depth, i_ref + lo, i_ref + hi), goal)
        seen |= got
        clock = np.where(nxt != stage, 0.0, clock)
        stage = nxt
    return i - i_ref, t, seen


def oracle(b: Batch, right, i_ref, v_ref, lo, hi, hold, gap, lag, rng, here=0.0, chunk: int = 30000) -> tuple:
    """What each manoeuvre (actions()) comes to for each laser of the batch, from where it is: (the least margin it
    ends with on its right mode, mA, MARGIN at most; -1: it does not end on it; NaN: its turning point is outside
    its window, not flown; the s it takes), lasers x manoeuvres each. lo, hi: each laser's window (mA from i_ref);
    gap: s between its readings; lag: its chip's as the program has it (its model's)."""
    turns, ways, depths = actions()
    n_act = len(turns)
    y, took = np.full((b.n, n_act), np.nan), np.full((b.n, n_act), np.nan)
    f = lambda a: np.broadcast_to(np.asarray(a, dtype=float), (b.n,))  # noqa: E731
    lo, hi, hold, gap, lag, here = f(lo), f(hi), f(hold), f(gap), f(lag), f(here)
    step = max(1, chunk // n_act)
    for a in range(0, b.n, step):
        idx = np.arange(a, min(a + step, b.n))
        who, which = np.repeat(idx, n_act), np.tile(np.arange(n_act), len(idx))
        fits = np.isnan(turns[which]) | ((turns[which] >= lo[who] - 1e-9) & (turns[which] <= hi[who] + 1e-9))
        who, which = who[fits], which[fits]
        rows = b.take(who)
        _, t, _ = fly(rows, right[who], i_ref[who], v_ref[who], turns[which], ways[which], depths[which], lo[who],
                      hi[who], hold[who], gap[who], lag[who], rng)
        ok = rows.order == right[who]
        res = np.full(len(who), -1.0)
        if ok.any():
            sub = rows.take(np.flatnonzero(ok))
            down, up = sub.margins(sub.x, sub.piezo_for(sub.x, i_ref[who][ok], v_ref[who][ok]), reach=MARGIN + 0.02)
            res[ok] = np.minimum(np.minimum(down, up), MARGIN)
        y[who, which], took[who, which] = res, t
    return y, took


# ---------------------------------------------------------------- situations
N_FEATURES = 15


def _jump(j, de, dc) -> np.ndarray:
    """A jump of j external orders, as the network takes it: how many chip modes it is (halved), how far from a
    whole number of them (which external order next to it), and the orders themselves (a laser of one filter hops by
    one or two)."""
    j = np.asarray(j, dtype=float)
    a = j * de / dc
    return np.stack([np.clip(a, -4, 4) / 2, np.clip(5 * (a - np.round(a)), -2.5, 2.5), np.clip(j / 8, -1.5, 1.5)], -1)


def features(model, i_ref, v_ref, jump, lo, hi, here=0.0, came=0.0, turned=np.nan, jump0=None) -> np.ndarray:
    """A situation as the network takes it (one, or arrays of them: n x N_FEATURES): where the laser ran on target
    (i_ref mA, v_ref V as its model has the piezo: from the model's own i0, v0); its current's window (lo, hi: mA
    from i_ref); the mode it is on now (jump: external orders from its right one); and, after a manoeuvre that did
    not bring it back: where its current is now (here: mA from i_ref), the way that manoeuvre was catching (came: +1,
    -1; 0: none yet, it hopped where it stood), its turning point (turned: mA from i_ref, NaN: none), and the hop
    it all began with (jump0)."""
    i_ref, v_ref, jump, lo, hi, here, came, turned = np.broadcast_arrays(
        *(np.atleast_1d(np.asarray(a, dtype=float)) for a in (i_ref, v_ref, jump, lo, hi, here, came, turned)))
    jump0 = jump if jump0 is None else np.broadcast_to(np.asarray(jump0, dtype=float), jump.shape)
    de, dc = model.ext.spacing, model.dc
    later = (came != 0).astype(float)
    none = np.isnan(turned)
    return np.concatenate([
        np.stack([(i_ref - model.i0) / 2, (v_ref - model.v0) / 5, lo / 2.5, hi / 2.5, later, here / 2.5, came,
                  later * (~none), np.where(none, 0.0, turned) / 2.5], -1),
        _jump(jump, de, dc), _jump(jump0, de, dc)], -1)


# ------------------------------------------------------------------ network
class Net:
    """A small network: the situation in, for every manoeuvre three numbers out (each through a sigmoid): the chance
    that it ends on its right mode, the margin to expect there as a share of MARGIN (0 where it does not end on
    it), and the time it takes as a share of SLOWEST. Two hidden layers; numpy; fitted by Adam to the oracle's word
    (fit)."""

    HEADS = 3

    def __init__(self, n_in: int, n_out: int, hidden: int = 96, seed: int = 0):
        rng = np.random.default_rng(seed)
        sizes = [n_in, hidden, hidden, self.HEADS * n_out]
        self.w = [rng.normal(0, math.sqrt(2.0 / a), (a, b)).astype(np.float32) for a, b in zip(sizes, sizes[1:])]
        self.b = [np.zeros(b, dtype=np.float32) for b in sizes[1:]]
        self.n_out = n_out

    def forward(self, x, keep=False):
        hs, h = [np.asarray(x, dtype=np.float32)], np.asarray(x, dtype=np.float32)
        for k, (w, b) in enumerate(zip(self.w, self.b)):
            h = h @ w + b
            if k < len(self.w) - 1:
                h = np.maximum(h, 0.0)
            hs.append(h)
        p = 1.0 / (1.0 + np.exp(-np.clip(hs[-1], -30, 30)))
        return (p, hs) if keep else p

    def maps(self, x) -> tuple:
        """(the chance each manoeuvre ends on its right mode, the margin to expect of it as a share of MARGIN, the s
        it takes): for one situation (n_out each) or many (n x n_out)."""
        p = self.forward(np.atleast_2d(x))
        n = self.n_out
        out = p[:, :n], p[:, n:2 * n], p[:, 2 * n:] * SLOWEST
        return tuple(o[0] for o in out) if np.ndim(x) == 1 else out

    def fit(self, x, y, took, epochs: int = 60, batch: int = 64, lr: float = 2e-3, seed: int = 0, say=None) -> float:
        """Fitted to the oracle's word (y: n x n_out, the least margin, mA; -1: not on its right mode; NaN: not
        flown, not counted; took: the s each took) by Adam, cross-entropy on each output. Returns the last epoch's
        loss."""
        rng = np.random.default_rng(seed)
        x = np.asarray(x, dtype=np.float32)
        seen = ~np.isnan(y)
        t = np.concatenate([np.where(seen, y >= 0, 0.0), np.where(seen, np.clip(y, 0, MARGIN) / MARGIN, 0.0),
                            np.where(seen, np.clip(took / SLOWEST, 0, 1), 0.0)], 1).astype(np.float32)
        mask = np.concatenate([seen, seen, seen], 1).astype(np.float32)
        m = [np.zeros_like(a) for a in self.w + self.b]
        v = [np.zeros_like(a) for a in self.w + self.b]
        step, loss, n = 0, math.nan, len(x)
        for epoch in range(epochs):
            order, total = rng.permutation(n), 0.0
            rate = lr * 0.5 * (1 + math.cos(math.pi * epoch / epochs))
            for a in range(0, n, batch):
                idx = order[a:a + batch]
                p, hs = self.forward(x[idx], keep=True)
                k = mask[idx]
                count = max(float(k.sum()), 1.0)
                tt = t[idx]
                total += float(-(k * (tt * np.log(p + 1e-7) + (1 - tt) * np.log(1 - p + 1e-7))).sum())
                d = (p - tt) * k / count
                grads_w, grads_b = [], []
                for layer in range(len(self.w) - 1, -1, -1):
                    grads_w.append(hs[layer].T @ d)
                    grads_b.append(d.sum(0))
                    if layer:
                        d = (d @ self.w[layer].T) * (hs[layer] > 0)
                grads = grads_w[::-1] + grads_b[::-1]
                step += 1
                for j, (par, g) in enumerate(zip(self.w + self.b, grads)):
                    m[j] = 0.9 * m[j] + 0.1 * g
                    v[j] = 0.999 * v[j] + 0.001 * g * g
                    par -= rate * (m[j] / (1 - 0.9 ** step)) / (np.sqrt(v[j] / (1 - 0.999 ** step)) + 1e-8)
            loss = total / max(float(mask.sum()), 1.0)
            if say is not None and (epoch % 20 == 19 or epoch == epochs - 1):
                say(f"epoch {epoch + 1}: loss {loss:.4f}")
        return loss


# ------------------------------------------------------------------- policy
W_ON, W_TIME = 0.5, 0.03  # a manoeuvre's worth: the margin to expect (as a share), + W_ON its chance, - W_TIME a s


class Choice:
    """A manoeuvre chosen: its turning point (mA from where it ran; None: none), the way it catches (+1 up, -1
    down), how deep into its right mode it is left (mA); the chance it ends on its right mode, the margin to expect
    there if it does (mA), the s it should take; k: which of actions() it is."""

    def __init__(self, turn, way, depth, on, margin, took, k):
        self.turn, self.way, self.depth, self.on, self.margin, self.took, self.k = turn, way, depth, on, margin, took, k

    def __repr__(self):
        return (f"Choice(turn={'none' if self.turn is None else format(self.turn, '+.1f')}, "
                f"{'up' if self.way > 0 else 'down'}, depth={self.depth:.2f}, on={self.on:.2f}, "
                f"margin={self.margin:.2f}, took={self.took:.1f})")


class Policy:
    """A laser's policy: its network, the s its manoeuvres are held (hold), its chip's lag as its model has it (lag),
    and what it was made from (meta: the model's key, how many lasers, how it did on those kept apart)."""

    def __init__(self, net: Net, hold: float, lag: float, meta: dict | None = None):
        self.net, self.hold, self.lag, self.meta = net, float(hold), float(lag), dict(meta or {})
        self._turns, self._ways, self._depths = actions()

    def choose(self, x, lo: float, hi: float, bonus=None, least: float = 0.0, skip=()):
        """The manoeuvre with the best prospects in the situation x (features(): of the same window lo .. hi, mA
        from where it ran), its turning point within the window. bonus: added to each manoeuvre's worth (its
        laser's own memory: Memory.bonus); skip: manoeuvres (their k) not to be chosen (tried already, in vain).
        None: none within the window has a chance of `least` at least."""
        on, share, took = self.net.maps(x)
        turns = self._turns
        none = np.isnan(turns)
        ok = none | ((turns >= lo - 1e-9) & (turns <= hi + 1e-9))
        for k in skip:
            ok[k] = False
        worth = share + W_ON * on - W_TIME * took + (bonus if bonus is not None else 0.0)
        worth = np.where(ok & (on >= least), worth, -np.inf)
        k = int(np.argmax(worth))
        if not np.isfinite(worth[k]):
            return None
        return Choice(None if none[k] else float(turns[k]), float(self._ways[k]), float(self._depths[k]),
                      float(on[k]), float(min(share[k] / max(on[k], 1e-6), 1.0) * MARGIN), float(took[k]), k)

    # ---------------------------------------------------------------- files
    def save(self, path: str):
        arrays = {f"w{k}": w for k, w in enumerate(self.net.w)}
        arrays.update({f"b{k}": b for k, b in enumerate(self.net.b)})
        np.savez_compressed(path, hold=self.hold, lag=self.lag, meta=json.dumps(self.meta), version=VERSION, **arrays)

    @classmethod
    def load(cls, path: str) -> "Policy":
        with np.load(path, allow_pickle=False) as d:
            if int(d["version"]) != VERSION:
                raise ValueError(f"a policy of version {int(d['version'])}, not {VERSION}")
            n = len([k for k in d.files if k.startswith("w")])
            net = Net.__new__(Net)
            net.w = [d[f"w{k}"] for k in range(n)]
            net.b = [d[f"b{k}"] for k in range(n)]
            net.n_out = net.w[-1].shape[1] // Net.HEADS
            return cls(net, float(d["hold"]), float(d["lag"]), json.loads(str(d["meta"])))


def model_key(model) -> str:
    """What tells one laser's model from another (and from itself fitted again): a hash of its terms."""
    d = model.to_dict()
    d.pop("notes", None)
    return hashlib.sha1(json.dumps(d, sort_keys=True, default=str).encode()).hexdigest()[:16]


# ----------------------------------------------------------------- training
GAPS = (0.08, 0.3)  # s between readings: each simulated laser's own, from this range


def _places(model, n: int, rng) -> tuple:
    """Where n lasers of the model run (currents and piezo voltages about its own i0, v0, as far as its map went),
    their currents' windows (lo, hi: mA from there; three mA wide as a rule, put anywhere about it), and the s
    between their readings."""
    i_lo, i_hi = model.i_span or (model.i0 - 3, model.i0 + 3)
    v_lo, v_hi = model.v_span or (model.v0 - 10, model.v0 + 10)
    i = np.clip(model.i0 + rng.uniform(-1.5, 1.5, n), i_lo + 2.5, i_hi - 2.5)
    v = np.clip(model.v0 + rng.uniform(-4, 4, n), v_lo + 3, v_hi - 3)
    width = np.where(rng.random(n) < 0.6, 3.0, rng.uniform(2.0, 4.5, n))
    lo = -np.minimum(np.where(rng.random(n) < 0.4, 1.0, rng.uniform(0.2, 2.5, n)), width - 0.2)
    lo = np.maximum(lo, -2.5)
    return i, v, lo, np.clip(lo + width, 0.2, 2.5), rng.uniform(GAPS[0], GAPS[1], n)


def hopped(model, n: int, rng, **kw) -> dict:
    """n lasers like the model (randomized), each on its mode somewhere and hopped (situations): those that did, as
    {b: the batch, right, i_ref, v_ref, lo, hi, gap, jump: the orders it hopped by, way: its drift's}."""
    b = randomized(model, n, rng, **kw)
    i_ref, v_ref, lo, hi, gap = _places(model, n, rng)
    right, way, keep = situations(b, i_ref, v_ref, rng)
    idx = np.flatnonzero(keep)
    b = b.take(idx)
    return {"b": b, "right": right[idx], "i_ref": i_ref[idx], "v_ref": v_ref[idx], "lo": lo[idx], "hi": hi[idx],
            "gap": gap[idx], "jump": (b.order - right[idx]).astype(float), "way": way[idx]}


def examples(model, n: int, rng, policy: Policy | None = None, **kw) -> tuple:
    """n lasers like the model, each hopped: (x: their situations as the network takes them, y: the oracle's word on
    each manoeuvre, took: the s each took; lasers x manoeuvres). With a policy: each is first flown through what
    that policy would choose (now and then another manoeuvre, at random), and those it leaves off their right mode
    are the situations (the oracle's word from there): what a second try must know."""
    hold, lag = hold_s(model), model.lag or 0.0
    s = hopped(model, n, rng, **kw)
    b, right, i_ref, v_ref, lo, hi, gap, jump0 = (s[k] for k in ("b", "right", "i_ref", "v_ref", "lo", "hi", "gap",
                                                                "jump"))
    x1 = features(model, i_ref, v_ref, jump0, lo, hi)
    if policy is None:
        return (x1,) + oracle(b, right, i_ref, v_ref, lo, hi, hold, gap, lag, rng)
    turns, ways, depths = actions()
    pick = np.zeros(b.n, dtype=int)
    for k in range(b.n):
        c = policy.choose(x1[k], lo[k], hi[k]) if rng.random() > 0.25 else None
        if c is None:
            ok = np.flatnonzero(np.isnan(turns) | ((turns >= lo[k]) & (turns <= hi[k])))
            pick[k] = int(rng.choice(ok))
        else:
            pick[k] = c.k
    end, _, _ = fly(b, right, i_ref, v_ref, turns[pick], ways[pick], depths[pick], lo, hi, hold, gap, lag, rng)
    b.step(b.x, b.piezo_for(b.x, i_ref, v_ref), 0.3)
    off = np.flatnonzero(b.order != right)
    b2 = b.take(off)
    x2 = features(model, i_ref[off], v_ref[off], (b2.order - right[off]).astype(float), lo[off], hi[off],
                  here=end[off], came=ways[pick][off], turned=turns[pick][off], jump0=jump0[off])
    return (x2,) + oracle(b2, right[off], i_ref[off], v_ref[off], lo[off], hi[off], hold, gap[off], lag, rng)


def train(model, n: int = 1000, again: int = 2000, seed: int = 0, say=None, epochs: int = 80) -> Policy:
    """A policy for the laser the model describes: n simulated lasers like it hopped and looked at by the oracle; a
    first network fitted to them; `again` more flown through what that network would choose, those it does not
    bring back looked at by the oracle from where they are; the network fitted to both. Kept apart, and tried at
    the end: 400 hops more (meta: of those whose chip's modes drifted up, 'up', and down, 'down': the share its
    first manoeuvre brings back with a third of MARGIN at least, by one of its first two, and the s those took)."""
    rng = np.random.default_rng(seed)
    t0 = time.time()
    say = say or (lambda text: None)
    hold, lag = hold_s(model), model.lag or 0.0
    x1, y1, t1 = examples(model, n, rng)
    say(f"{len(x1)} hops looked at by the oracle ({time.time() - t0:.0f} s)")
    first = Net(N_FEATURES, y1.shape[1], seed=seed)
    first.fit(x1, y1, t1, epochs=max(epochs // 2, 10), seed=seed)
    x2, y2, t2 = examples(model, again, rng, policy=Policy(first, hold, lag))
    say(f"{len(x2)} of {again} more not brought back by a first manoeuvre: looked at from there "
        f"({time.time() - t0:.0f} s)")
    net = Net(N_FEATURES, y1.shape[1], seed=seed + 1)
    loss = net.fit(np.concatenate([x1, x2]), np.concatenate([y1, y2]), np.concatenate([t1, t2]), epochs=epochs,
                   seed=seed, say=say)
    policy = Policy(net, hold, lag, {"key": model_key(model), "n": int(len(x1)), "again": int(len(x2)),
                                     "loss": round(float(loss), 4), "seed": seed, "version": VERSION})
    policy.meta["trial"] = trial(model, policy, 400, np.random.default_rng(seed + 977))
    policy.meta["took_s"] = round(time.time() - t0, 1)
    say("of 400 hops kept apart: " + "; ".join(
        f"drifting {name}: {r['first']:.0%} back by its first manoeuvre, {r['second']:.0%} by one of two, in "
        f"{r['s']:.1f} s" for name, r in policy.meta["trial"].items()) + f" ({policy.meta['took_s']:.0f} s in all)")
    return policy


def trial(model, policy: Policy, n: int, rng, window=None, gap: float | None = None, tries: int = 2, **kw) -> dict:
    """The policy tried on n simulated hops of lasers like the model (randomized, as it was trained: others), by the
    way their chip's modes drifted: {'up': .., 'down': ..}, each {n, first: the share brought back onto their right
    mode with a third of MARGIN at least by the first manoeuvre, second: by one of the first `tries`, on: on their
    right mode at all after those, s: the s the manoeuvres took (the median of those brought back), can: the share
    some manoeuvre of the first try could have brought back, as the oracle has it}. window: (lo, hi) mA about where
    each ran, for all (None: as trained, each its own); gap: s between readings, likewise."""
    hold, lag = policy.hold, policy.lag
    s = hopped(model, n, rng, **kw)
    b, right, i_ref, v_ref, lo, hi, gaps, jump0, drift = (s[k] for k in ("b", "right", "i_ref", "v_ref", "lo", "hi",
                                                                        "gap", "jump", "way"))
    n = b.n
    if window is not None:
        lo, hi = np.full(n, float(window[0])), np.full(n, float(window[1]))
    if gap is not None:
        gaps = np.full(n, float(gap))
    y0, _ = oracle(b, right, i_ref, v_ref, lo, hi, hold, gaps, lag, rng)
    can = np.nan_to_num(y0, nan=-1.0).max(1) >= MARGIN / 3
    turns, ways, depths = actions()
    x = features(model, i_ref, v_ref, jump0, lo, hi)
    done, good, took, first = (np.zeros(n, dtype=bool), np.zeros(n, dtype=bool), np.zeros(n),
                               np.zeros(n, dtype=bool))
    tried = [[] for _ in range(n)]
    for attempt in range(tries):
        pick, go = np.zeros(n, dtype=int), np.zeros(n, dtype=bool)
        for k in np.flatnonzero(~done):
            c = policy.choose(x[k], lo[k], hi[k], skip=tried[k])
            if c is not None:
                pick[k], go[k] = c.k, True
                tried[k].append(c.k)
        if not go.any():
            break
        w = np.flatnonzero(go)
        sub = b.take(w)
        end, t, _ = fly(sub, right[w], i_ref[w], v_ref[w], turns[pick[w]], ways[pick[w]], depths[pick[w]], lo[w],
                        hi[w], hold, gaps[w], lag, rng)
        v_end = sub.piezo_for(sub.x, i_ref[w], v_ref[w])
        sub.step(sub.x, v_end, 0.3)
        on = sub.order == right[w]
        down, up = sub.margins(sub.x, v_end, reach=MARGIN)
        ok = on & (np.minimum(down, up) >= MARGIN / 3)
        for name in ("order", "warm", "x", "up", "delta", "ghz"):
            getattr(b, name)[w] = getattr(sub, name)
        done[w], good[w] = on, ok
        took[w] += t + 0.45
        if attempt == 0:
            first[w] = ok
        x[w] = features(model, i_ref[w], v_ref[w], (sub.order - right[w]).astype(float), lo[w], hi[w], here=end,
                        came=ways[pick[w]], turned=turns[pick[w]], jump0=jump0[w])
    out = {}
    for name, sel in (("up", drift > 0), ("down", drift < 0)):
        if sel.any():
            out[name] = {"n": int(sel.sum()), "first": round(float(first[sel].mean()), 3),
                         "second": round(float(good[sel].mean()), 3), "on": round(float(done[sel].mean()), 3),
                         "s": round(float(np.median(took[sel & good])) if (sel & good).any() else math.nan, 2),
                         "can": round(float(can[sel].mean()), 3)}
    return out


# -------------------------------------------------------------------- memory
class Memory:
    """What its policy's manoeuvres came to on the laser itself: the situation (features()), the manoeuvre (which of
    actions()), and whether it ended on its right mode; the last KEEP of them, kept with its channel. bonus(x) is
    what they add to each manoeuvre's worth in the situation x: a manoeuvre that worked in a like situation (the
    same kind of hop, about the same place: their features WIDTH apart count a little, the same fully) is worth
    more, one that did not, less; so the laser's own word mends its model's, a hop at a time, and a manoeuvre that
    failed is not the first choice the next time. A recapture by hand, recorded and read as a manoeuvre
    (handrec.py), is added the same way."""

    KEEP = 120
    GAIN = 0.6    # what one try in the very same situation adds to (or takes from) its manoeuvre's worth, at most
    WIDTH = 0.3

    def __init__(self, items=None):
        self.items = [(list(map(float, x)), int(k), bool(ok)) for x, k, ok in (items or [])][-self.KEEP:]

    def add(self, x, k: int, ok: bool):
        self.items = (self.items + [([round(float(v), 4) for v in x], int(k), bool(ok))])[-self.KEEP:]

    def bonus(self, x) -> np.ndarray:
        out = np.zeros(len(actions()[0]))
        if not self.items:
            return out
        xs = np.array([it[0] for it in self.items])
        w = np.exp(-0.5 * ((xs - np.asarray(x, dtype=float)[None, :]) ** 2).sum(1) / self.WIDTH ** 2)
        for (_, k, ok), wk in zip(self.items, w):
            out[k] += wk if ok else -wk
        return self.GAIN * np.tanh(out)

    def to_list(self) -> list:
        return [[x, k, ok] for x, k, ok in self.items]

    @classmethod
    def from_list(cls, items) -> "Memory":
        try:
            return cls(items)
        except (TypeError, ValueError):
            return cls()


# ---------------------------------------------------------------------- files
def policy_for(model, directory: str, make: bool = True, say=None, **kw):
    """The policy learned for the model, kept in `directory` under the model's key (trained now if it is not there
    and `make`; None: not there, or of another version)."""
    import os

    path = os.path.join(directory, f"{model_key(model)}.npz")
    if os.path.exists(path):
        try:
            return Policy.load(path)
        except (OSError, ValueError, KeyError):
            pass
    if not make:
        return None
    policy = train(model, say=say, **kw)
    os.makedirs(directory, exist_ok=True)
    policy.save(path + ".tmp.npz")
    os.replace(path + ".tmp.npz", path)
    return policy


def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(prog="python -m wmlock.learned", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    tr = sub.add_parser("train", help="learn a policy for a laser's model")
    tr.add_argument("laser", nargs="?", help="blue, red (the trials' 493 and 650 nm lasers) or a seed (a random one)")
    tr.add_argument("--map", help="a map's file (lasermap): the model kept in it")
    tr.add_argument("--out", required=True, help="where to keep the policy (.npz)")
    tr.add_argument("--n", type=int, default=1000, help="simulated hops looked at by the oracle (1000)")
    tr.add_argument("--again", type=int, default=2000, help="more, flown by a first policy: those it fails (2000)")
    tr.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    if args.map:
        from . import lasermap, lasermodel

        with lasermap.LaserMap(args.map) as m:
            model = lasermodel.stored(m)
        if model is None:
            ap.error(f"{args.map} keeps no model (fit one: python -m wmlock.lasermap FILE --model)")
    else:
        from . import trials

        model = ({"blue": trials.lab493, "red": trials.lab650}[args.laser]() if args.laser in ("blue", "red")
                 else trials.random_model(int(args.laser)))
    policy = train(model, n=args.n, again=args.again, seed=args.seed, say=print)
    policy.save(args.out)
    print(f"kept in {args.out}: {json.dumps(policy.meta)}")


if __name__ == "__main__":
    main()
