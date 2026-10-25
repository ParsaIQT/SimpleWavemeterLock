"""The laser model's inner loops (lasermodel._select and _likelihood) compiled by numba, if it is installed: a point at
a time along each sweep, for each candidate rule, as lasermodel's numpy code has them, but scanning only the modes
that could be chosen: outward from the one nearest the window's middle, each way until a mode's loss beyond the
window's edge alone is more than the best so far (from there on it only grows: none further can be chosen). The same
modes chosen, ten times as fast and more (no arrays of every mode at every point to fill, and most of the modes left
alone). Each runs on one thread, letting go of the GIL: lasermodel shares its candidates out among several (_shared).
Compiled the first time it is used, and kept (numba's cache, beside this file). Without numba (pip install numba),
or with WMLOCK_NO_NUMBA set, lasermodel works as before, in numpy.
"""
import math
import os

import numpy as np

try:
    import numba
except ImportError:  # (as before, in numpy)
    numba = None
if os.environ.get("WMLOCK_NO_NUMBA"):  # (numpy, though numba is there: to compare, or should it ever misbehave)
    numba = None

AVAILABLE = numba is not None


def _jit(fn):
    return numba.njit(cache=True, nogil=True)(fn) if AVAILABLE else fn


@_jit
def _sin(t):
    """sin t for |t| no more than π/2, as a polynomial (to 7e-10: a quarter of the time math.sin takes)."""
    t2 = t * t
    return t * (1.0 + t2 * (-1.0 / 6 + t2 * (1.0 / 120 + t2 * (-1.0 / 5040 + t2 * (1.0 / 362880 + t2 * (
        -1.0 / 39916800 + t2 * (1.0 / 6227020800)))))))


@_jit
def _loss(m, mid, inv_dc, lo, hi, inv_soft):
    """A mode's loss (lasermodel._losses): sin²(π δ / dc), δ its distance from its chip resonance's best place, and
    ((how far beyond the window's edge) / soft)² (inv_dc, inv_soft: 1 / dc, 1 / soft)."""
    u = (m - mid) * inv_dc
    s = _sin(math.pi * (u - math.floor(u + 0.5)))
    w = (max(m - hi, 0.0) + max(lo - m, 0.0)) * inv_soft
    return s * s + w * w


@_jit
def _edge(m, lo, hi, inv_soft):
    """A mode's loss beyond the window's edge alone (what its loss is at least)."""
    w = (max(m - hi, 0.0) + max(lo - m, 0.0)) * inv_soft
    return w * w


@_jit
def _far(m, mid, inv_dc, g):
    """Whether a mode is too far from its chip resonance's best place for its sin² part to be g or less: sin²(π x) is
    at least 4 x² (x = |δ| / dc, no more than ½), so if that is more than g, so is it."""
    u = (m - mid) * inv_dc
    x = u - math.floor(u + 0.5)
    return 4.0 * x * x > g


@_jit
def _index(Ms, x, per):
    """The first of the modes (GHz, rising) at x or above (numpy's searchsorted): where an evenly spaced comb puts it
    (per: modes per GHz), then a step or so either way to be sure."""
    K = Ms.shape[0]
    k = min(max(int(round((x - Ms[0]) * per)), 0), K)
    while k > 0 and Ms[k - 1] >= x:
        k -= 1
    while k < K and Ms[k] < x:
        k += 1
    return k


@_jit
def _best(Ms, mid, dc, inv_dc, lo, hi, inv_soft, below):
    """(the mode of least loss at a point, if its loss is below `below`: the first of them, as numpy's argmin, -1 if
    none is; its loss), Ms its modes' GHz (rising). Outward from the window's middle, each way until a mode's loss
    beyond the edge alone is more than the least so far (from `below` down), a mode's sin² worked out only if it is
    near enough a chip resonance to be less (_far): the rest cannot be. A hop only to a mode whose loss is below the
    one it is on, less the hysteresis: that, as `below`, leaves most points with no mode to look at."""
    K = Ms.shape[0]
    best, gb = -1, below
    if below <= 0.0:
        return best, gb
    per = (K - 1) / (Ms[K - 1] - Ms[0]) if K > 1 and Ms[K - 1] > Ms[0] else 0.0
    k0 = min(_index(Ms, 0.5 * (lo + hi), per), K - 1)
    k = k0
    while k < K:
        m = Ms[k]
        w = _edge(m, lo, hi, inv_soft)
        if w >= gb:
            if m > hi:
                break
        elif not _far(m, mid, inv_dc, gb - w):
            g = _loss(m, mid, inv_dc, lo, hi, inv_soft)
            if g < gb or (g == gb and best >= 0 and k < best):
                best, gb = k, g
        k += 1
    k = k0 - 1
    while k >= 0:
        m = Ms[k]
        w = _edge(m, lo, hi, inv_soft)
        if w >= gb:
            if m < lo:
                break
        elif not _far(m, mid, inv_dc, gb - w):
            g = _loss(m, mid, inv_dc, lo, hi, inv_soft)
            if g < gb or (g == gb and best >= 0 and k < best):
                best, gb = k, g
        k -= 1
    return best, gb


@_jit
def _fades(T, s, lag, out):
    """How much of the chip's warmth from each point of sweep s is left at the next (exp(-Δt / lag)), all at once
    (each its own exp, none waiting on the one before)."""
    out[0] = 1.0
    for j in range(1, T.shape[1]):
        out[j] = math.exp(-(T[s, j] - T[s, j - 1]) / lag)


@_jit
def select(M, B, A, V, T, up, WT, meas, valid, ci0, i0, v0, P, cur, scored, forced):
    """lasermodel._select, compiled: (how many valid points each candidate put on the mode measured, if scored; the
    modes chosen: candidates x sweeps x points, if not). P: candidates x 12, as LaserModel.rule(); cur: candidates x
    sweeps, the mode each begins on."""
    npop = P.shape[0]
    n, L, K = M.shape
    right = np.zeros(npop)
    chosen = np.empty((0 if scored else npop, n, L), dtype=np.int64)
    fade = np.empty(L)
    for c in range(npop):
        lo, hi, h_up, h_down, offset = P[c, 0], P[c, 1], P[c, 2], P[c, 3], P[c, 4]
        pull, lag, dc, ci, w_v, w_i, soft = P[c, 5], max(P[c, 6], 1e-9), P[c, 7], P[c, 8], P[c, 9], P[c, 10], P[c, 11]
        inv_dc, inv_soft = 1.0 / dc, 1.0 / soft
        for s in range(n):
            _fades(T, s, lag, fade)
            now = cur[c, s]
            warm = A[s, 0]
            count = 0
            for j in range(L):
                a = A[s, j]
                if j > 0:
                    warm = a + (warm - a) * fade[j]
                    if forced:
                        now = meas[s, j - 1]
                    base = B[s, j] + (ci - ci0) * (a - i0) + offset + ci * (warm - a)
                    shift = w_v * (V[s, j] - v0) + w_i * (a - i0) + WT[s, j]
                    fc = M[s, j, now]
                    d = fc - base
                    mid = base + pull * (d - dc * math.floor((d + 0.5 * dc) * inv_dc))
                    g_now = _loss(fc, mid, inv_dc, lo + shift, hi + shift, inv_soft)
                    best, gb = _best(M[s, j], mid, dc, inv_dc, lo + shift, hi + shift, inv_soft,
                                     g_now - (h_up if up[s, j] else h_down))
                    if best >= 0:  # (a mode better than the one it is on by more than the hysteresis: the best)
                        now = best
                if scored:
                    if valid[s, j] and now == meas[s, j]:
                        count += 1
                else:
                    chosen[c, s, j] = now
            right[c] += count
    return right, chosen


@_jit
def likelihood(M, B, A, V, T, up, WT, meas, valid, ci0, i0, v0, P, eps):
    """lasermodel._likelihood, compiled: the mean log likelihood of what the laser did for each candidate (P:
    candidates x 13, a rule and its temperature). Its softmax over the modes whose loss is within 20 temperatures of
    the least, and of the mode before's less the hysteresis (the rest add less than e^-20, 2e-9, to it: its log
    changes by less than 1e-8), found as _best finds its mode: outward from the window's middle, a mode's loss
    worked out only if it could be within that."""
    npop = P.shape[0]
    n, L, K = M.shape
    out = np.zeros(npop)
    ks, gs = np.empty(K, dtype=np.int64), np.empty(K)
    fade = np.empty(L)
    total = 0
    for s in range(n):
        for j in range(L):
            if valid[s, j]:
                total += 1
    for c in range(npop):
        lo, hi, h_up, h_down, offset = P[c, 0], P[c, 1], P[c, 2], P[c, 3], P[c, 4]
        pull, lag, dc, ci, w_v, w_i, soft = P[c, 5], max(P[c, 6], 1e-9), P[c, 7], P[c, 8], P[c, 9], P[c, 10], P[c, 11]
        temp = P[c, 12]
        inv_dc, inv_soft, inv_temp, room = 1.0 / dc, 1.0 / soft, 1.0 / temp, 20.0 * temp
        acc = 0.0
        for s in range(n):
            _fades(T, s, lag, fade)
            warm = A[s, 0]
            for j in range(1, L):
                a = A[s, j]
                warm = a + (warm - a) * fade[j]
                if not valid[s, j]:
                    continue
                base = B[s, j] + (ci - ci0) * (a - i0) + offset + ci * (warm - a)
                shift = w_v * (V[s, j] - v0) + w_i * (a - i0) + WT[s, j]
                was, now = meas[s, j - 1], meas[s, j]
                Ms = M[s, j]
                fc = Ms[was]
                d = fc - base
                mid = base + pull * (d - dc * math.floor((d + 0.5 * dc) * inv_dc))
                lo_s, hi_s = lo + shift, hi + shift
                h = h_up if up[s, j] else h_down
                g_was = _loss(fc, mid, inv_dc, lo_s, hi_s, inv_soft)
                per = (K - 1) / (Ms[K - 1] - Ms[0]) if K > 1 and Ms[K - 1] > Ms[0] else 0.0
                k0 = min(_index(Ms, 0.5 * (lo_s + hi_s), per), K - 1)
                least = np.inf  # (a mode whose loss is room above the mode before's less the hysteresis, or above
                count = 0  # the least's: its term less than e^-20 of the largest, which is either's)
                for way in (1, -1):
                    k = k0 if way == 1 else k0 - 1
                    while 0 <= k < K:
                        m = Ms[k]
                        w = _edge(m, lo_s, hi_s, inv_soft)
                        bound = min(g_was - h, least) + room
                        if w >= bound:
                            if (way == 1 and m > hi_s) or (way == -1 and m < lo_s):
                                break
                        elif not _far(m, mid, inv_dc, bound - w):
                            g = _loss(m, mid, inv_dc, lo_s, hi_s, inv_soft)
                            ks[count], gs[count] = k, g
                            count += 1
                            if g < least:
                                least = g
                        k += way
                reach = min(g_was - h, least) + room
                top = 0.0  # (its mode before: 0)
                for q in range(count):
                    if gs[q] <= reach and ks[q] != was:
                        top = max(top, (g_was - gs[q] - h) * inv_temp)
                total_exp = math.exp(-top)  # (its mode before)
                z_now = 0.0 if now == was else -np.inf
                for q in range(count):
                    if gs[q] <= reach and ks[q] != was:
                        zq = (g_was - gs[q] - h) * inv_temp
                        total_exp += math.exp(zq - top)
                        if ks[q] == now:
                            z_now = zq
                if z_now == -np.inf:  # (the mode it went to: its term negligible, but its likelihood is wanted)
                    z_now = (g_was - _loss(Ms[now], mid, inv_dc, lo_s, hi_s, inv_soft) - h) * inv_temp
                p = math.exp(z_now - top) / total_exp
                acc += math.log((1.0 - eps) * p + eps / K)
        out[c] = acc
    return out / max(total, 1)
