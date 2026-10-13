"""Reading a DLC pro's own diagnostics the way you would by eye.

  SHG sweep        with the master laser parked and the SHG cavity scanned, the intra-cavity
                   signal shows one resonance per free spectral range for each frequency the
                   master emits. A single-mode master gives one tall peak per FSR (plus small
                   transverse-mode bumps from imperfect mode matching, always the same); a
                   multimode one gives a second family of peaks, and the main peaks lose
                   what the other mode takes. sweep() measures the peaks, judge() compares
                   them with what a single-mode laser looks like on this system.
  cavity scan      with the master scanned, the reference cavity's transmission shows its
                   resonances: TEM00 peaks one FSR apart, weaker transverse modes between.
                   The DLC pro marks lockpoint candidates on them; choose_peak() picks the
                   TEM00 peak where the wavemeter says the target is.
  error signal     the PDH error signal should have no DC level away from the resonances;
                   the FALC pro's input offset cancels one. dc_level() measures it.

Peaks are characterised by their area (the sum of the samples above the floor) as well as their
height: a resonance only a few samples wide can be sampled anywhere between its top and its
flank, which changes its height a lot and its area little.

Pure functions on plain lists: no Qt, no I/O.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


def _median(xs) -> float:
    xs = sorted(xs)
    n = len(xs)
    if not n:
        return math.nan
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def floor_level(y) -> float:
    """The level between resonances: the 10th percentile."""
    s = sorted(y)
    return s[len(s) // 10] if s else math.nan


def noise(y) -> float:
    """rms of the sample-to-sample noise (resonances are few, so the median ignores them)."""
    if len(y) < 3:
        return 0.0
    return _median([abs(b - a) for a, b in zip(y, y[1:])]) / 0.6745 / math.sqrt(2)


@dataclass
class Peak:
    i: int        # index of its top
    h: float      # height of the top above the floor
    w: float      # area: the sum of its samples above the floor
    lo: int       # its extent, indices
    hi: int


def find_peaks(y, rel: float = 0.03) -> list:
    """Resonances in y: local maxima standing out by more than `rel` of the full range (and well
    above the noise) from what lies between them and any higher sample, either side."""
    n = len(y)
    if n < 3:
        return []
    base = floor_level(y)
    top = max(y)
    span = top - base
    sigma = noise(y)
    if span <= 0 or span < 6 * sigma:
        return []
    need = max(rel * span, 4 * sigma)
    edge = max(0.02 * span, 3 * sigma)  # a peak's extent: down to this above the floor
    peaks = []
    for i in range(n):
        v = y[i]
        if v - base < need:
            continue
        left = y[i - 1] if i > 0 else -math.inf
        right = y[i + 1] if i < n - 1 else -math.inf
        if not (v > left and v >= right):
            continue
        lo_min, j = v, i - 1  # prominence: the deepest dip before a higher sample, each side
        while j >= 0 and y[j] <= v:
            lo_min = min(lo_min, y[j])
            j -= 1
        hi_min, k = v, i + 1
        while k < n and y[k] <= v:
            hi_min = min(hi_min, y[k])
            k += 1
        if v - max(lo_min if j >= 0 else base, hi_min if k < n else base) < need:
            continue
        lo = i
        while lo > 0 and y[lo - 1] - base > edge and y[lo - 1] <= y[lo] + 3 * sigma:
            lo -= 1
        hi = i
        while hi < n - 1 and y[hi + 1] - base > edge and y[hi + 1] <= y[hi] + 3 * sigma:
            hi += 1
        peaks.append(Peak(i, v - base, sum(y[m] - base for m in range(lo, hi + 1)), lo, hi))
    out = []
    for p in peaks:  # a noisy top can count twice: keep the higher of overlapping peaks
        if out and p.lo <= out[-1].hi + 1:
            if p.h > out[-1].h:
                out[-1] = p
            continue
        out.append(p)
    return out


# ------------------------------------------------------------------ SHG sweep
@dataclass
class ModeRef:
    """What the SHG sweep of this laser looks like when it is single-mode."""
    area: float = 0.0   # area of a main peak
    side: float = 0.0   # side-peak ratio (transverse modes: always there)
    n_main: int = 0     # main peaks in one sweep, with these scan settings

    @property
    def known(self) -> bool:
        return self.area > 0


@dataclass
class ModeCheck:
    seen: bool = False     # any resonance at all
    area: float = 0.0      # of the biggest peak
    height: float = 0.0    # of the biggest peak, above the floor
    floor: float = 0.0     # the level between resonances
    peaks: list = field(default_factory=list)  # [(index, height, area)], whole peaks only

    def split(self, area: float = 0.0):
        """(side, n_main): main peaks have at least MAIN of `area` (by default the biggest one's);
        side is the biggest of the others over the biggest main one (a second mode makes one
        such peak per FSR, all alike; transverse-mode bumps stay small)."""
        cut = MAIN * (area or self.area)
        main = [w for _, _, w in self.peaks if w >= cut]
        side = [w for _, _, w in self.peaks if w < cut]
        if not main:
            return math.inf, 0
        return (max(side) / max(main) if side else 0.0), len(main)


MAIN = 0.6  # a peak with at least this fraction of a main peak's area is one too


def sweep(y) -> ModeCheck:
    """Measure an SHG sweep (the intra-cavity signal, in acquisition order). Peaks cut off by
    the ends of the trace are left out."""
    peaks = find_peaks(y)
    whole = [p for p in peaks if p.lo > 0 and p.hi < len(y) - 1] or peaks
    if not whole:
        return ModeCheck()
    big = max(whole, key=lambda p: p.w)
    if big.w <= 0:
        return ModeCheck()
    return ModeCheck(True, big.w, big.h, floor_level(y), [(p.i, p.h, p.w) for p in whole])


SIDE_DEFAULT = 0.2   # with nothing to compare to: a side peak bigger than this means multimode
SIDE_MARGIN = 0.1    # a side peak bigger than usual by this much: multimode
AREA_MIN = 0.8       # main peaks smaller than this fraction of usual: power is going elsewhere


def judge(m: ModeCheck, ref: ModeRef | None = None):
    """(ok, text): True single-mode, False multimode, None cannot tell (no resonance seen).

    Knowing the laser's single-mode look (ref) catches what a sweep alone cannot: two modes of
    about equal power look like one clean mode, but each main peak is about half as big."""
    if not m.seen:
        return None, "no resonance in the SHG sweep (is its scan wide enough?)"
    if ref is None or not ref.known:
        side, _ = m.split()
        pct = f"side peak {side * 100:.0f} %"
        return (False, f"multimode: {pct}") if side > SIDE_DEFAULT else (True, f"single mode: {pct}")
    side, _ = m.split(ref.area)
    usual = f"usually {ref.side * 100:.0f} %"
    if m.area < AREA_MIN * ref.area:
        return False, f"multimode: main peaks at {m.area / ref.area:.2f} of usual"
    if side > ref.side + SIDE_MARGIN:
        return False, f"multimode: side peak {side * 100:.0f} %, {usual}"
    return True, f"single mode: side peak {side * 100:.0f} %, {usual}"


def reference_from(checks) -> ModeRef | None:
    """The single-mode look of a laser from many sweeps across its modes (a map): where all of its
    power is in one mode, the main peaks are biggest. The quarter with the biggest are taken."""
    seen = sorted((c for c in checks if c.seen), key=lambda c: -c.area)
    if len(seen) < 6:
        return None
    best = seen[:max(3, len(seen) // 4)]
    area = _median([c.area for c in best])
    splits = [c.split(area) for c in best]
    return ModeRef(area, _median([s for s, _ in splits]), int(round(_median([n for _, n in splits]))))


def updated(ref: ModeRef | None, m: ModeCheck) -> ModeRef:
    """Learn from a sweep judged single-mode."""
    if ref is None or not ref.known:
        side, n = m.split()
        return ModeRef(m.area, side, n)
    side, n = m.split(ref.area)
    return ModeRef(0.7 * ref.area + 0.3 * m.area, 0.7 * ref.side + 0.3 * side, n)


# ---------------------------------------------------------------- cavity scan
@dataclass
class PeakChoice:
    ok: bool
    x: float = math.nan    # the lockpoint candidate chosen (scan output, V)
    y: float = math.nan    # its level
    kind: str = ""         # its type, as the DLC pro marks it
    off: float = math.nan  # MHz: where the wavemeter puts it relative to the target
    text: str = ""
    tall: list = field(default_factory=list)  # [(x, y, off MHz)] of every tall peak seen


def choose_peak(cands, x_pred: float, kv: float, window: float, y_ref: float = 0.0, kind: str = "",
                gap: float = 0.0) -> PeakChoice:
    """The lockpoint candidate on the right peak.

    cands: [(x, y, type)] from the DLC pro. x_pred: scan output (V) where the target frequency is,
    from the wavemeter; kv: MHz/V. The right peak is a tall one (at least half as tall as the
    usual TEM00 peak, y_ref, or as the tallest in the scan) within +-window MHz of x_pred, and the
    only one there. Candidates of `kind` are preferred (the type of the lockpoint chosen by hand);
    candidates closer than `gap` V belong to one peak, whose highest is taken."""
    if kind and any(t == kind for _, _, t in cands):
        cands = [c for c in cands if c[2] == kind]
    if not cands:
        return PeakChoice(False, text="the DLC pro offers no lockpoint candidates (is the scan on?)")
    gap = gap or 0.25 * window / max(abs(kv), 1e-9)
    groups = []
    for c in sorted(cands):
        if groups and c[0] - groups[-1][-1][0] <= gap:
            groups[-1].append(c)
        else:
            groups.append([c])
    tops = [max(g, key=lambda c: c[1]) for g in groups]
    highest = max(y for _, y, _ in tops)
    if y_ref > 0 and highest < 0.5 * y_ref:
        return PeakChoice(False, text=f"no TEM00 peak in the scan: the tallest is {highest / y_ref:.2f} "
                                      f"of the usual height")
    tall = [(x, y, t, (x - x_pred) * kv) for x, y, t in tops if y >= 0.5 * max(y_ref, highest)]
    seen = [(x, y, off) for x, y, _, off in tall]
    near = sorted((c for c in tall if abs(c[3]) <= window), key=lambda c: abs(c[3]))
    if not near:
        nearest = min(tall, key=lambda c: abs(c[3]))
        return PeakChoice(False, text=f"no tall peak within ±{window:.0f} MHz of the target "
                                      f"(nearest {nearest[3]:+.0f} MHz)", tall=seen)
    if len(near) > 1 and abs(near[1][3]) < 2 * abs(near[0][3]) + 0.25 * window:
        return PeakChoice(False, text=f"two tall peaks near the target ({near[0][3]:+.0f} and "
                                      f"{near[1][3]:+.0f} MHz): cannot tell which", tall=seen)
    x, y, t, off = near[0]
    return PeakChoice(True, x, y, t, off, f"peak at {x:.3f} V, {off:+.0f} MHz from the target", seen)


def dc_level(err, trans=None) -> float:
    """The error signal's level away from the resonances. With the transmission (same samples),
    the samples near its peaks are left out; of the rest, the mean of the middle half (the
    dispersive tails either side of a resonance cancel, the resonances themselves drop out)."""
    keep = list(err)
    if trans is not None and len(trans) == len(err) and err:
        for widen in (10, 3, 1):
            near = set()
            for p in find_peaks(trans):
                w = p.hi - p.lo + 1
                near.update(range(p.lo - widen * w, p.hi + widen * w + 1))
            rest = [e for k, e in enumerate(err) if k not in near]
            if len(rest) >= len(err) // 5:
                keep = rest
                break
    if not keep:
        return math.nan
    s = sorted(keep)
    mid = s[len(s) // 4:len(s) - len(s) // 4] or s
    return sum(mid) / len(mid)


def decimate_xy(x, y, n: int = 400):
    """(x, y) cut down to about n points for a plot, keeping each bucket's highest y."""
    if len(y) <= n:
        return [round(v, 5) for v in x], [round(v, 5) for v in y]
    xs, ys, k = [], [], len(y) / n
    for b in range(n):
        lo, hi = int(b * k), max(int((b + 1) * k), int(b * k) + 1)
        j = max(range(lo, min(hi, len(y))), key=lambda m: y[m])
        xs.append(round(x[j], 5))
        ys.append(round(y[j], 5))
    return xs, ys


def decimate(y, n: int = 400) -> list:
    """At most about n points for a log or a plot, keeping narrow peaks: (min, max) per bucket."""
    if len(y) <= n:
        return [round(v, 5) for v in y]
    out, k = [], len(y) / (n / 2)
    for b in range(n // 2):
        chunk = y[int(b * k):max(int((b + 1) * k), int(b * k) + 1)]
        lo, hi = min(chunk), max(chunk)
        first = chunk.index(lo) < chunk.index(hi)
        out += [round(lo, 5), round(hi, 5)] if first else [round(hi, 5), round(lo, 5)]
    return out
