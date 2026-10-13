"""The trace analysis: SHG sweeps, cavity peaks, the error signal's DC level."""
import math
import random

import pytest

from wmlock.cavity import (ModeCheck, ModeRef, choose_peak, dc_level, decimate, find_peaks, judge,
                           reference_from, sweep, updated)


def lorentz(x, w):
    return 1.0 / (1.0 + (2 * x / w) ** 2)


def shg_sweep(modes, n=1000, fsr_samples=400, finesse=150, tem=((0.31, 0.05), (0.62, 0.03)), phase=0.123,
              noise=0.002, seed=0):
    """An intra-cavity signal over a linear sweep: for each laser mode (power, phase in FSR),
    a TEM00 resonance every FSR and small transverse modes."""
    rng = random.Random(seed)
    w = 1.0 / finesse
    y = []
    for k in range(n):
        s = k / fsr_samples + phase
        v = 0.0
        for power, ph in modes:
            for off, h in ((0.0, 1.0),) + tuple(tem):
                d = (s - ph - off) % 1.0
                d = min(d, 1.0 - d)
                v += power * h * lorentz(d, w)
        y.append(0.05 + v + rng.gauss(0.0, noise))
    return y


def test_peaks_are_found_however_narrow_and_sampled():
    heights, areas = [], []
    for phase in (0.3, 0.30063, 0.30125):  # the resonances fall on a sample, a quarter, half way between
        y = shg_sweep([(1.0, 0.0)], n=1200, finesse=400, phase=phase, fsr_samples=400)  # 1 sample wide
        main = [p for p in find_peaks(y) if p.h > 0.3]
        assert len(main) == 3
        heights.append(main[0].h)
        areas.append(main[0].w)
    assert max(areas) / min(areas) < 1.15  # the area does not care where the samples fall...
    assert max(heights) / min(heights) > 1.5  # ...the height does


def test_flat_or_noisy_traces_have_no_peaks():
    rng = random.Random(1)
    assert find_peaks([0.1 + rng.gauss(0, 0.01) for _ in range(500)]) == []
    assert find_peaks([0.2] * 100) == [] and find_peaks([]) == []
    m = sweep([0.1 + rng.gauss(0, 0.01) for _ in range(500)])
    assert not m.seen and judge(m) == (None, "no resonance in the SHG sweep (is its scan wide enough?)")


def test_single_mode_and_multimode_sweeps():
    single = sweep(shg_sweep([(1.0, 0.0)]))
    side, n_main = single.split()
    assert single.seen and n_main == 2 and side == pytest.approx(0.035, abs=0.02)  # the biggest TEM bump
    assert judge(single)[0] is True
    seventy_thirty = sweep(shg_sweep([(0.7, 0.0), (0.3, 0.45)]))
    assert seventy_thirty.split()[0] == pytest.approx(0.3 / 0.7, abs=0.05) and judge(seventy_thirty)[0] is False
    ninety_ten = sweep(shg_sweep([(0.85, 0.0), (0.15, 0.45)]))
    assert judge(ninety_ten)[0] is True  # 15 % in a second mode: too little to tell without knowing the laser
    ref = ModeRef(single.area, side, n_main)
    assert judge(single, ref)[0] is True
    ok, text = judge(seventy_thirty, ref)
    assert ok is False and "multimode" in text
    assert judge(ninety_ten, ref)[0] is False  # ...but knowing it, a side peak 18 % against its usual 3 %
    # equal powers: the second family counts as main peaks too, but there are twice as many
    even = sweep(shg_sweep([(0.5, 0.0), (0.5, 0.45)]))
    assert judge(even)[0] is True  # without knowing the laser, this looks clean...
    ok, text = judge(even, ref)
    assert ok is False  # ...but its main peaks are half as big as usual, and twice as many
    # a little weaker overall (less light) is still single mode
    assert judge(sweep(shg_sweep([(0.85, 0.0)])), ref)[0] is True


def test_reference_from_a_map_and_learning():
    checks = [sweep(shg_sweep([(1.0, 0.0)], seed=s)) for s in range(8)]
    checks += [sweep(shg_sweep([(0.6, 0.0), (0.4, 0.3)], seed=s)) for s in range(4)]
    ref = reference_from(checks)
    assert ref.n_main == 2 and ref.side < 0.12 and ref.area == pytest.approx(checks[0].area, rel=0.05)
    assert reference_from(checks[:4]) is None  # too few to tell
    r2 = updated(ref, checks[-1])  # a multimode sweep would drag it down: only learn from good ones
    assert r2.area == pytest.approx(0.7 * ref.area + 0.3 * checks[-1].area)
    assert updated(None, checks[0]).area == checks[0].area and updated(None, checks[0]).n_main == 2
    assert not ModeRef().known and judge(ModeCheck(), ref)[0] is None


def test_choose_the_tem00_peak_where_the_wavemeter_says():
    kv = 400.0  # MHz/V
    # TEM00 peaks 1.5 GHz apart (3.75 V), a TEM01 0.37 FSR above each, edges either side of each top
    cands = []
    for k in (-1, 0, 1):
        x00 = 70.0 + 3.75 * k
        cands += [(x00, 0.80, "t"), (x00 - 0.01, 0.4, "p"), (x00 + 0.01, 0.4, "n"),
                  (x00 + 0.37 * 3.75, 0.15, "t")]
    c = choose_peak(cands, x_pred=70.05, kv=kv, window=150)
    assert c.ok and c.x == 70.0 and c.y == 0.8 and c.kind == "t" and c.off == pytest.approx(-20)
    assert len(c.tall) == 3
    # predicted near the transverse mode: no TEM00 within the window
    c = choose_peak(cands, x_pred=70.0 + 0.37 * 3.75, kv=kv, window=150)
    assert not c.ok and "no tall peak" in c.text
    # only the transverse mode in view: not taken for the TEM00, given the usual height
    c = choose_peak([(71.39, 0.15, "t")], x_pred=71.39, kv=kv, window=150, y_ref=0.8)
    assert not c.ok and "no TEM00" in c.text
    # two tall peaks inside the window: refuse to guess
    c = choose_peak([(70.0, 0.8, "t"), (70.25, 0.8, "t")], x_pred=70.12, kv=kv, window=150)
    assert not c.ok and "cannot tell" in c.text
    # the type chosen by hand is preferred: an edge lock stays an edge lock
    c = choose_peak(cands, x_pred=70.0, kv=kv, window=150, kind="p")
    assert c.ok and c.kind == "p" and c.x == pytest.approx(69.99)
    assert not choose_peak([], 70.0, kv, 150).ok


def test_dc_level_ignores_the_resonances():
    n = 1000
    for c in (500, 300.3, 812.7):  # a resonance anywhere in the scan, with its dispersive tails
        trans = [0.8 * lorentz(k - c, 1.0) for k in range(n)]
        err = [0.012 + 0.5 * (-(k - c) / 0.5) * lorentz(k - c, 1.0) for k in range(n)]
        assert dc_level(err, trans) == pytest.approx(0.012, abs=1e-3), c
        assert dc_level(err) == pytest.approx(0.012, abs=2e-3), c
    assert math.isnan(dc_level([]))


def test_decimate_keeps_narrow_peaks():
    y = [0.0] * 5000
    y[1234] = 1.0
    d = decimate(y, 400)
    assert len(d) == 400 and max(d) == 1.0
    assert decimate([1.0, 2.0], 400) == [1.0, 2.0]
    assert all(math.isfinite(v) for v in d)
