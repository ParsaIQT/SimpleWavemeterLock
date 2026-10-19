"""The trace analysis: SHG sweeps, cavity peaks, the error signal's DC level."""
import math
import random

import pytest

from wmlock.cavity import (ModeCheck, ModeRef, choose_peak, dc_level, decimate, find_peaks, judge, peak_top,
                           reference_from, same_peak, sweep, updated)


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


def test_the_true_height_of_a_peak_is_had_between_samples():
    """What the SHG cavity locked to the laser would show, its intra-cavity signal's maximum: from a sweep whose
    resonances are two samples wide, wherever the samples fall (the highest sample alone: 20 % short at worst)."""
    tops, highest = [], []
    for k in range(8):
        phase = 0.3 + k / 8 / 400  # (an eighth of a sample at a time)
        m = sweep(shg_sweep([(1.0, 0.0)], n=1200, finesse=200, phase=phase, fsr_samples=400))
        tops.append(m.top)
        highest.append(m.height)
    assert min(tops) == pytest.approx(1.0, abs=0.03) and max(tops) == pytest.approx(1.0, abs=0.03)
    assert min(highest) < 0.9
    assert sweep(shg_sweep([(0.7, 0.0), (0.3, 0.45)], finesse=200)).top == pytest.approx(0.7, abs=0.03)
    y = [0.0, 0.2, 1.0, 0.2, 0.0]  # a Lorentzian sampled on its top: as it is
    assert peak_top(y, 2, 0.0) == pytest.approx(1.0)
    y = [1.0 / (1 + (2 * (x - 2.5)) ** 2) for x in range(6)]  # ...and half way between samples
    assert peak_top(y, 2, 0.0) == pytest.approx(1.0, abs=1e-9) and peak_top(y, 0, 0.0) == pytest.approx(y[0])


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
        assert dc_level(err, trans, list(range(n))) == pytest.approx(0.012, abs=1e-3), c  # (about the carrier)
        assert dc_level(err) == pytest.approx(0.012, abs=2e-3), c
    assert math.isnan(dc_level([]))


def test_dc_level_of_the_lab_error_signal():
    """As on the lab's screen (SimLocked.LAB): a carrier and its sidebands 20 MHz out, the dispersion's tails
    between them (some 0.04 V, up on one side of the carrier and down on the other), weaker lines 52 MHz either side,
    all a sample or so wide, the transmission showing only the lines: its DC level to a millivolt, wherever the
    carrier is in the scan (without the scan voltage, near the middle, where a relock has it). The mean of the
    middle half of the samples misses by several."""
    from wmlock.sim import ECDL, FakeDLCPro, SimLocked

    laser = ECDL(400.0, seed=3)
    dlc = FakeDLCPro("SN", "lab", {1: {"current": 150.0, "offset": 70.0, "scan": True}}, serve=False)
    dlc.params["laser1:scan:amplitude"] = SimLocked.LAB_SCAN
    lock = SimLocked(dlc, 1, laser, laser.freq(70.0, 150.0), seed=3, pdh_dc=0.007, **SimLocked.LAB)
    worst = blind = trimmed = 0.0
    for k in range(-10, 11):
        lock.wobble = 2.0 * k  # MHz: the carrier anywhere within 20 MHz of the middle of the scan
        x, trans, err = zip(*lock._master_scan())
        worst = max(worst, abs(dc_level(err, trans, x) - 0.007))
        if abs(k) <= 2:
            blind = max(blind, abs(dc_level(err) - 0.007))
        s = sorted(err)
        trimmed = max(trimmed, abs(sum(s[250:750]) / 500 - 0.007))
    assert worst < 0.0012 and blind < 0.0012 and trimmed > 0.003


def test_a_narrow_peak_sampled_low_is_still_the_tem00():
    """A TEM00 peak a sample or so wide shows anywhere from about half its height to all of it in a scan; the
    tallest of a few scans counts. A transverse mode a third as tall is still not taken for it."""
    kv = 400.0
    scan = [(70.0, 0.21, "t"), (70.13, 0.008, "t"), (69.87, 0.007, "t")]  # 0.42 of the usual 0.5
    c = choose_peak(scan, x_pred=70.0, kv=kv, window=150, y_ref=0.5)
    assert c.ok and c.x == 70.0
    assert not choose_peak([(71.39, 0.15, "t")], x_pred=71.39, kv=kv, window=150, y_ref=0.5).ok
    c = choose_peak(scan + [(70.0003, 0.48, "t")], x_pred=70.0, kv=kv, window=150, y_ref=0.5)  # and another scan
    assert c.ok and c.y == 0.48
    # the peak chosen, as the next scan shows it: the nearest candidate of its type
    assert same_peak([(69.9, 0.4, "t"), (70.001, 0.2, "t"), (70.0, 0.1, "p")], 70.0, "t", 0.09) == (70.001, 0.2, "t")
    assert same_peak([(69.9, 0.4, "t")], 70.0, "t", 0.09) is None


def test_decimate_keeps_narrow_peaks():
    y = [0.0] * 5000
    y[1234] = 1.0
    d = decimate(y, 400)
    assert len(d) == 400 and max(d) == 1.0
    assert decimate([1.0, 2.0], 400) == [1.0, 2.0]
    assert all(math.isfinite(v) for v in d)
