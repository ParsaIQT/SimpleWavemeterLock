"""The random lasers and the bench trials.py tries the program on (and what sim.py gained for them): lasers that are
the same for the same seed, a rig that keeps time, an ECDL's power along its modes, a model's laser with a second mode
before it hops and its hysteresis by the edge it is at, and the trials themselves, small."""
import json
import math
import statistics

import pytest

from wmlock import trials
from wmlock.modehop import HopParams, Move, Recenter, Wait
from wmlock.sim import ECDL, ModelLaser, SimCCD
from wmlock.trials import I0, V0, Rig, far_ecdl, make_case, near_ecdl, recover, recoveries, strong_ecdl, summary


def test_a_laser_is_the_same_for_the_same_seed():
    for make in (far_ecdl, strong_ecdl, near_ecdl):
        a, b, c = make(3)[0], make(3)[0], make(4)[0]
        assert (a.fsr, a.kv, a.ki, a.ksi, a.hysteresis, a.multimode) == (b.fsr, b.kv, b.ki, b.ksi, b.hysteresis,
                                                                         b.multimode)
        assert a.fsr != c.fsr
    for kind in ("far", "strong", "shg", "near"):
        x, y = make_case(kind, 5), make_case(kind, 5)
        assert x.hop == pytest.approx(y.hop) and x.feasible == y.feasible and abs(x.hop) > 0.5  # (GHz: it hopped)
    with pytest.raises(ValueError):
        make_case("green", 1)


def test_the_models_lasers_are_drawn_like_the_labs():
    pytest.importorskip("numpy")
    a, b = trials.random_model(2), trials.random_model(2)
    assert a.to_dict() == b.to_dict() and a.to_dict() != trials.random_model(3).to_dict()
    for seed in range(12):
        m = trials.random_model(seed)
        assert 3.0 <= m.ext.spacing <= 9.0 and 15.0 <= m.dc <= 112.0
        assert 2.0 <= m.chip.p_i / m.ext.p_i <= 5.0 and m.hi - m.lo >= 1.5 * m.dc - 1e-9
        assert abs(m.dc / m.ext.spacing - round(m.dc / m.ext.spacing)) >= 0.04  # (no whole number of its modes)
    blue, red = trials.lab493(), trials.lab650()
    assert blue.dc == pytest.approx(87.4, abs=0.1) and red.ext.spacing == pytest.approx(3.39, abs=0.01)
    import numpy as np
    other = trials.unlike(blue, np.random.default_rng(1))
    assert other.h_down != blue.h_down and other.offset != blue.offset and other.dc == blue.dc
    c, d = make_case("red", 7), make_case("red", 7)
    assert c.hop == pytest.approx(d.hop) and abs(c.hop) >= 0.4 * red.ext.spacing and c.kwargs["model"] is not None


class Script(Recenter):
    def __init__(self, steps, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.steps = steps

    def run(self):
        return (yield from self.steps(self))


def test_a_rig_keeps_time_as_the_laser_would():
    las = ECDL(473.0, fsr=12.0, hysteresis=0.5, multimode=0.0, seed=1)
    rig = Rig(las, las.freq(V0, I0), dt=0.1, op_s=0.002, scope_s=0.05)
    p = HopParams(enabled=True, i_nom=I0, smooth=True)

    def steps(rec):
        yield Move(current=I0 - 0.5, read=False)  # (25 writes of 0.02 mA, 2 ms each: no reading waited for)
        t1 = rig.t
        yield Wait(0.37)
        t2 = rig.t
        yield Move(current=I0 - 0.52)             # (one write; then the reading after the one that straddled it)
        return t1, t2, rig.t

    r = rig.run(Script(steps, p, I0, V0, I0 - 2, I0 + 2, V0 - 10, V0 + 10))
    t1, t2, t3 = r.result
    assert r.state == "done" and t1 == pytest.approx(25 * 0.002, abs=1e-6) and rig.ops == 26
    assert t2 - t1 == pytest.approx(0.37, abs=0.006)
    assert 0.1 <= t3 - t2 <= 0.2 + 1e-9 and rig.reads >= 2
    assert rig.off_mhz() == pytest.approx(0.52 * 250.0, abs=1.0)  # (0.25 GHz/mA down)
    assert rig.margin() > 1.0 and rig.second_mode() == 0.0
    rig.set(current=I0, offset=V0 + 1.0)  # (by hand)
    assert rig.I == I0 and rig.V == V0 + 1.0 and rig.off_mhz() == pytest.approx(400.0, abs=1.0)


def test_an_ecdls_power_is_a_sawtooth_along_its_modes():
    e = ECDL(473.0, fsr=6.0, ki=-0.25, ksi=-2.0, hysteresis=0.6, multimode=0.3, seed=1, power_dp=-0.2)
    plain = ECDL(473.0, fsr=6.0, ki=-0.25, ksi=-2.0, hysteresis=0.6, multimode=0.3, seed=1)
    rows = []
    for k in range(81):
        i = 150.0 + 0.1 * k
        e.lase(70.0, i)
        plain.lase(70.0, i)
        rows.append((i, e.m, e.power(i), plain.power(i)))
    assert [r[3] for r in rows] == pytest.approx([20.0 + 0.025 * k for k in range(81)])  # (without: its slope alone)
    hops = [k for k in range(1, 81) if rows[k][1] != rows[k - 1][1]]
    assert len(hops) == 2
    for k in hops:  # (at a hop its power jumps up: the new mode sits low on its selection filter)
        assert rows[k][2] - rows[k - 1][2] > 1.0
    along = [rows[k][2] - rows[k - 1][2] for k in range(1, 81) if k not in hops]
    assert max(along) < 0 and statistics.fmean(along) == pytest.approx(0.025 - 0.2 * 1.75 * 0.1, abs=1e-6)
    e.stop()
    assert e.power(155.0) == pytest.approx(0.3)  # (not lasing: a trace, as before)


def red_two_modes(**kw):
    t = [0.0]
    model = trials.two_modes(trials.lab650(), 0.2)
    return model, ModelLaser(model, clock=lambda: t[0], seed=1, **kw), t


def test_a_models_laser_runs_on_two_modes_shortly_before_it_hops():
    pytest.importorskip("numpy")
    model, las, t = red_two_modes()
    assert 0.0 < model.mm < 0.1  # (a loss, as its hysteresis)
    rows = []
    for k in range(400):  # (its current up slowly: 60 to 62 mA)
        i = 60.0 + 0.005 * k
        t[0] += 5.0
        ghz = las.lase(model.v0, i)
        comps = las.components(model.v0, i)
        look = model.look(i, model.v0, model.t0 + t[0], las._state)
        rows.append((i, las._state[0], comps, look, ghz))
    hop = next(k for k in range(1, 400) if rows[k][1] != rows[k - 1][1])
    two = [k for k in range(hop) if len(rows[k][2]) == 2]
    assert two and two == list(range(two[0], hop))  # (two modes from somewhere on, up to the hop: not before)
    assert 0.05 < rows[hop][0] - rows[two[0]][0] < 0.5  # mA of it
    parts = [min(p for p, _ in rows[k][2]) for k in two]
    assert parts == sorted(parts) and parts[0] < 0.05 and 0.4 < parts[-1] <= 0.5  # (growing to a half)
    for k in two:
        i, order, comps, look, ghz = rows[k]
        assert look["order"] == order and look["ghz"] == pytest.approx(ghz)
        assert look["p2"] == pytest.approx(0.5 * (1 - look["u"] / model.mm)) and 0 < look["u"] < model.mm
        assert comps[0] == (pytest.approx(1 - look["p2"]), pytest.approx(ghz))
        assert comps[1] == (pytest.approx(look["p2"]), pytest.approx(look["rival"]))
    # the mode that lased as well is the one it hops to; after the hop, one mode again, holding by a good margin
    assert rows[hop - 1][3]["rival_order"] == rows[hop][1] and len(rows[hop][2]) == 1
    assert rows[hop][3]["u"] > model.mm and rows[hop][3]["p2"] == 0.0
    # on two modes the wavemeter reads noise (and now and then nonsense): what it reads scatters
    i = rows[two[len(two) // 2]][0]
    t[0] += 5.0
    las2 = ModelLaser(model, clock=lambda: t[0], seed=2)
    for k in range(two[len(two) // 2] + 1):
        t[0] += 5.0
        las2.lase(model.v0, 60.0 + 0.005 * k)
    reads = [las2.freq(model.v0, i) for _ in range(200)]
    assert statistics.pstdev(reads) * 1e3 > 0.02  # GHz
    quiet = ModelLaser(model, clock=lambda: t[0], seed=2, mm_noise=0.0)
    for k in range(two[len(two) // 2] + 1):
        t[0] += 5.0
        quiet.lase(model.v0, 60.0 + 0.005 * k)
    assert statistics.pstdev([quiet.freq(model.v0, i) for _ in range(50)]) == 0.0
    # without mm: never two, as it was
    plain, t2 = ModelLaser(trials.lab650(), clock=lambda: 0.0), 0
    assert all(len(plain.components(plain.model.v0, 60.0 + 0.005 * k)) == 1
               for k in range(400) if plain.lase(plain.model.v0, 60.0 + 0.005 * k) is not None)


def test_how_far_its_current_can_go_before_it_hops():
    pytest.importorskip("numpy")
    model, las, t = red_two_modes()
    for k in range(101):  # (onto 61.0 mA from above)
        t[0] += 5.0
        las.lase(model.v0, 62.0 - 0.01 * k)
    order = las._state[0]
    down, up = las.holds(model.v0, 61.0)
    assert trials.holds(las, model.v0, 61.0) == (down, up)
    assert model.margins(61.0, model.v0, model.t0 + t[0], las._state) == (down, up)
    assert math.isfinite(down) and 0.1 < down < 1.5 and las._state[0] == order  # (asked, not moved)
    for k in range(1, int(round(down / 0.01)) + 4):  # (and so it is: it hops going down, just past that)
        t[0] += 50.0
        las.lase(model.v0, 61.0 - 0.01 * k)
        if las._state[0] != order:
            break
    assert las._state[0] != order and down - 0.03 <= 0.01 * k <= down + 0.04
    held, _ = model.margins(61.0, model.v0, model.t0, (order, 61.0, 0.0, 61.0, False), hold=True)
    assert math.isfinite(held) and held != down  # (its piezo moved along, to keep its frequency: another distance)
    t[0] += 5.0
    assert las.lase(model.v0, 10.0) is None and math.isnan(las.holds(model.v0, 10.0)[0])  # (below threshold)


def drift_until_it_hops(sided, last_up, sign):
    """A model's laser on its mode at 61 mA, its current last moved up or down, then left alone while its chip's
    modes drift (sign): GHz they drifted when it hops."""
    model, t = trials.lab650(), [0.0]
    las = ModelLaser(model, clock=lambda: t[0], seed=1, sided=sided)
    for di in [0.01 * k for k in range(40, -1, -1)] + ([-0.02, -0.01, 0.0] if last_up else [0.02, 0.01, 0.0]):
        t[0] += 5.0
        las.lase(model.v0, 61.0 + di)
    order, moved = las._state[0], 0.0
    while las._state[0] == order and moved < 20.0:
        model.offset += sign * 0.002
        moved += 0.002
        t[0] += 5.0
        las.lase(model.v0, 61.0)
    return moved


def test_sided_its_hysteresis_is_the_edges_not_its_last_moves():
    pytest.importorskip("numpy")
    # as its map has it (sweeps): whether it holds goes by which way its current last moved, even standing still
    assert drift_until_it_hops(False, True, 1) < 0.6 * drift_until_it_hops(False, False, 1)
    # sided: by the edge its chip's modes drift it to, whichever way its current moved last
    for sign in (1, -1):
        assert drift_until_it_hops(True, True, sign) == pytest.approx(drift_until_it_hops(True, False, sign))
    # ...each edge with the hysteresis a sweep that way would have: the same as unsided, come the matching way
    assert drift_until_it_hops(True, True, 1) == pytest.approx(drift_until_it_hops(False, False, 1))
    assert drift_until_it_hops(True, True, -1) == pytest.approx(drift_until_it_hops(False, True, -1))


def test_the_ccds_amplitudes_are_noisy_if_asked():
    comps = [(1.0, 0.0)]
    still, ccd = SimCCD(seed=1), SimCCD(seed=1, amp_noise=5.0)
    quiet = [still.measure(comps, 20.0, 150.0, 473.0)[1]["amps"] for _ in range(30)]
    noisy = [ccd.measure(comps, 20.0, 150.0, 473.0)[1]["amps"] for _ in range(30)]
    assert len({tuple(a) for a in quiet[5:]}) == 1  # (its exposure settled: the same each time)
    assert len({tuple(a) for a in noisy[5:]}) > 5 and all(abs(a[1] - quiet[5][1]) < 40 for a in noisy[5:])


# ------------------------------------------------------------------ the trials
def test_recoveries_in_numbers():
    outs = recoveries("far", 8, first=3, smooth=True)
    s = summary(outs)
    assert s["n"] == 8 and s["feasible"] >= 6 and s["done_feasible"] == s["feasible"]
    assert s["said_done_off_target"] == 0 and s["second_mode_over_10pc"] == 0
    assert 10.0 < s["s_median"] < 30.0 and s["s_median"] <= s["s_p90"] <= s["s_max"]
    assert s["margin_median"] > 0.3 and s["readings_median"] > 50
    line = trials.said("far", s)
    assert line.startswith("far        8 lasers:") and "s median" in line and "SAID DONE" not in line
    assert trials.said("far", summary([])) .endswith("none recovered")
    o = outs[0]
    assert o.kind == "far" and o.seed == 3 and o.ops > o.readings and o.method == "map" and o.notes
    stepped = summary(recoveries("far", 8, first=3))  # (as it comes: by steps)
    assert stepped["done_feasible"] == stepped["feasible"] and stepped["s_median"] > 1.3 * s["s_median"]


@pytest.mark.parametrize("kind,least", [("red", 5), ("blue", 5), ("model", 4)])
def test_a_models_lasers_come_back_mostly_by_the_short_way(kind, least):
    pytest.importorskip("numpy")
    pytest.importorskip("scipy")
    outs = recoveries(kind, 6, smooth=True)
    s = summary(outs)
    assert s["done"] >= least and s["said_done_off_target"] == 0
    short = [o for o in outs if "short way" in o.summary]
    assert len(short) >= 3 and statistics.median(o.seconds for o in short) < 15.0
    assert all(o.margin >= 0.1 for o in short)


def test_the_command_line(tmp_path, capsys):
    out = tmp_path / "far.json"
    assert trials.main(["recover", "far", "--n", "3", "--first", "3", "--json", str(out), "-v"]) == 0
    text = capsys.readouterr().out
    assert text.count("far 3:") == 1 and "far        3 lasers:" in text
    data = json.loads(out.read_text())
    assert data["summary"]["far"]["n"] == 3 and len(data["outcomes"]["far"]) == 3 and data["set"] == {}
    assert trials.main(["recover", "far", "--n", "2", "--first", "3", "--set", "smooth=true", "--set", "fine=0.05",
                        "--json", str(out)]) == 0
    assert json.loads(out.read_text())["set"] == {"smooth": True, "fine": 0.05}
    capsys.readouterr()
    assert trials.main(["lasers", "far", "--n", "2"]) == 0
    text = capsys.readouterr().out
    assert text.count("GHz apart") == 2 and "hopped" in text
    kept = tmp_path / "keep.json"
    assert trials.main(["keep", "shg", "--n", "1", "--first", "1", "--hours", "1", "--json", str(kept)]) == 0
    text = capsys.readouterr().out
    assert "shg   left alone 1 lasers, 1 h each: 1 hopped" in text and "shg   kept       1 lasers, 1 h each: 0 hopped" in text
    data = json.loads(kept.read_text())
    assert data["shg alone"][0]["hop"] < 1.0 and data["shg kept"][0]["hop"] is None and data["shg kept"][0]["kept"] >= 1
    with pytest.raises(SystemExit):
        trials.main(["recover", "green"])
