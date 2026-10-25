"""The laser's model (lasermodel.py) fitted to maps of simulated lasers: one made of a model (sim.ModelLaser, a chip
with modes of its own: what it was made of comes back) and the simulator's ECDL (one selection filter); its facts,
its file, the command line, the engine's fit after a map."""
import threading
import types

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from test_lasermap import MapBench, plan  # noqa: E402

from wmlock import lasermodel as lm  # noqa: E402
from wmlock.engine import Channel, Engine  # noqa: E402
from wmlock.lasermap import MapFile, Mapper, load, main  # noqa: E402
from wmlock.sim import ECDL, ModelLaser  # noqa: E402


def lab_like(**kw) -> lm.LaserModel:
    """A laser like the lab's 493 nm DL pro, as its map's model has it (rounded): a chip whose own modes, 87.35 GHz
    apart, decide which of the external cavity's (8.26 GHz apart) lases."""
    model = lm.LaserModel(
        f_ref=607.42, i0=65.0, v0=69.0, t0=0.0,
        ext=lm.Comb(8.26, p0=0.0, p_i=-1.71, p_v=1.40, p_vv=0.008),
        chip=lm.Comb(87.35, p0=3.0, p_i=-4.18, p_v=-0.12),
        lo=-90.0, hi=180.0, h_up=0.0005, h_down=0.007, offset=0.8, pull=0.35, lag=0.2,
        i_th=28.0, slope=0.5, power_dp=-0.35)
    for k, v in kw.items():
        setattr(model, k, v)
    return model


def mapped(tmp_path, laser_of, p, **bench):
    """A map of a laser (laser_of(clock) makes it, on the bench's time), run to its end: its file's path."""
    b = None
    laser = laser_of(lambda: b.t if b is not None else 0.0)
    b = MapBench(laser=laser, patterns=False, **bench)
    path = tmp_path / "map.sqlite"
    out = MapFile(path)
    b.run(Mapper(p, out))
    out.close()
    return path


@pytest.fixture(scope="module")
def lab_map(tmp_path_factory):
    path = mapped(tmp_path_factory.mktemp("lab"), lambda clock: ModelLaser(lab_like(), clock=clock),
                  plan(i_lo=55.0, i_hi=75.0, i_step=0.25, v_lo=49.0, v_hi=89.0, v_step=2.0, patterns="off"),
                  current=65.0, offset=69.0)
    with load(path) as m:
        return path, lm.fit(m, budget=40)


def test_a_laser_made_of_a_model_comes_back_from_its_map(lab_map):
    path, got = lab_map
    want = lab_like()
    assert got.ext.spacing == pytest.approx(want.ext.spacing, rel=2e-3)
    assert got.chip.spacing == pytest.approx(want.chip.spacing, rel=5e-3)
    assert got.ext.p_i == pytest.approx(want.ext.p_i, rel=0.05)
    assert got.chip.p_i == pytest.approx(want.chip.p_i, rel=0.05)
    pace = got.pace / lm.UNIT_T  # (the piezo as mapped: its drift with it, none here)
    assert got.ext.p_v + got.ext.p_t * pace == pytest.approx(want.ext.p_v + 2 * want.ext.p_vv * (got.v0 - want.v0),
                                                             rel=0.05)
    assert abs(got.chip.p_v) < 0.15 * got.ext.p_v
    assert got.ideal_feedforward() == pytest.approx(want.ideal_feedforward(), rel=0.1)
    shift = (want.f_ref - got.f_ref) * 1e3  # (its frequencies are from the map's first reading)
    assert got.lo == pytest.approx(want.lo + shift, abs=15.0) and got.hi == pytest.approx(want.hi + shift, abs=15.0)
    assert got.right > 0.8 and got.pull > 0.1  # (a mode, once lasing, held: as it was made)
    assert got.near > 0.8  # (the rest a point or two from where it hopped)
    assert got.i_th == pytest.approx(28.0, abs=3.0) and got.slope == pytest.approx(0.5, rel=0.05)  # (where its
    #   modes sit on their resonances on the whole moves its power: the threshold, as a line through it, with it)
    assert got.power_dp == pytest.approx(-0.35, rel=0.25)
    assert got.hysteresis > 0 and 0 < got.apart <= 1


def test_its_facts_say_the_laser_in_numbers(lab_map):
    _, got = lab_map
    facts = got.facts()
    text = "\n".join(facts)
    assert facts[0].startswith(f"its external cavity's modes are {got.ext.spacing:.4f} GHz apart: an optical length "
                               f"of 18.1")  # (c / 2 / 8.26 GHz)
    assert "its chip's own modes are 87." in text and "anti-reflection coating is poor" in text
    assert "the ideal feedforward: " in text and " mA/V keeps the combs aligned" in text
    assert "the piezo moves the external modes +1.4" in text and "nearer the chip per volt" in text
    assert "it lases only within " in text and "pulls its chip resonance" in text
    assert "on a mode its power changes -0.3" in text
    assert "the opposite of what the map shows" in text  # (less light, more heat: that would push a mode on)
    assert "the model puts " in text
    assert got.alt is not None and got.alt["right"] > 0.5 and set(got.alt) >= set(lm.RULE + lm.WINDOW) | {"dc", "ci"}
    near = got.alt["right"] >= got.right - 0.05  # (the map tells the two apart only so far, or it does)
    assert (f"another rule does {'nearly as well' if near else 'less well'}: its pull " in text
            and ("so its recovery plans for both" in text) == near)


def test_misread_frequencies_by_the_dozen_do_not_mislead_it(tmp_path, lab_map):
    import sqlite3

    path, got = lab_map
    copy = tmp_path / "misread.sqlite"
    copy.write_bytes(path.read_bytes())
    db = sqlite3.connect(str(copy))  # (one point in 20 read 4.9 THz off, one in 33 -2.45 THz: its order misread)
    db.execute("UPDATE points SET f = f + 4.9 WHERE f IS NOT NULL AND id % 20 = 0")
    db.execute("UPDATE points SET f = f - 2.45 WHERE f IS NOT NULL AND id % 33 = 1")
    db.commit()
    db.close()
    with load(copy) as m:  # (its very first reading among them: the frequencies' reference is not)
        d = lm.data_of(m)
        assert d.f_ref == pytest.approx(got.f_ref, abs=0.01)  # (THz: the middle of its frequencies, not 2.45 THz off)
        assert not d.ok[np.abs(d.f) > 1000].any() and d.ok.sum() > 0.85 * len(d.f)  # (left out, the rest kept)
        again = lm.fit(m, budget=2)
    assert again.ext.spacing == pytest.approx(got.ext.spacing, rel=2e-3)
    assert again.chip.spacing == pytest.approx(got.chip.spacing, rel=5e-3)
    assert again.plausible and again.hops_on > 0.9 and again.right > again.null + 0.2


def test_the_hops_tell_its_comb_from_a_coarser_one(lab_map):
    path, got = lab_map
    with load(path) as m:
        d = lm.data_of(m)
    assert lm.hops_on(d, got.ext.spacing) > 0.9  # (its hops are whole numbers of its external modes...)
    coarse = 4 * got.ext.spacing  # (...not of a comb as coarse, its single hops within its modes, few others near)
    assert lm.hops_on(d, coarse, tol=0.25 * got.ext.spacing) < 0.3


def test_a_model_no_laser_could_be_is_said_so_and_not_gone_by():
    bad = lab_like(ext=lm.Comb(108.07, p_i=-0.375, p_v=0.685), chip=lm.Comb(493.97, p_i=-0.377, p_v=0.685),
                   right=0.9999, null=0.999, hops_on=0.02)  # (the 650 nm laser, as misreads once had its model)
    doubts = bad.doubts()
    assert not bad.plausible and len(doubts) == 4
    assert "would make its cavity 1.39 mm long (optically): no ECDL's is" in doubts[0]
    assert "would make its chip 0.303 mm long (optically, n_g L): no laser diode's is" in doubts[1]
    assert doubts[2].startswith("only 2% of its map's hops are whole numbers of its external modes")
    assert doubts[3].startswith("it puts no more of the map's points on the right mode (100%) than one that never")
    assert bad.facts()[0].startswith("NOT THIS LASER'S MODEL: its map misled its fit")
    good = lab_like(right=0.8, null=0.3, hops_on=0.97)
    assert good.plausible and not good.facts()[0].startswith("NOT")


def test_a_drift_the_map_cannot_tell_is_said_so():
    e = lm.Comb(8.26, p_v=1.28, p_t=1.8, drift_known=True,
                others=[{"p0": 0.0, "p_v": 1.48, "p_vv": 0.0, "p_t": -2.6, "p_leg": 0.0}])
    model = lab_like(ext=e, pace=27.0, again_s=1140.0)
    text = "\n".join(model.facts())
    assert "how much its cavity drifted, this map cannot tell" in text and "+1.80 GHz per 10 min" in text
    assert "-2.60" in text and "19 min later" in text
    assert "(or +1.480: the drift is not known, below)" in text
    assert "+1.361 GHz/V as the map went" in text  # (1.28 + 1.8 GHz per 10 min, 27 s a volt)


def test_kept_in_its_file_and_read_back(tmp_path, lab_map):
    path, got = lab_map
    copy = tmp_path / "copy.sqlite"
    copy.write_bytes(path.read_bytes())
    lm.save(copy, got)
    with load(copy) as m:
        back = lm.stored(m)
        assert back == got and m.meta["model_facts"] == got.facts()
    with load(path) as m:
        assert lm.stored(m) is None


def test_a_step_at_a_time_lases_as_the_whole_path_does():
    model = lab_like()
    i = [60.0 + 0.2 * k for k in range(60)] + [72.0 - 0.2 * k for k in range(60)]
    t = [0.25 * k for k in range(len(i))]
    v = [69.0] * len(i)
    whole = model.simulate(i, v, t)
    state, steps = None, []
    for a, b, c in zip(i, v, t):
        ghz, _, state = model.step(a, b, c, state)
        steps.append(ghz)
    assert steps == pytest.approx(list(whole), abs=1e-9)
    assert len(set(round(x) for x in whole)) > 5  # (it hops, up and down)


def test_the_simulators_ecdl_has_one_selection_filter(tmp_path):
    laser = ECDL(384230.0, seed=0)  # (fsr 6 GHz, mode -0.25 GHz/mA, +0.4 GHz/V; its filter -2 GHz/mA, +0.32 GHz/V)
    path = mapped(tmp_path, lambda clock: laser, plan(repeat=False))
    with load(path) as m:
        got = lm.fit(m, budget=15)
    assert got.single and got.chip.spacing == 0
    assert got.ext.spacing == pytest.approx(6.0, rel=5e-3)
    assert got.ext.p_i == pytest.approx(-0.25, rel=0.1) and got.ext.p_v == pytest.approx(0.4, rel=0.1)
    assert got.chip.p_i == pytest.approx(-2.0, rel=0.1) and got.chip.p_v == pytest.approx(0.32, rel=0.2)
    assert got.ideal_feedforward() == pytest.approx((0.4 - 0.32) / (-2.0 + 0.25), abs=0.02)
    assert got.right > 0.8
    text = "\n".join(got.facts())
    assert "one selection filter" in text and "its drift is not told apart" in text


def test_the_command_line_fits_it_and_keeps_it_if_asked(tmp_path, capsys, lab_map, monkeypatch):
    monkeypatch.setattr(lm, "BUDGET", 4)  # (quick, coarse: what it says, not how well)
    path, _ = lab_map
    copy = tmp_path / "copy.sqlite"
    copy.write_bytes(path.read_bytes())
    assert main([str(copy), "--model"]) == 0
    out = capsys.readouterr().out
    assert "the laser, as its model has it:" in out and "external cavity's modes are 8.2" in out
    with load(copy) as m:
        assert "model" not in m.meta  # (not kept unless asked)
    assert main([str(copy), "--model", "--save-model"]) == 0
    with load(copy) as m:
        assert lm.stored(m) is not None
    capsys.readouterr()
    assert main([str(copy)]) == 0
    assert "the laser, as its model has it (kept in the file):" in capsys.readouterr().out


def test_the_engine_fits_it_once_the_map_is_written(tmp_path, lab_map, monkeypatch):
    monkeypatch.setattr(lm, "BUDGET", 4)
    path, _ = lab_map
    copy = tmp_path / "copy.sqlite"
    copy.write_bytes(path.read_bytes())
    ch = Channel(2, {"link": {"device": "192.168.1.30", "laser": 1}})
    eng = types.SimpleNamespace(lock=threading.Lock(), dirty=False, MODEL_GOOD=0.0)
    m = Mapper(plan(), info={"laser": {"device": "192.168.1.30", "laser": 1}})
    done = threading.Thread(target=lambda: None)
    done.start()
    Engine._model_of(eng, ch, m, copy, done)
    assert m.model is not None and m.model_note.startswith("its model: right mode at ")
    assert m.notes[0] == (None, "the laser, as its model has it:") and len(m.notes) == 2 + len(m.model.facts())
    assert m.notes[-1][1].startswith("its recoveries go by it from now on")
    assert ch.laser_model() == m.model and ch.model_dict["from"] == "copy.sqlite" and eng.dirty  # (kept with it)
    with load(copy) as f:
        assert lm.stored(f) == m.model
    other = Channel(3, {"link": {"device": "192.168.1.31", "laser": 1}})  # (linked to another laser by now)
    m3 = Mapper(plan(), info={"laser": {"device": "192.168.1.30", "laser": 1}})
    Engine._model_of(eng, other, m3, copy, done)
    assert m3.model is not None and other.laser_model() is None
    poor = types.SimpleNamespace(lock=threading.Lock(), dirty=False, MODEL_GOOD=1.01)
    ch4 = Channel(4, {"link": {"device": "192.168.1.30", "laser": 1}})
    m4 = Mapper(plan(), info={"laser": {"device": "192.168.1.30", "laser": 1}})
    Engine._model_of(poor, ch4, m4, copy, done)
    assert ch4.laser_model() is None and "its recoveries do not go by it" in m4.notes[-1][1]
    bad = tmp_path / "bad.sqlite"
    MapFile(bad).close()  # (an empty map)
    m2 = Mapper(plan())
    Engine._model_of(eng, ch, m2, bad, done)
    assert m2.model is None and m2.model_note.startswith("no model of the laser: ")
    assert m2.notes == [(None, m2.model_note)]
