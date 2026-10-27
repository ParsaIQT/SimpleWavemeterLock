"""Mode-hop recovery by the laser's own model (modelhop.py, Recenter's method 'model'): the planner anchored where the
laser was on target, fitted to what it did, planning a way onto a mode on target; the recovery by it, against the lab's
two lasers as their maps' models have them (sim.ModelLaser), their rules a little off what the model says and their
chips' modes moved until they hopped; a model that does not describe the laser not gone by; the method chosen; the
engine keeping a channel's model from its map."""
import copy
import threading
import types

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from test_modehop import Bench  # noqa: E402

from wmlock import lasermodel as lm  # noqa: E402
from wmlock.engine import Channel, Engine  # noqa: E402
from wmlock.modehop import HopParams, Recenter  # noqa: E402
from wmlock.modelhop import AVOID_V, WINDOW_IN, Planner, _apply, _grid  # noqa: E402
from wmlock.sim import ModelLaser  # noqa: E402


def blue(**kw) -> lm.LaserModel:
    """The lab's 493 nm DL pro as its map's model has it (rounded): its chip's own modes, 87.4 GHz apart, decide which
    of its external modes (8.26 GHz apart) lases, within a window 270 GHz wide; and the rule held the other way."""
    model = lm.LaserModel(
        f_ref=607.4237, i0=64.40, v0=67.16, t0=0.0,
        ext=lm.Comb(8.2619, p0=24.795, p_i=-1.7114, p_ii=-0.00285, p_v=1.2845, p_vv=0.00882, p_leg=-0.2757),
        chip=lm.Comb(87.379, p0=7.521, p_i=-4.186, p_ii=-0.0073, p_v=-0.1174, p_vv=-0.0005),
        lo=-88.76, hi=180.62, h_up=0.0006, h_down=0.0065, offset=0.712, pull=0.40, lag=0.298, w_v=0.2516,
        w_i=-0.1805, i_th=28.24, slope=0.512, power_dp=-0.57, right=0.77, step_s=0.252, step_x=0.2,
        i_span=[43.0, 87.0], v_span=[49.16, 89.16],
        alt={"lo": -88.48, "hi": 179.80, "h_up": 0.0001, "h_down": 0.0047, "offset": 0.606, "pull": 0.233,
             "lag": 0.537, "dc": 87.143, "ci": -4.186, "w_v": 0.607, "w_i": -0.724, "right": 0.767})
    for k, v in kw.items():
        setattr(model, k, v)
    return model


def red(**kw) -> lm.LaserModel:
    """The lab's 650 nm DL pro likewise: chip modes 18.4 GHz apart, external modes 3.39 GHz apart, a window only 27 GHz
    wide that its piezo moves nearly with its modes."""
    model = lm.LaserModel(
        f_ref=461.2938, i0=61.39, v0=66.99, t0=0.0,
        ext=lm.Comb(3.3886, p0=11.185, p_i=-0.2486, p_ii=0.00032, p_v=0.4018, p_vv=0.002, p_leg=-0.1196),
        chip=lm.Comb(18.373, p0=-5.236, p_i=-1.3615, p_ii=-0.0014, p_v=0.0149, p_vv=0.00013),
        lo=-9.06, hi=17.73, h_up=0.0203, h_down=0.0404, offset=0.086, pull=0.078, lag=0.041, w_v=0.6524,
        w_i=-0.1526, i_th=49.2, slope=75.4, power_dp=-22.2, right=0.85, step_s=0.084, step_x=0.2024,
        i_span=[52.9, 70.0], v_span=[36.8, 97.1], v_set_to_act=-0.5065,
        alt={"lo": -8.84, "hi": 18.85, "h_up": 0.0071, "h_down": 0.0141, "offset": -0.048, "pull": 0.154,
             "lag": 0.094, "dc": 18.431, "ci": -1.3833, "w_v": 0.6696, "w_i": -0.4538, "right": 0.8375})
    for k, v in kw.items():
        setattr(model, k, v)
    return model


def off(model: lm.LaserModel, rng) -> lm.LaserModel:
    """A laser like the model, not it: its rule's terms a little off (as another fit of its map would have them), its
    chip's modes anywhere, its cavity drifted."""
    true = copy.deepcopy(model)
    true.h_up *= float(np.exp(rng.normal(0, 0.4)))
    true.h_down *= float(np.exp(rng.normal(0, 0.4)))
    true.pull = float(np.clip(true.pull + rng.normal(0, 0.05), -0.4, 0.8))
    true.lag *= float(np.exp(rng.normal(0, 0.3)))
    w = true.hi - true.lo
    true.lo += float(rng.normal(0, 0.03 * w))
    true.hi += float(rng.normal(0, 0.03 * w))
    true.w_v *= float(1 + rng.normal(0, 0.15))
    true.w_i *= float(1 + rng.normal(0, 0.15))
    true.offset += float(rng.uniform(-0.5, 0.5) * true.dc)
    true.ext.p0 += float(rng.uniform(-5, 5))
    return true


def hopped(model: lm.LaserModel, seed: int, true=None, bench_of=None):
    """The laser (off the model, by seed) on target where it lases coming down onto its nominal current, then its
    chip's modes moving a little at a time until it hops: (bench, laser, its HopParams and Recenter's arguments).
    bench_of: the bench's class (Bench)."""
    rng = np.random.default_rng(seed)
    true = off(model, rng) if true is None else true
    i_nom = round(float(model.i0 + rng.uniform(-1, 1)), 2)
    v_nom = round(float(model.v0 - model.v_set_to_act + rng.uniform(-3, 3)), 2)
    bench = None
    laser = ModelLaser(true, clock=lambda: bench.t if bench is not None else 0.0)
    bench = (bench_of or Bench)(laser, 0.0, current=i_nom + 2.0, offset=v_nom, dt=0.15)
    for k in range(21):  # (onto it from 2 mA above, as by hand)
        bench.client.set("laser1:dl:cc:current-set", i_nom + 2.0 - 0.1 * k)
        bench.t += 0.15
    bench.target = target = laser.freq(v_nom, i_nom)
    for _ in range(20000):
        if abs(laser.freq(v_nom, i_nom) - target) * 1e3 >= 0.4 * true.ext.spacing:
            break
        true.offset += 0.0004 * true.dc
        bench.t += 0.15
        bench._settle(None)
    p = HopParams(enabled=True, method="model", i_nom=i_nom, i_below=1.0, i_window=2.0, v_window=25.0)
    return bench, laser, p, (i_nom, v_nom, target)


def recover(model: lm.LaserModel, seed: int, true=None, given=None):
    bench, laser, p, (i_nom, v_nom, target) = hopped(model, seed, true)
    hop = (laser.freq(v_nom, i_nom) - target) * 1e3
    rec = Recenter(p, i_nom, v_nom, i_nom - 1.0, i_nom + 2.0, v_nom - 25.0, v_nom + 25.0, ref=(v_nom, i_nom),
                   model=model if given is None else given, target=target)
    r = bench.run(rec)
    return r, rec, hop, (laser.freq(bench.V, bench.act) - target) * 1e6


def lase(true, path, v, dt, state=None, t=0.0):
    """GHz the model laser lases at along a path of currents at piezo v (as its map's model has it): (GHz, state, s)."""
    out = []
    for i in path:
        t += dt
        f, _, state = true.step(float(i), float(v), true.t0, state, clock=t)
        out.append(f)
    return np.array(out), state, t


def test_anchored_where_it_was_on_target_its_right_mode_is_on_target_there():
    model = blue()
    target = model.f_ref + 0.050  # (50 GHz above its map's first reading)
    pl = Planner(model, target, ref=(64.0, 68.0))
    assert pl.ghz(pl.right, 64.0, 68.0) == pytest.approx(0.0, abs=1e-9)
    assert pl.current_for(pl.right, 68.0, 65.0) == pytest.approx(64.0, abs=1e-6)
    assert pl.current_for(pl.right + 1, 68.0, 64.0) == pytest.approx(64.0 + 8.2619 / 1.7114, abs=0.1)  # (a mode on)
    assert pl.kv(64.0, 68.0) == pytest.approx(1e3 * (1.2845 + 2 * 0.00882 * 0.84), rel=1e-6)  # MHz/V
    assert pl.slope(64.0, 68.0) == pytest.approx(1e3 * (-1.7114 - 2 * 0.00285 * -0.4), rel=1e-6)  # MHz/mA
    assert pl.slid == 0.0  # (the target well inside where it lases)
    edge = Planner(model, model.f_ref + 0.190, ref=(64.0, 68.0))  # (beyond its window's top as the model has it)
    lo, hi = edge._window(64.0, 68.0)
    assert edge.slid > 0 and float(hi) == pytest.approx(WINDOW_IN) and float(lo) < -200  # (moved to have it in)


def test_fitted_to_what_it_did_it_finds_where_its_chip_modes_and_cavity_went():
    model = blue()
    i, v = 64.2, 67.5
    f, state, t = lase(model, _grid(i + 2.0, i, 0.1), v, 0.15)  # (on target: as the model has it, unmoved)
    target = model.f_ref + f[-1] / 1e3
    true = copy.deepcopy(model)
    true.offset += 20.0  # GHz: its chip's modes since the map
    true.ext.p0 += 1.3   # GHz: its cavity, since it was on target
    pl = Planner(model, target, ref=(i, v))
    _, state, t = lase(true, _grid(i + 2.0, i - 1.0, 0.2), v, 0.05)
    for a, b, up in ((i - 1.0, i + 2.0, True), (i + 2.0, i - 1.0, False)):  # (a map here: up, then down)
        seg, path = pl.segment(), _grid(a, b, 0.1)
        ghz, state, t0 = lase(true, path, v, 0.25, state, t)
        for k, (x, g) in enumerate(zip(path, ghz)):
            pl.observe(seg, t + 0.25 * (k + 1), x, v, up, g - f[-1])
        t = t0
    assert pl.fit() > 0.9 and pl.agree > pl.null  # (it hops, and the model says where)
    assert pl.drift == pytest.approx(1.3, abs=0.15)
    d = pl.ensemble[1][1] - pl.phi  # (how far either way its chip's modes may be, as well as the readings say)
    assert abs((pl.phi - 20.0 + 0.5 * model.dc) % model.dc - 0.5 * model.dc) <= d + 1.0
    assert pl.on_comb > 0.95


def test_its_plan_lands_the_laser_on_a_mode_on_target_and_it_holds():
    for make in (blue, red):
        model = make()
        i, v = model.i0, model.v0 - model.v_set_to_act
        f, _, _ = lase(model, _grid(i + 2.0, i, 0.1), v + model.v_set_to_act, 0.15)
        pl = Planner(model, model.f_ref + f[-1] / 1e3, ref=(i, v))
        plan = pl.plan(i - 1.0, i + 2.0, v - 10.0, v + 10.0, 0.1, i, v, step_s=0.25)
        assert plan is not None and plan.share > 0.5 and min(plan.ahead, plan.behind) >= 0.5
        assert plan.path[-1][0] == pytest.approx(plan.i) and abs(plan.path[-1][1]) < 0.05  # (its mode on target)
        way, ts, _, _ = pl._way(plan.i, plan.up, i - 1.0, i + 2.0, 0.1, 0.25)
        state, ghz = None, []  # (the laser as the model has it: the way there, a step at a time)
        for x, tt in zip(way, ts):
            g, _, state = model.step(float(x), plan.v + model.v_set_to_act, model.t0, state, clock=tt)
            ghz.append(g)
        there = ghz[-1] - f[-1]
        assert abs(there) < 0.3 * model.ext.spacing  # (on that mode, on target: the piezo does the rest)
        for k, x in enumerate(_grid(plan.i, plan.i + (0.5 if plan.up else -0.5), 0.02)):  # (on, as far as asked)
            g, _, state = model.step(float(x), plan.v + model.v_set_to_act, model.t0, state, clock=ts[-1] + 0.25 * k)
            assert abs((g - f[-1]) - there) < 0.3 * model.ext.spacing + 0.01


def test_where_a_plan_did_not_hold_it_plans_elsewhere():
    model = blue()
    i, v = 64.4, 67.2
    f, _, _ = lase(model, _grid(i + 2.0, i, 0.1), v, 0.15)
    pl = Planner(model, model.f_ref + f[-1] / 1e3, ref=(i, v))
    first = pl.plan(i - 1.0, i + 2.0, v - 10.0, v + 10.0, 0.1, i, v)
    other = pl.plan(i - 1.0, i + 2.0, v - 10.0, v + 10.0, 0.1, i, v, avoid=[(first.v, first.i, first.up)])
    assert other is not None
    assert abs(other.v - first.v) >= AVOID_V or abs(other.i - first.i) >= 0.5 or other.up != first.up


@pytest.mark.parametrize("seed", [300, 303, 307])
def test_the_493nm_laser_hopped_comes_back_on_target_by_its_model(seed):
    r, rec, hop, e = recover(blue(), seed)
    assert abs(hop) > 3.0  # GHz (it hopped)
    assert r.state == "done" and abs(e) < 20.0  # MHz
    m = rec.record["model"]
    assert m["agree"] > 0.6 and (m.get("done") or m.get("band")) and rec.maps == 1  # (one map: the model's fit; by a
    #                                                                                  plan, or its right mode's band)


@pytest.mark.parametrize("seed", [300, 304, 309])
def test_the_650nm_laser_hopped_comes_back_on_target_by_its_model(seed):
    r, rec, hop, e = recover(red(), seed)
    assert abs(hop) > 1.0
    assert r.state == "done" and abs(e) < 20.0
    m = rec.record["model"]
    assert (m.get("done") or m.get("band")) and rec.maps == 1


@pytest.mark.parametrize("seed", [300, 304])
def test_a_model_of_several_temperatures_goes_by_the_diodes_own(seed):
    model = red(T0=19.75, temp=19.75, temps=[19.0, 19.5, 20.0, 20.5], w_T=-1.8)  # (as its temperature map had it)
    model.chip.p_T, model.ext.p_T = -26.9, -7.35
    true = off(model, np.random.default_rng(seed)).at(20.0)  # (its diode at 20 °C, where its controller has it)
    r, rec, hop, e = recover(model, seed, true=true)
    assert abs(hop) > 1.0 and r.state == "done" and abs(e) < 20.0
    assert any(n.startswith("its model at 20.000 °C, the diode's temperature as set (its map's: 19, 19.5, 20, 20.5")
               for n in rec.notes)


class Thinking(Bench):
    """A bench whose io works things out on the side (as the engine's does, in a thread of its own): each answer a
    second later, readings going by meanwhile."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.thoughts, self.answered = [], 0

    def think(self, fn, done, failed):
        self.thoughts.append((self.t + 1.0, fn, done))

    def reading(self):
        out = super().reading()
        for th in [th for th in self.thoughts if th[0] <= self.t]:
            self.thoughts.remove(th)
            self.answered += 1
            self.pending.append(lambda th=th: th[2](th[1]()))
        return out


def test_its_model_is_worked_out_on_the_side_and_a_stop_meanwhile_puts_things_back():
    def rec(p, i_nom, v_nom, target):
        return Recenter(p, i_nom, v_nom, i_nom - 1.0, i_nom + 2.0, v_nom - 25.0, v_nom + 25.0, ref=(v_nom, i_nom),
                        model=blue(), target=target)

    b, laser, p, (i_nom, v_nom, target) = hopped(blue(), 300, bench_of=Thinking)
    r = b.run(rec(p, i_nom, v_nom, target))
    assert r.state == "done" and abs(laser.freq(b.V, b.act) - target) * 1e6 < 20.0
    assert b.answered >= 2 and not b.thoughts  # (fitted, planned: each while readings went by)
    b, laser, p, (i_nom, v_nom, target) = hopped(blue(), 300, bench_of=Thinking)
    began = (b.I, b.V)
    r = b.run(rec(p, i_nom, v_nom, target), until=lambda r: r._waiting == "think")
    assert r.busy
    r.cancel("stopped")
    while r.busy:
        if b.pending:
            b.pending.pop(0)()
        else:
            r.on_reading(*b.reading())
    for _, fn, done in b.thoughts:  # (the answer, too late)
        done(fn())
    assert r.state == "failed" and r.error == "stopped" and r.restored
    assert (b.I, b.V) == pytest.approx(began)  # (back where it began)


def test_a_model_that_does_not_describe_the_laser_is_not_gone_by():
    model = blue()
    other = blue(ext=lm.Comb(11.0, p0=3.0, p_v=1.28))  # (another laser's: its modes not where this one's are)
    r, rec, hop, e = recover(model, 300, given=other)
    m = rec.record["model"]
    assert not m["plans"] and any("its model fits this map's readings badly" in n for n in rec.notes)
    assert any("of them on its external modes" in n for n in rec.notes)  # (and how badly)


def test_auto_goes_by_its_model_without_an_shg_stage():
    assert HopParams().how(shg=False, model=True) == "model"
    assert HopParams().how(shg=True, model=True) == "catch"
    assert HopParams().how(shg=False, model=False) == "map"
    assert HopParams(method="model").how(shg=True, model=True) == "model"
    assert HopParams(method="model").how(shg=False, model=False) == "map"  # (none to go by)
    assert HopParams(method="map").how(shg=False, model=True) == "map"
    rec = Recenter(HopParams(enabled=True), 60.0, 70.0, 59.0, 62.0, 45.0, 95.0, model=blue(), target=607.5)
    assert rec.method == "model"
    rec = Recenter(HopParams(enabled=True), 60.0, 70.0, 59.0, 62.0, 45.0, 95.0, model=blue())  # (no target)
    assert rec.method == "map"


def test_a_channel_keeps_its_lasers_model_for_its_recoveries():
    ch = Channel(3, {"link": {"device": "192.168.1.30", "laser": 1}})
    assert ch.laser_model() is None
    model = blue()
    eng = types.SimpleNamespace(lock=threading.Lock(), dirty=False, MODEL_GOOD=Engine.MODEL_GOOD)
    ch.adopt_model(model, "map 1.sqlite")
    assert ch.laser_model() is model
    back = Channel(3, ch.to_config())
    got = back.laser_model()
    assert got == model and got.alt == model.alt
    assert back.model_dict["from"] == "map 1.sqlite" and back.model_dict["right"] == 0.77
    back.laser = 2  # (linked to another laser now: not its model)
    assert back.laser_model() is None
    assert not eng.dirty


def test_the_engine_works_a_recoverys_sums_out_on_the_side():
    from wmlock.engine import _HopIO

    eng = types.SimpleNamespace(lock=threading.Lock())
    io = _HopIO(eng, Channel(4, {"link": {"device": "192.168.1.30", "laser": 1}}), controller=None)
    got, done = [], threading.Event()

    def answer(result):
        got.append((result, eng.lock.locked(), threading.current_thread() is not threading.main_thread()))
        done.set()

    with eng.lock:  # (the engine's lock held where the recovery asks: the sums go on without it)
        io.think(lambda: sum(range(10)), answer, lambda exc: None)
    assert done.wait(5.0) and got == [(45, True, True)]  # (answered under the engine's lock, from its own thread)
    failed, done = [], threading.Event()

    def bad():
        raise ValueError("no")

    io.think(bad, lambda result: None, lambda exc: (failed.append((str(exc), eng.lock.locked())), done.set()))
    assert done.wait(5.0) and failed == [("no", True)]


def test_a_channel_takes_its_lasers_model_from_a_map_made_before(tmp_path):
    import time

    from wmlock.lasermap import MapFile

    def a_map(name, device, model=None):
        path = tmp_path / name
        f = MapFile(path)
        f.meta("laser", {"channel": 3, "name": "493", "device": device, "laser": 1})
        f.close()
        if model is not None:
            lm.save(path, model)
        return path

    ch = Channel(3, {"link": {"device": "192.168.1.30", "laser": 1}})
    eng = types.SimpleNamespace(lock=threading.Lock(), channels={3: ch}, MODEL_GOOD=Engine.MODEL_GOOD, dirty=False)
    eng._take_model = types.MethodType(Engine._take_model, eng)
    model = blue()
    said = Engine.model_from_file(eng, 3, a_map("mine.sqlite", "192.168.1.30", model))
    assert said.startswith("from mine.sqlite, 77% of that map's points on the right mode: its recoveries go by it")
    assert ch.laser_model() == model and ch.model_dict["from"] == "mine.sqlite" and eng.dirty
    said = Engine.model_from_file(eng, 3, a_map("other.sqlite", "192.168.1.99", blue(right=0.9)))
    assert said == ("other.sqlite is a map of laser 1 of 192.168.1.99, not of the laser channel 3 is linked to: its "
                    "model not taken") and ch.laser_model() == model
    poor = Engine.model_from_file(eng, 3, a_map("poor.sqlite", "192.168.1.30", blue(right=0.4)))
    assert "puts only 40% of that map's points on the right mode: not gone by" in poor and ch.laser_model() == model
    junk = tmp_path / "junk.sqlite"
    junk.write_bytes(b"not a map")
    assert Engine.model_from_file(eng, 3, junk).startswith("cannot read junk.sqlite: ")
    misled = blue(ext=lm.Comb(108.07, p_i=-0.375, p_v=0.685))  # (as misread frequencies once had a laser's model)
    said = Engine.model_from_file(eng, 3, a_map("misled.sqlite", "192.168.1.30", misled))
    assert said.startswith("misled.sqlite: the model it keeps is not this laser's (its external modes 108.0700 GHz "
                           "apart would make its cavity 1.39 mm long")
    assert "a model is being fitted to it again, in the background" in said
    end = time.monotonic() + 10.0
    while ch.model_note.startswith("being fitted") and time.monotonic() < end:
        time.sleep(0.05)
    assert ch.laser_model() == model  # (not taken: fitted again, to no points)
    said = Engine.model_from_file(eng, 3, a_map("bare.sqlite", "192.168.1.30"))  # (no model kept: fitted, empty)
    assert said.startswith("bare.sqlite: it keeps no model of the laser: one is being fitted to it, in the background")
    end = time.monotonic() + 10.0
    while ch.model_note.startswith("being fitted") and time.monotonic() < end:
        time.sleep(0.05)
    assert ch.model_note.startswith("none from bare.sqlite: ") and ch.laser_model() == model
    said = Engine.model_from_file(eng, 3, a_map("old.sqlite", "192.168.1.30", blue(right=0.9, version=1)))
    assert said.startswith("old.sqlite: the model it keeps is an older version's: a model is being fitted to it "
                           "again, in the background")  # (its laser's, as far as it can tell, but not by this version)
    end = time.monotonic() + 10.0
    while ch.model_note.startswith("being fitted") and time.monotonic() < end:
        time.sleep(0.05)
    assert ch.model_note.startswith("none from old.sqlite: ") and ch.laser_model() == model


# ------------------------------------------- the lab's 493 nm laser on 27 October
BLUE_NM = 493.545433
BLUE_REF = (67.8905, 65.5175)  # (V, mA) where it ran on target for days, come to sweeping up


def blue_v1() -> lm.LaserModel:
    """The lab's 493 nm DL pro as its map of 25 October has it, fitted by a version before the chip's modes were given
    no drift (version 1): its chip's modes drifting -5.41 GHz per 10 min along the map and moved +0.977 GHz/V by its
    piezo (the same thing: its slow axis's doing), its piezo moving its external modes +0.918 GHz/V (its map could not
    tell that from its cavity's drift; +1.19, as its recoveries have it since)."""
    return lm.LaserModel.from_dict({
        "f_ref": 607.5268773, "i0": 64.9658, "v0": 73.31425, "t0": 24531.563,
        "ext": {"spacing": 8.260032, "p0": 61.24649, "p_i": -1.714243, "p_ii": -0.0060591, "p_v": 0.917847,
                "p_vv": 0.0024266, "p_t": 3.195623, "p_leg": -0.375774, "drift_known": True},
        "chip": {"spacing": 87.175876, "p0": -19.671275, "p_i": -4.234911, "p_ii": -0.0087092, "p_v": 0.976839,
                 "p_vv": -0.00034083, "p_t": -5.409938, "drift_known": True},
        "lo": -214.11636, "hi": 75.564297, "h_up": 0.00050378, "h_down": 0.0048471, "offset": 0.543025,
        "pull": 0.393302, "lag": 0.268832, "soft": 5.0, "i_th": 29.118, "slope": 0.50637, "power_dp": -0.55185,
        "right": 0.726, "pace": 108.705, "step_s": 0.269, "step_x": 0.2024, "kind": "current"})


def as_found(phi: float = 8.0, p_v: float = 1.19) -> tuple:
    """(the laser as its recoveries' maps have it, the target THz): its model's chip drift its piezo's, its piezo moving
    its external modes p_v GHz/V, its chip's modes phi GHz from where its model has them; on target where it ran
    (BLUE_REF), its frequencies GHz from the target."""
    target = lm.C_LIGHT / BLUE_NM * 1e-3
    m = blue_v1()
    m.chip.p_v += m.chip.p_t * m.pace / lm.UNIT_T
    m.chip.p_t = 0.0
    m.ext.p_v, m.ext.p_t, m.ext.others = p_v, 0.0, []
    pl = Planner(m, target, ref=(BLUE_REF[1], BLUE_REF[0]))
    _apply(pl.m, pl._rule(0, phi, 0.0))
    return pl.m, target


def blue_bench(true, target):
    """It on a DL pro whose feedforward (-0.6 mA/V, on) adds nothing as the scan's offset moves the piezo (its scan
    off), on target where it ran, its current come up to it."""
    b = None
    laser = ModelLaser(true, clock=lambda: b.t if b is not None else 0.0)
    v, i = BLUE_REF
    b = Bench(laser, 0.0, current=i - 2.0, offset=v, dt=0.27)
    b.dlc.params["laser1:dl:cc:feedforward-enabled"] = True
    b.dlc.params["laser1:dl:cc:feedforward-factor"] = -0.6
    b.dlc.ff_from[1] = None
    for k in range(21):
        b.client.set("laser1:dl:cc:current-set", i - 2.0 + 2.0 * (k + 1) / 21)
        b.t += 0.15
    b.target = target
    return b, laser


def blue_recover(b, target, p=None):
    p = p or HopParams(enabled=True, method="model", i_nom=65.404, i_below=1.0, i_window=2.0, v_window=25.0,
                       kv=1925.5, spacing=89973.8)  # (as its settings had them)
    n0 = len(b.dlc.log)
    rec = Recenter(p, b.I, b.V, p.i_nom - p.i_below, p.i_nom + p.i_window, b.V - p.v_window, b.V + p.v_window,
                   i_top=p.i_nom + p.jump_max, ref=BLUE_REF, model=blue_v1(), target=target)
    r = b.run(rec)
    sets = [x for name, x in b.dlc.log[n0:] if name == "laser1:dl:cc:current-set"]
    return r, rec, sets


@pytest.mark.parametrize("case", ["drifted", "knocked", "moved"])
def test_the_493nm_laser_as_on_27_october_comes_back_by_its_model(case):
    """Its recoveries of 27 October failed: its DL pro's feedforward adds nothing as the piezo moves, but they reckoned
    the current as if it did (it went 3.2 mA above its window, and the plans' currents were off by 0.6 mA a volt), its
    model (an older version's) had its piezo's doing wrong, and the margins' check took it off its narrow mode. Now:
    drifted 330 MHz off on its right mode, the piezo alone; knocked off it, back where it was on target, come to
    sweeping up; its right mode moved (it hopped as its chip's modes went on), where the map has it, the piezo onto the
    target with the current moved along as its feedforward would. Each time on target, its current inside its window,
    and what its feedforward does kept for the next."""
    true, target = as_found()
    b, laser = blue_bench(true, target)
    assert abs(laser.freq(b.V, b.act) - target) * 1e3 < 1.0  # GHz (on target where it ran)
    if case == "drifted":
        true.ext.p0 += 0.33
    elif case == "knocked":
        for x in (66.6, 67.4, 66.4, BLUE_REF[1]):
            b.client.set("laser1:dl:cc:current-set", x)
            b.t += 0.3
    else:
        for _ in range(60):  # (its chip's modes moved, its cavity drifted: where it ran it is on its mode's edge now)
            true.offset += 0.009
            true.ext.p0 -= 0.006
            b.t += 0.3
            b._settle(None)
        for x in (66.6, 67.4, 66.4, BLUE_REF[1]):  # (and knocked off it)
            b.client.set("laser1:dl:cc:current-set", x)
            b.t += 0.3
    began = (laser.freq(b.V, b.act) - target) * 1e3
    r, rec, sets = blue_recover(b, target)
    assert r.state == "done", r.error
    assert abs(laser.freq(b.V, b.act) - target) * 1e6 < 20.0  # MHz
    assert 65.404 - 1.0 - 1e-6 <= min(sets) and max(sets) <= 65.404 + 2.0 + 1e-6  # (as measured: the same)
    if abs(b.V - BLUE_REF[0]) > 0.01:  # (the piezo moved: what its feedforward does seen, and kept)
        assert rec.ff == 0.0 and rec.learned["ff_set"] == -0.6 and rec.learned["ff_act"] == 0.0
    assert rec.mA(rec.I) == pytest.approx(b.act, abs=Recenter.FF_TELLS)  # (the current as it is, not as the factor
    #                                                                     would have it; a piezo move too small to tell
    #                                                                     what the feedforward does: less than that)
    m = rec.record.get("model") or {}
    if case == "drifted":
        assert abs(began) < 1.0 and rec.maps == 0 and not m
    elif case == "knocked":
        assert abs(began) > 50.0 and rec.back_at == "up" and rec.maps == 0
    else:
        assert abs(began) > 50.0 and rec.maps == 1 and m.get("band")
        assert m["agree"] > 0.9 and rec.learned["model_adapt"]["phi"] == pytest.approx(8.6, abs=1.0)


def test_an_older_models_chip_drift_is_taken_as_its_slow_axis_doing():
    model = blue_v1()
    assert model.version == 1
    pl = Planner(model, lm.C_LIGHT / BLUE_NM * 1e-3, ref=(BLUE_REF[1], BLUE_REF[0]))
    assert pl.m.chip.p_t == 0.0 and pl.m.chip.p_v == pytest.approx(0.976839 - 5.409938 * 108.705 / 600, abs=1e-6)
    assert abs(pl.m.chip.p_v) < 0.01  # (the chip's modes not moved by the piezo, as a fit now has it)
    v2 = blue_v1()
    v2.version = lm.MODEL_VERSION  # (a fit that gave them no drift: as it is)
    assert Planner(v2, lm.C_LIGHT / BLUE_NM * 1e-3).m.chip.p_v == pytest.approx(0.976839)


def test_how_its_piezo_tunes_it_is_learned_from_what_it_did_at_two_voltages():
    true, target = as_found(p_v=1.19)
    v, i = BLUE_REF
    pl = Planner(blue_v1(), target, ref=(i, v))
    for vv in (v, v + 2.0):  # (two maps, 2 V apart)
        state, t = None, 0.0
        for leg, path in ((1, _grid(64.4, 67.4, 0.1)), (2, _grid(67.4, 64.4, 0.1))):
            seg = pl.segment()
            for x in path:
                t += 0.27
                g, _, state = true.step(float(x), vv, true.t0, state, clock=t)
                pl.observe(seg, t, float(x), vv, leg == 1, g)
    pl.fit()
    assert pl.pv0 == pytest.approx(0.917847) and pl.pv0 + pl.pv == pytest.approx(1.19, abs=0.05)
    one = Planner(blue_v1(), target, ref=(i, v))  # (at one voltage: nothing to tell it by)
    for x in _grid(64.4, 67.4, 0.1):
        one.observe(1, 0.0, float(x), v, True, float(true.step(float(x), v, true.t0)[0]))
    one.fit()
    assert one.pv == 0.0


def test_a_channel_keeps_what_its_recoveries_found_of_its_model():
    ch = Channel(3, {"link": {"device": "192.168.1.30", "laser": 1}})
    ch.adopt_model(blue(), "map 1.sqlite")
    ch.adapt_model({"phi": 8.0, "p_v": 1.19})
    got = ch.laser_model()
    assert got.offset == pytest.approx(blue().offset + 8.0) and got.ext.p_v == 1.19 and got.ext.others == []
    assert got.alt["offset"] == pytest.approx(blue().alt["offset"] + 8.0)
    back = Channel(3, ch.to_config())  # (kept in its settings)
    assert back.laser_model().offset == pytest.approx(got.offset) and back.model_dict["adapt"]["n"] == 1
    back.adapt_model({"phi": 80.0})  # (as well a whole chip mode on: 88.0 GHz, 0.62 GHz)
    assert back.model_dict["adapt"]["phi"] == pytest.approx(88.0 - 87.379, abs=1e-3)
    assert back.laser_model().ext.p_v == 1.19 and back.model_dict["adapt"]["n"] == 2
    back.adopt_model(blue(), "map 2.sqlite")  # (a new map: nothing of the old one's)
    assert "adapt" not in back.model_dict and back.laser_model().offset == blue().offset


def test_check_setup_says_what_its_feedforward_does_and_what_its_model_found():
    from test_modehop import Loopback

    from wmlock.lock import LockParams
    from wmlock.relock import setup_report
    from wmlock.sim import FakeDLCPro

    dlc = FakeDLCPro("SN-B", "bench", {1: {"current": 65.5, "offset": 67.9, "feedforward": -0.6}}, serve=False)
    ch = Channel(5, {"link": {"device": "192.168.1.103", "laser": 2}})
    ch.adopt_model(blue_v1(), "map_2026-10-25_012516_ch5.sqlite")
    ch.model_dict["model"].pop("version")  # (kept by the version before it said)
    ch.adapt_model({"phi": 8.5, "p_v": 1.19})

    def text(**hop):
        p = HopParams(enabled=True, method="model", i_nom=65.404, **hop)
        return "\n".join(t for _, t in setup_report(Loopback(dlc), 1, p, LockParams(), ch.model_dict))

    before = text()
    assert "Whether it does, a recovery's first piezo move shows" in before
    seen = text(ff_set=-0.6, ff_act=0.0)
    assert "its recoveries saw that the feedforward adds nothing as the piezo moves" in seen
    assert "move it along themselves as they tune the piezo, -0.6000 mA/V" in seen
    assert "its chip's modes +8.5 GHz from where it had them, its piezo tuning its external modes +1.19 GHz/V" in seen
    assert "fitted by an older version" in seen and "back where it was on target" in seen
    assert "Whether it does" not in text(ff_set=-0.6, ff_act=-0.6)  # (it does)
