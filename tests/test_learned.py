"""The learned recovery (HopParams.method 'learned'): many model lasers at once (fastsim.Batch) against the
simulator's own (sim.ModelLaser); what a policy is learned from (learned.py: lasers like the model hopped, an oracle's
word on every manoeuvre) and what it chooses; and the recovery that flies it (modehop.Recenter.by_learning) on the
trials' rig, in simulated time: the lasers back on their right mode in seconds, every write within the safety rules,
and the methods of old where there is no policy or its manoeuvres do not get there."""
import statistics

import pytest

np = pytest.importorskip("numpy")

from wmlock import fastsim, learned, modehop, trials  # noqa: E402
from wmlock.modehop import STEP_MAX, HopParams, Recenter  # noqa: E402
from wmlock.trials import make_case, recover  # noqa: E402

CURRENT, OFFSET = "laser1:dl:cc:current-set", "laser1:scan:offset"
ALLOWED = {CURRENT, OFFSET} | {"laser1:" + name for name in modehop.WRITABLE}


@pytest.fixture(scope="module")
def kept(tmp_path_factory):
    """Policies learned for the tests' lasers: small ones (seconds each), kept where the trials look for them.
    kept(model) makes sure there is one, and returns it."""
    where = str(tmp_path_factory.mktemp("policies"))
    was, trials.POLICIES = trials.POLICIES, where

    def ready(model):
        return learned.policy_for(model, where, n=400, again=800, epochs=40)

    yield ready
    trials.POLICIES = was


# ------------------------------------------------------- simulated hops
def test_a_simulated_laser_hops_where_its_mode_gives_way_and_not_before():
    rng = np.random.default_rng(3)
    model = trials.lab493()
    s = learned.hopped(model, 200, rng)
    b, right = s["b"], s["right"]
    assert b.n > 150 and np.all(b.order != right) and set(np.unique(s["way"])) == {-1.0, 1.0}
    assert np.all(np.abs(s["jump"]) >= 1) and np.all(s["lo"] <= -0.2) and np.all(s["hi"] >= 0.2)
    before = b.copy()  # (its chip's modes a thousandth of their spacing back: still on its right mode there)
    before.offset = b.offset - s["way"] * 0.001 * b.dc
    before.put(right, i=s["i_ref"], v=s["v_ref"])
    before.step(s["i_ref"], s["v_ref"], np.inf)
    assert np.mean(before.order == right) > 0.97


# ----------------------------------------------------------------- a policy
def test_a_policy_is_learned_from_the_oracle_kept_and_read_back(kept, tmp_path, monkeypatch):
    model = trials.lab650()
    policy = kept(model)
    meta = policy.meta
    assert meta["key"] == learned.model_key(model) and meta["version"] == learned.VERSION
    assert meta["n"] > 300 and meta["again"] > 20 and meta["took_s"] < 300
    up = meta["trial"]["up"]  # (of lasers it never saw: those some manoeuvre could bring back, it brings back)
    assert up["on"] >= up["can"] - 0.05 and up["first"] >= 0.75 and up["s"] < 2.5
    policy.save(str(tmp_path / "p.npz"))
    again = learned.Policy.load(str(tmp_path / "p.npz"))
    x = learned.features(model, model.i0 + 0.3, model.v0, 6, -1.0, 2.0)[0]
    a, b = policy.choose(x, -1.0, 2.0), again.choose(x, -1.0, 2.0)
    assert (a.k, a.turn, a.way, a.depth) == (b.k, b.turn, b.way, b.depth) and a.on == pytest.approx(b.on)
    assert again.hold == policy.hold == learned.hold_s(model) and again.lag == model.lag
    monkeypatch.setattr(learned, "train", lambda *a, **k: pytest.fail("it is kept: not learned again"))
    assert learned.policy_for(model, trials.POLICIES).meta["key"] == meta["key"]
    other = trials.lab650()
    other.h_up *= 1.5  # (another model: another key, none kept)
    assert learned.model_key(other) != meta["key"] and learned.policy_for(other, trials.POLICIES, make=False) is None


def test_its_choice_is_within_the_window_and_not_one_tried_already(kept):
    model = trials.lab650()
    policy = kept(model)
    turns, ways, depths = learned.actions()
    assert len(turns) == len(ways) == len(depths) == policy.net.n_out == 72
    rng = np.random.default_rng(5)
    for _ in range(60):
        lo, hi = -rng.uniform(0.2, 2.5), rng.uniform(0.2, 2.5)
        x = learned.features(model, model.i0 + rng.uniform(-1, 1), model.v0 + rng.uniform(-3, 3),
                             int(rng.choice([-11, -5, 1, 5, 6, 11])), lo, hi)[0]
        c = policy.choose(x, lo, hi)
        assert c is not None and (c.turn is None or lo - 1e-9 <= c.turn <= hi + 1e-9)
        assert 0.0 <= c.on <= 1.0 and 0.0 <= c.margin <= learned.MARGIN + 1e-6 and c.took > 0
        assert (turns[c.k], ways[c.k], depths[c.k]) == (pytest.approx(np.nan if c.turn is None else c.turn, nan_ok=True),
                                                        c.way, c.depth)
        d = policy.choose(x, lo, hi, skip=[c.k])
        assert d is not None and d.k != c.k
        assert policy.choose(x, lo, hi, least=1.1) is None  # (none has such a chance)


def test_what_the_laser_itself_showed_mends_its_policys_choice(kept):
    """Its memory: a manoeuvre that did not bring it back is not the first choice the next time in a like situation;
    one that did, is; in another kind of hop, neither counts."""
    model = trials.lab650()
    policy = kept(model)
    x = learned.features(model, model.i0, model.v0, 6, -1.0, 2.0)[0]
    far = learned.features(model, model.i0, model.v0, -5, -1.0, 2.0)[0]
    first = policy.choose(x, -1.0, 2.0)
    m = learned.Memory()
    assert not m.bonus(x).any()
    m.add(x, first.k, False)
    assert m.bonus(x)[first.k] < -0.4 and abs(m.bonus(far)[first.k]) < 0.02
    second = policy.choose(x, -1.0, 2.0, bonus=m.bonus(x))
    assert second.k != first.k
    m.add(x, second.k, True)
    m.add(x, second.k, True)
    assert policy.choose(x, -1.0, 2.0, bonus=m.bonus(x)).k == second.k and m.bonus(x)[second.k] > 0.4
    again = learned.Memory.from_list(m.to_list())
    assert again.bonus(x) == pytest.approx(m.bonus(x)) and len(again.items) == 3
    for _ in range(300):
        m.add(far, 0, True)
    assert len(m.items) == learned.Memory.KEEP and learned.Memory.from_list("nonsense").items == []


# ------------------------------------------------------- flown, on the rig
def written(rig, name=CURRENT):
    return rig.dlc.written(name)


def safe(case, rig, before: int = 0):
    """The safety rules, of the recovery's own writes (those after the first `before`, which put the trial's laser
    on its mode): the current within its window, no write of it further than STEP_MAX from the last; the piezo
    within its window; nothing written that a recovery may not write. Returns its writes of the current."""
    p, i0, v0 = case.p, case.args[0], case.args[1]
    lo, hi = p.i_nom - p.i_below, p.i_nom + p.i_window
    log = rig.dlc.log[before:]
    w = [value for name, value in log if name == CURRENT]
    assert all(lo - 1e-6 <= x <= hi + 1e-6 for x in w)
    assert max((abs(b - a) for a, b in zip([i0] + w, w)), default=0.0) <= STEP_MAX + 1e-9
    assert all(v0 - p.v_window - 1e-6 <= value <= v0 + p.v_window + 1e-6 for name, value in log if name == OFFSET)
    assert {name for name, _ in log} <= ALLOWED
    return w


@pytest.mark.parametrize("kind,n,median,most", [("red", 40, 2.5, 4.0), ("blue", 24, 5.0, 8.0)])
def test_monte_carlo_learned_recoveries_in_seconds_every_write_within_the_rules(kept, kind, n, median, most):
    """The lab's two lasers as their maps' models have them, each trial's laser unlike its model in its own way,
    hopped: its policy (learned from its model alone) has every one back on its own mode, on target, in seconds,
    with room either way; and every write of every recovery keeps the safety rules."""
    kept({"blue": trials.lab493, "red": trials.lab650}[kind]())
    outs = []
    for seed in range(300, 300 + n):
        case = make_case(kind, seed, method="learned")
        before = len(case.rig.dlc.log)
        o = recover(case)
        assert o.on_target and o.right, (kind, seed, o.error, o.notes)
        assert "by its policy" in o.summary and o.method == "learned", (kind, seed, o.summary)
        w = safe(case, case.rig, before)
        steps = [abs(b - a) for a, b in zip([case.args[0]] + w, w)]
        assert statistics.median(steps) <= Recenter.FLY_FINE + 1e-9  # (ramped in small writes)
        outs.append(o)
    assert statistics.median(o.seconds for o in outs) < median and max(o.seconds for o in outs) < most
    margins = sorted(o.margin for o in outs)
    assert margins[len(margins) // 10] >= 0.25 and statistics.median(margins) >= 0.5
    assert max(o.second for o in outs) < 0.2  # (none left with a second mode to speak of)


def test_a_manoeuvre_flown_on_the_rig_ends_where_the_simulation_has_it(kept):
    """What a policy is learned from is learned.fly's manoeuvres; what it then flies is Recenter.by_learning's, by a
    Runner's sweeps on the rig: the same manoeuvre on the same laser must end at the same current, within what a
    reading's time is worth, on the same mode."""
    kept(trials.lab650())
    for seed in (301, 305, 311, 317):
        case = make_case("red", seed, method="learned")
        rig, laser = case.rig, case.rig.laser
        true, policy, model = laser.model, case.kwargs["policy"], case.kwargs["model"]
        v_ref, i_ref = case.kwargs["ref"]
        b = fastsim.Batch.of([true], sided=laser.sided)
        order, warm, _, _, up = laser._state
        b.put(order, i=i_ref, warm=warm, up=up, delta=laser._delta)
        right = np.array([int(round((b.mode_ghz(0, i_ref, v_ref)[0] - (case.kwargs["target"] - true.f_ref) * 1e3)
                                    / -true.ext.spacing))])
        lo, hi = case.args[2] - i_ref, case.args[3] - i_ref
        x = learned.features(model, i_ref, v_ref + model.v_set_to_act, order - right[0], lo, hi)[0]
        c = policy.choose(x, lo, hi, least=Recenter.FLY_LEAST)
        b = b.repeat(8)  # (eight times over: its readings falling anywhere in their interval)
        end, took, seen = learned.fly(b, np.repeat(right, 8), i_ref, v_ref, np.nan if c.turn is None else c.turn,
                                      c.way, c.depth, lo, hi, policy.hold, rig.dt, policy.lag,
                                      np.random.default_rng(seed))
        o = recover(case)
        assert o.on_target and o.right and np.all(b.order == right[0]) and seen.all()
        assert "manoeuvre" not in o.summary, o.summary  # (its first manoeuvre: the one simulated)
        assert end.max() - end.min() <= learned.PER_READ + 1e-9  # (a reading's phase: PER_READ at most)
        assert rig.act - i_ref == pytest.approx(end.mean(), abs=0.75 * learned.PER_READ)
        assert o.seconds == pytest.approx(took.mean() + 6 * rig.dt, abs=5 * rig.dt)  # (the readings before and after)


# ------------------------------------------------------- the methods of old
def test_without_a_model_or_a_policy_it_is_what_auto_is():
    h = HopParams(method="learned")
    assert h.how(model=True, policy=True) == "learned" and h.how(model=True) == "model" and h.how() == "map"
    assert h.how(shg=True, model=True) == "catch" and h.how(shg=True, model=True, policy=True) == "learned"
    assert HopParams().how(model=True, policy=True) == "model"  # (not chosen: nothing changes)
    for kind, seed in (("far", 3), ("strong", 5)):  # (no model: the map, to the reading the same)
        a, b = recover(make_case(kind, seed, method="learned")), recover(make_case(kind, seed))
        assert (a.ok, a.seconds, a.readings, a.summary) == (b.ok, b.seconds, b.readings, b.summary)
        assert a.method == b.method == "map"
    case = make_case("red", 7)  # (a model, no policy: its model's way, to the reading the same)
    mine = Recenter(HopParams.from_dict(dict(case.p.to_dict(), method="learned")), *case.args, **case.kwargs)
    assert mine.method == "model" and mine.policy is None
    r = case.rig.run(mine)
    theirs = recover(make_case("red", 7))
    assert r.state == "done" and r.result.summary == theirs.summary


def test_manoeuvres_that_do_not_bring_it_back_hand_over_to_its_models_way(kept, monkeypatch):
    """A policy that is wrong about the laser (here: made to choose the wrong way each time): its manoeuvres are
    flown, each read and found wanting, no more than FLY_TRIES of them, and its model's way takes over from where
    they left it; the laser back on target all the same, every write within the rules."""
    policy = kept(trials.lab650())
    turns, ways, depths = learned.actions()
    wrong = [k for k in range(len(turns)) if np.isnan(turns[k]) and ways[k] < 0]

    def choose(self, x, lo, hi, bonus=None, least=0.0, skip=()):
        k = next(k for k in wrong if k not in skip)
        return learned.Choice(None, -1.0, float(depths[k]), 0.5, 0.3, 1.0, k)

    monkeypatch.setattr(learned.Policy, "choose", choose)
    case = make_case("red", 302, method="learned")
    memory = learned.Memory()
    before = len(case.rig.dlc.log)
    rec = Recenter(case.p, *case.args, memory=memory, **case.kwargs)
    r = case.rig.run(rec)
    assert r.state == "done" and abs(case.rig.off_mhz()) <= case.tol, r.error
    flown = rec.record["learned"]["tries"]
    assert len(flown) == Recenter.FLY_TRIES and all(t["jump"] != 0 and t["way"] < 0 for t in flown)
    assert any("its policy did not bring it back onto its right mode" in n for n in rec.notes)
    assert "by its policy" not in r.result.summary and rec.flown is None
    assert len(memory.items) == Recenter.FLY_TRIES and not any(ok for _, _, ok in memory.items)
    assert r.result.learned["policy_memory"] == memory.to_list()  # (what it found is kept: not those again)
    safe(case, case.rig, before)


def test_step_by_step_it_asks_before_each_manoeuvre(kept):
    kept(trials.lab650())
    case = make_case("red", 303, method="learned", confirm=True)
    rec = Recenter(case.p, *case.args, **case.kwargs)
    before = len(case.rig.dlc.log)  # (what put the laser on its mode, and had it hop: the trial's own)
    r = case.rig.run(rec, until=lambda r: r.asking)
    assert r.asking.startswith("Its policy: the current ") and "until a reading has it on its right mode" in r.asking
    assert len(case.rig.dlc.log) == before
    r.cancel("stopped")
    while r.busy:
        case.rig.step(r)
    assert len(case.rig.dlc.log) == before and r.state == "failed"
