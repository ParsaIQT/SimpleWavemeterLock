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
    assert h.how(shg=True, model=True) == "catch" and h.how(shg=True, model=True, policy=True) == "catch"
    assert HopParams(method="learned", cavity=True).how(model=True, policy=True) == "model"  # (neither tried: as auto)
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


# -------------------------------------------------------------- the engine
def test_the_engine_keeps_a_policy_for_its_lasers_model_and_what_its_manoeuvres_found(kept, tmp_path, monkeypatch):
    """A channel with a model, its method 'learned': its policy is the one kept for that model beside the recoveries'
    log; none kept, one is learned in a thread of its own, the recoveries going by the model meanwhile; what its
    recoveries have since found of the model (where its chip's modes are, how its piezo tunes it) leaves the policy
    its own; another model, another policy. What its manoeuvres came to is kept with its model."""
    import functools
    import threading
    import time
    import types

    from wmlock.engine import Channel, Engine

    model = trials.lab650()
    made = kept(model)
    key = learned.model_key(model)
    ch = Channel(3, {"link": {"device": "192.168.1.30", "laser": 1}})
    ch.adopt_model(model, "map.sqlite")
    eng = types.SimpleNamespace(lock=threading.RLock(), dirty=False, policy_dir=trials.POLICIES)
    for name in ("_policy", "_policy_said", "_memory"):
        setattr(eng, name, getattr(Engine, name) if isinstance(Engine.__dict__[name], staticmethod) else
                types.MethodType(getattr(Engine, name), eng))
    got = eng._policy(ch)  # (kept: read)
    assert got is not None and ch.policy is got and ch.policy_key == key and not ch.policy_busy
    x = learned.features(model, model.i0, model.v0, 6, -1.0, 2.0)[0]
    assert got.choose(x, -1.0, 2.0).k == made.choose(x, -1.0, 2.0).k
    assert ch.policy_note.startswith("learned from its laser's model: ") and "back by its first manoeuvre" in ch.policy_note
    ch.adapt_model({"phi": 3.2, "p_v": 0.43})  # (what a recovery found since: its policy still)
    assert learned.model_key(ch.laser_model()) == key and eng._policy(ch) is got
    # none kept: learned on the side
    eng.policy_dir = str(tmp_path / "none-yet")
    ch.policy = None
    monkeypatch.setattr(learned, "train", functools.partial(learned.train, n=200, again=300, epochs=10))
    assert eng._policy(ch) is None and ch.policy_busy and ch.policy_note == "being learned from its laser's model…"
    assert eng._policy(ch) is None  # (asked again meanwhile: not begun twice)
    end = time.monotonic() + 120
    while ch.policy is None and time.monotonic() < end:
        time.sleep(0.2)
    assert ch.policy is not None and not ch.policy_busy and ch.policy_key == key
    assert learned.policy_for(model, eng.policy_dir, make=False) is not None  # (kept for the next time)
    # its memory, kept with its model in its settings
    assert eng._memory(ch).items == []
    m = learned.Memory()
    m.add(learned.features(model, model.i0, model.v0, 6, -1.0, 2.0)[0], 5, True)
    ch.model_dict.setdefault("learned", {})["memory"] = m.to_list()
    again = Channel(3, ch.to_config())
    assert eng._memory(again).to_list() == m.to_list()
    # another model: not this policy
    other = trials.lab650()
    other.h_down *= 2
    ch.adopt_model(other, "other.sqlite")
    eng.policy_dir = str(tmp_path / "nowhere")
    monkeypatch.setattr(learned, "train", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("no")))
    assert eng._policy(ch) is None and eng._memory(ch).items == []
    end = time.monotonic() + 10
    while ch.policy_busy and time.monotonic() < end:
        time.sleep(0.05)
    assert ch.policy is None and ch.policy_note == "not learned: no"


# ------------------------------------------------------ a recapture by hand
def recorded(kind, seed):
    """A trial's laser brought back while every reading is recorded with its current and its piezo, as a hand at it
    would be (here the hand is the policy's): (the case, its recovery's record, the recording)."""
    from wmlock import handrec

    case = make_case(kind, seed, method="learned")
    rig = case.rig
    v_ref, i_ref = case.kwargs["ref"]
    rec = handrec.Recording(1, "red", target=case.kwargs["target"], ref=[v_ref, i_ref],
                            window=[case.args[2], case.args[3]], tol=case.tol)
    rig.also = lambda t, e: rec.add(t, rig.I, rig.act, rig.V, e) and None
    recovery = Recenter(case.p, *case.args, **case.kwargs)
    r = rig.run(recovery)
    assert r.state == "done"
    for _ in range(40):  # (and left there, locked: on target for a while)
        t, e = rig.reading()
        if rec.add(t, rig.I, rig.act, rig.V, e):
            break
    return case, recovery, rec


def test_a_recapture_by_hand_is_recorded_and_read_as_a_manoeuvre(kept, tmp_path):
    """What is recorded while a laser is brought back (its current, its piezo, its readings) ends by itself once it
    has been on target a while, is kept as a line of JSON, and reads as the manoeuvre it was: the way its current
    was going as it came onto its right mode, how far it had been the other way, how deep it was left; which its
    laser's memory then holds for the next hop of that kind."""
    from wmlock import handrec

    kept(trials.lab650())
    case, recovery, rec = recorded("red", 304)
    model = case.kwargs["model"]
    assert rec.end == "on target" and len(rec.samples) > 15 and rec.samples[0][0] == 0.0
    path = tmp_path / "hand.jsonl"
    handrec.save(path, rec)
    handrec.save(path, rec)
    back = handrec.load(path)
    assert len(back) == 2 and back[0].samples == rec.samples and back[0].ref == rec.ref and back[0].end == "on target"
    got = handrec.read(back[0], model)
    flown = recovery.record["learned"]["tries"][-1]
    assert got["why"] == "" and got["ok"] and got["jump"] == round(case.hop / model.ext.spacing)
    assert got["way"] == flown["way"] and (got["turn"] is None) == (flown["turn"] is None or
                                                                    got["way"] * flown["turn"] > -0.2)
    assert got["left"] == pytest.approx(flown["at"], abs=0.02) and 0.0 < got["depth"] < 1.0
    turns, ways, _ = learned.actions()
    assert ways[got["k"]] == got["way"] and np.isnan(turns[got["k"]]) == (got["turn"] is None)
    memory = learned.Memory()
    memory.add(got["x"], got["k"], True)
    v_ref, i_ref = case.kwargs["ref"]
    x = learned.features(model, i_ref, v_ref + model.v_set_to_act, got["jump"], case.args[2] - i_ref,
                         case.args[3] - i_ref)[0]
    assert x == pytest.approx(np.array(got["x"]), abs=1e-3) and memory.bonus(x)[got["k"]] > 0.4
    # what is not a manoeuvre says why
    still = handrec.Recording(1, target=rec.target, ref=rec.ref, tol=rec.tol)
    for k in range(30):
        still.add(0.1 * k, i_ref, i_ref, v_ref, 3.0)
    assert still.end == "" and handrec.read(still, model)["why"] == "it was on its right mode all along"
    off = [smp for smp in rec.samples if abs(smp[4]) > 1000.0]  # (given up while it was still a mode away)
    lost = handrec.Recording.from_dict(dict(rec.to_dict(), samples=(off * 5)[:max(len(off), 6)]))
    assert handrec.read(lost, model)["why"] == "it did not end on its right mode"
    assert handrec.read(handrec.Recording(1), model)["why"].startswith("too few readings")
    long = handrec.Recording(1, tol=5.0)
    assert not long.add(0.0, 1, 1, 1, 500.0) and long.add(handrec.MOST_S + 1, 1, 1, 1, 500.0) and long.end == "too long"


def test_the_engine_records_a_recapture_by_hand_and_takes_it_into_its_memory(kept, tmp_path):
    import threading
    import types

    from wmlock import handrec
    from wmlock.engine import Channel, Engine

    kept(trials.lab650())
    case, recovery, rec = recorded("red", 306)
    model = case.kwargs["model"]
    v_ref, i_ref = case.kwargs["ref"]
    ch = Channel(3, {"link": {"device": "192.168.1.30", "laser": 1}, "name": "650"})
    ch.adopt_model(model, "map.sqlite")
    ch.hop = HopParams(enabled=True, method="learned", i_nom=case.p.i_nom, i_below=1.0, i_window=2.0)
    ch.f_set, ch.current, ch.current_set, ch.offset = case.kwargs["target"], i_ref, i_ref, v_ref
    ch.fine_at = (0.0, v_ref, i_ref)
    eng = types.SimpleNamespace(lock=threading.RLock(), dirty=False, channels={3: ch},
                                hand_path=str(tmp_path / "x-hand.jsonl"), _memory=Engine._memory)
    eng._hand_done = types.MethodType(Engine._hand_done, eng)
    assert Engine.record_hand(eng, 3, False) == "no recapture by hand is being recorded"
    assert Engine.record_hand(eng, 3).startswith("recording a recapture by hand") and ch.hand is not None
    assert ch.hand.ref == [v_ref, i_ref] and ch.hand.window == [case.args[2], case.args[3]] and ch.hand.name == "650"
    assert Engine.record_hand(eng, 3, True) == "a recapture by hand is being recorded already"
    on = next(k for k, smp in enumerate(rec.samples) if abs(smp[4]) <= 20.0)  # (on target; stopped a second later)
    for t, i_set, i_act, v, e in rec.samples[:on + 8]:  # (as its readings would feed it)
        assert not ch.hand.add(t, i_set, i_act, v, e)
    said = Engine.record_hand(eng, 3)  # (stopped by hand)
    assert ch.hand is None and "taken into its policy's memory" in said and "onto its right mode at" in said
    assert ch.note == said and eng.dirty
    kept_ = handrec.load(eng.hand_path)
    assert len(kept_) == 1 and kept_[0].end == "stopped" and len(kept_[0].samples) == on + 8
    items = ch.model_dict["learned"]["memory"]
    assert len(items) == 1 and items[0][2] is True and Engine._memory(ch).bonus(items[0][0])[items[0][1]] > 0.4
    ch.device = ""
    assert Engine.record_hand(eng, 3).startswith("recording a recapture by hand needs a laser on a DLC pro")
