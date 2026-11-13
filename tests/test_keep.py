"""A laser kept off the edge of its mode before it hops (keep.py, modehop.Recenter.keep, the engine's part): what is
watched (a signal's level against what it should be; what is left of the wavemeter's fringes), when its watch says so,
what the keeping does on the simulated lasers (trials.Rig: its SHG signal, or its fringes on the wavemeter, with
this program's lock going on), and lasers drifting towards their hops for hours, kept and left alone."""
import math
import random
import re

import pytest

from wmlock import keep as keep_mod
from wmlock.keep import Fringes, Level, ModeWatch
from wmlock.lock import Lock, LockParams
from wmlock.modehop import HopParams, Recenter
from wmlock.sim import ECDL, SimCCD
from wmlock.trials import I0, V0, Rig, _locked, _windows, far_ecdl, keep_trial, near_ecdl

CURRENT = "laser1:dl:cc:current-set"


def feed(level, t, seconds, value, noise=0.0, rng=None, dt=0.5):
    """`seconds` of a signal at `value` (callable: of the time), read every dt: the time after."""
    rng = rng or random.Random(0)
    for _ in range(int(round(seconds / dt))):
        t += dt
        x = value(t) if callable(value) else value
        level.feed(t, x + (rng.gauss(0.0, noise) if noise else 0.0))
    return t


# ------------------------------------------------------------------- the watch
def test_a_level_knows_what_it_should_be_and_how_far_it_has_dipped():
    lv, rng = Level(), random.Random(1)
    assert lv.base() is None and lv.now() is None and lv.dip() == (0.0, 0.0)
    t = feed(lv, 0.0, 50.0, 1.20, 0.003, rng)
    assert lv.base() is None  # (not yet a baseline of its own: BASE_MIN blocks of BLOCK_S)
    t = feed(lv, t, 600.0, 1.20, 0.003, rng)
    assert lv.base() == pytest.approx(1.20, abs=0.002) and lv.now() == pytest.approx(1.20, abs=0.002)
    assert lv.noise() == pytest.approx(0.003, rel=0.5)
    d, sd = lv.dip()
    assert abs(d) < 0.002 and sd < 3.0
    t = feed(lv, t, 30.0, 1.20 * 0.97, 0.003, rng)  # (3 % down, for half a minute)
    d, sd = lv.dip()
    assert d == pytest.approx(0.03, abs=0.003) and sd > 20.0
    assert lv.base() == pytest.approx(1.20, abs=0.002)  # (what it should be: not dragged down)
    t = feed(lv, t, 600.0, 1.20 * 0.97, 0.003, rng)     # (nor by ten minutes of it, of an hour's blocks)
    assert lv.base() == pytest.approx(1.20, abs=0.003) and lv.dip()[0] == pytest.approx(0.03, abs=0.004)


def blocks(level, t, n, value, noise=0.0, rng=None):
    """n whole blocks of it at `value` (BLOCK_S of readings each, every 0.5 s): the time after."""
    return feed(level, t, n * (Level.BLOCK_S + 0.5), value, noise, rng)


def test_one_odd_block_is_not_a_dip_and_a_prior_serves_until_it_has_its_own():
    lv = Level(prior=2.0)
    t = blocks(lv, 0.0, 2, 1.9)
    assert lv.base() == 2.0 and lv.dip()[0] == pytest.approx(0.05)  # (against what it was learned to be before)
    lv = Level()
    t = blocks(lv, 0.0, 30, 1.0)
    t = blocks(lv, t, 1, 0.9)    # (one block low...)
    assert lv.dip()[0] == pytest.approx(0.0, abs=1e-9)
    t = blocks(lv, t, 1, 1.0)    # (...the next up again)
    assert lv.dip()[0] == pytest.approx(0.0, abs=1e-9)
    t = blocks(lv, t, 1, 0.9)
    assert lv.dip()[0] == pytest.approx(0.0, abs=1e-9)  # (the last two: one of them is up)
    t = blocks(lv, t, 1, 0.9)
    assert lv.dip()[0] == pytest.approx(0.1, abs=1e-6)
    lv.feed(t + 1.0, math.nan)
    lv.feed(t + 1.5, None)  # (no reading: passed over)
    assert len(lv.blocks) == 34


def test_a_level_follows_a_signal_that_rose_and_forgets_after_an_hour():
    lv = Level()
    t = feed(lv, 0.0, 1800.0, 1.0)
    t = feed(lv, t, 300.0, 1.1)
    assert lv.base() == pytest.approx(1.0)        # (five minutes higher, of thirty-five: not yet)
    t = feed(lv, t, 900.0, 1.1)
    assert lv.base() == pytest.approx(1.1)        # (a fifth of its blocks and more: what it should be now)
    t = feed(lv, t, 3700.0, 0.8)
    assert lv.base() == pytest.approx(0.8)        # (an hour of it: the old blocks are gone)
    assert lv.trend() == pytest.approx(0.0, abs=1e-9)
    t = feed(lv, t, 120.0, lambda s: 0.8 * (1 - 0.01 * (s - t) / 60.0))  # (falling 1 % a minute)
    assert lv.trend() == pytest.approx(-0.01, rel=0.3)
    lv.rebase()                                   # (what it is now is what it should be)
    assert lv.base() == pytest.approx(0.8 * 0.985, abs=0.006) and not lv.blocks
    lv.reset(1.5)
    assert lv.base() == 1.5 and lv.now() is None


def pattern(ccd, comps, power=20.0):
    value, extra, _ = ccd.measure(comps, power, 150.0, 473.0, 473.0, True)
    return extra["patterns"][0][1]


def test_what_is_left_of_its_fringes_tells_a_second_mode_and_nothing_else():
    np = pytest.importorskip("numpy")
    ccd, fr = SimCCD(seed=3, amp_noise=3.0), Fringes()
    first = [fr.seen(pattern(ccd, [(1.0, 0.0)])) for _ in range(Fringes.AVERAGE)]
    assert first[:-1] == [None] * (Fringes.AVERAGE - 1) and first[-1] == 1.0  # (learned over its first patterns)
    one = [fr.seen(pattern(ccd, [(1.0, 0.0)])) for _ in range(40)]
    assert np.mean(one) == pytest.approx(1.0, abs=0.002) and np.std(one) < 0.001
    # the laser elsewhere (its fringes anywhere along the CCD), brighter, dimmer (the exposure follows): the same
    moved = [fr.seen(pattern(ccd, [(1.0, g)])) for g in np.linspace(0.0, 9.0, 19)]
    assert max(abs(q - 1.0) for q in moved) < 0.02
    for power in (10.0, 40.0, 25.0, 20.0):
        q = [fr.seen(pattern(ccd, [(1.0, 0.0)], power)) for _ in range(6)][-1]
        assert q == pytest.approx(1.0, abs=0.012)
    # a second mode with 3 % of the power, then 10 %: its fringes fade by more than its part, whether it is 3, 6 or
    # 87 GHz away (the 493's chip modes), and wherever the fringes are; one exactly a fringe of the finest
    # interferometer away (9 GHz) still shows in the others
    for away in (3.0, 6.0, 12.0, 87.0):
        q3 = np.mean([fr.seen(pattern(ccd, [(0.97, 0.0), (0.03, away)])) for _ in range(10)])
        q10 = np.mean([fr.seen(pattern(ccd, [(0.90, 0.0), (0.10, away)])) for _ in range(10)])
        assert 0.93 < q3 < 0.965 and 0.80 < q10 < 0.88, (away, q3, q10)
    at = [fr.seen(pattern(ccd, [(0.95, g), (0.05, g + 6.0)])) for g in np.linspace(0.0, 9.0, 10)]
    assert max(at) - min(at) < 0.01 and np.mean(at) == pytest.approx(0.926, abs=0.01)
    assert np.mean([fr.seen(pattern(ccd, [(0.97, 0.0), (0.03, 9.0)])) for _ in range(10)]) < 0.985
    assert fr.seen(None) is None and fr.seen([1, 2, 3]) is None and fr.seen("no") is None  # (no pattern, or not one)
    assert fr.seen(pattern(ccd, [(1.0, 0.0)])[:1024]) is None  # (another size: learned afresh)
    assert fr.parts is None


def test_the_watch_says_when_and_waits_longer_each_time_it_did_not_help():
    w, rng = ModeWatch(), random.Random(2)
    lv = w.signals["shg"]
    t = blocks(lv, 0.0, 60, 1.23, 0.003, rng)
    assert w.alarm(t, 0.02) is None and w.state()["shg"][0] == pytest.approx(0.0, abs=0.002)
    t = blocks(lv, t, 3, 1.23 * 0.99, 0.003, rng)    # (1 % down: less than asked)
    assert w.alarm(t, 0.02) is None
    t = blocks(lv, t, 2, 1.23 * 0.97, 0.003, rng)    # (3 %, two blocks running: twenty seconds)
    a = w.alarm(t, 0.02)
    assert a["signal"] == "shg" and a["dip"] == pytest.approx(0.03, abs=0.004) and a["way"] == 0
    assert a["base"] == pytest.approx(1.23, abs=0.003) and a["level"] == pytest.approx(1.23 * 0.97, abs=0.003)
    w.kept(t, True, way=-1)                          # (mended, the current down: not again for GAP s, and not
    assert w.alarm(t + ModeWatch.GAP - 1, 0.02) is None and w.kept_n == 1  # by what was read before)
    assert w.alarm(t + ModeWatch.GAP + 1, 0.02) is None and lv.dip() == (0.0, 0.0)
    t = blocks(lv, t, 2, 1.23 * 0.97, 0.003, rng)    # (still down, by new readings)
    a = w.alarm(t, 0.02)
    assert a is not None and a["way"] == -1          # (the way that mended it last: first, the next time)
    w.kept(t, False)                                 # (not mended: WAIT s, then twice that, ...)
    assert w.alarm(t + ModeWatch.WAIT - 1, 0.02) is None and w.alarm(t + ModeWatch.WAIT + 1, 0.02) is not None
    w.kept(t, False)
    assert w.alarm(t + 2 * ModeWatch.WAIT - 1, 0.02) is None and w.wait == 2 * ModeWatch.WAIT
    for _ in range(10):
        w.kept(t, False)
    assert w.wait == ModeWatch.WAIT_MAX
    w.kept(t, True, way=1)
    assert w.wait == 0.0 and w.way == 1
    # the current made no difference: the signal as it is now is what it should be from now on
    t = blocks(lv, t, 2, 1.23 * 0.97, 0.003, rng)
    w.next = 0.0
    assert w.alarm(t, 0.02) is not None
    w.kept(t, False, rebase=True)
    w.next = 0.0
    assert lv.base() == pytest.approx(1.23 * 0.97, abs=0.004) and w.alarm(t, 0.02) is None
    t = blocks(lv, t, 60, 1.23 * 0.97, 0.003, rng)
    assert w.alarm(t, 0.02) is None
    # a dip within its noise is none, however large a part of it
    noisy = ModeWatch()
    t = blocks(noisy.signals["fringes"], 0.0, 60, 1.0, 0.2, rng)
    t = blocks(noisy.signals["fringes"], t, 3, 0.97, 0.2, rng)
    assert noisy.alarm(t, 0.02) is None
    w.reset(shg_prior=1.1)
    assert lv.base() == 1.1 and not lv.blocks and w.fringes.parts is None


def test_a_deep_dip_is_not_waited_out():
    w, rng = ModeWatch(), random.Random(4)
    lv = w.signals["shg"]
    t = blocks(lv, 0.0, 60, 1.23, 0.003, rng)
    assert lv.sharp()[0] == pytest.approx(0.0, abs=0.005) and w.alarm(t, 0.02) is None
    for k in range(Level.SHARP_N - 1):  # (8 % down, all at once: five readings of it are not yet enough...)
        t += 0.5
        lv.feed(t, 1.23 * 0.92 + rng.gauss(0.0, 0.003))
    assert w.alarm(t, 0.02) is None
    lv.feed(t + 0.5, 1.23 * 0.92)       # (...six are: three seconds, not the twenty its blocks would take)
    a = w.alarm(t + 0.5, 0.02)
    assert a is not None and a["dip"] == pytest.approx(0.08, abs=0.006) and a["level"] == pytest.approx(1.13, abs=0.01)
    assert lv.dip()[0] < 0.02           # (its blocks have not seen it yet)
    w2 = ModeWatch()                    # (a dip of 4 %, twice what is asked, waits for its blocks; one odd reading
    lv = w2.signals["shg"]              # far down is no dip at all)
    t = blocks(lv, 0.0, 60, 1.23, 0.003, rng)
    for k in range(Level.SHARP_N):
        t += 0.5
        lv.feed(t, 1.23 * 0.96)
    assert w2.alarm(t, 0.02) is None
    w3 = ModeWatch()
    lv = w3.signals["shg"]
    t = blocks(lv, 0.0, 60, 1.23, 0.003, rng)
    for k in range(2 * Level.SHARP_N):
        t += 0.5
        lv.feed(t, 0.2 if k == 3 else 1.23)
        assert w3.alarm(t, 0.02) is None


def test_the_fringes_amplitudes_serve_where_there_are_no_patterns():
    w = ModeWatch()
    assert w.pattern(1.0, None, [100, 900, 500]) == pytest.approx(0.8)  # (their contrast)
    assert w.pattern(1.0, None, None) is None and w.pattern(1.0, None, [0, 0]) is None


# ------------------------------------------------------- kept, its SHG signal
def shg_bench(seed, into=0.15, way=1, **hop):
    """An SHG master (near: its next mode close by), its DLC pro locking it to its cavity and its SHG cavity to it,
    drifted `into` of the way from where a second mode begins to where it hops, at the top of its mode (way +1) or
    the bottom: (rig, lock, p, what its watch would say)."""
    laser, _ = near_ecdl(seed)
    rig, locked = _locked(laser, seed, shg=True, lockpoints=True)
    p = HopParams(enabled=True, i_nom=I0, cavity=True, shg=True, opt_i=I0, opt_v=V0, opt_shg=rig.shg_usual,
                  shg_lock=rig.shg_usual, i_below=2.0, **hop)
    if into:
        edge, full = laser.zone(way)
        laser.nudge(laser.misalignment(laser.m, rig.V + locked.dv, rig.act) - way * (edge + into * (full - edge)))
    xs = []
    for _ in range(30):
        rig.t += 0.5
        locked.frequency(0.0)
        xs.append(locked._intra_now())
    level = sorted(xs)[15]
    alarm = {"signal": "shg", "base": rig.shg_usual, "level": level, "noise": 0.003, "dip": 1 - level / rig.shg_usual,
             "trend": 0.0, "way": 0}
    return rig, locked, p, alarm


def keeping(rig, p, alarm, cavity=True, lo=None, hi=None):
    a, b, top = _windows(p, I0)
    rec = Recenter(p, rig.I, rig.V, a if lo is None else lo, b if hi is None else hi, V0 - p.v_window,
                   V0 + p.v_window, cavity=cavity, i_top=top, keep=alarm)
    rig.due = rig.t + rig.dt
    return rig.run(rec), rec


def only_the_current(rig, start=I0, fine=Recenter.UP_STEP):
    """Nothing written but the current, in a keeping's small steps: its locks were left as they are."""
    names = {name for name, _ in rig.dlc.log}
    assert names <= {CURRENT}, names
    w = rig.dlc.written(CURRENT)
    assert all(abs(b - a) <= fine + 1e-9 for a, b in zip([start] + w, w))
    return w


@pytest.mark.parametrize("seed", [0, 1, 2, 3])
@pytest.mark.parametrize("way", [1, -1])
def test_a_second_mode_beginning_its_current_is_trimmed_with_its_locks_on(seed, way):
    rig, locked, p, alarm = shg_bench(seed, way=way)
    assert 0.03 < alarm["dip"] < 0.12 and 0.03 < rig.second_mode() < 0.12  # (a second mode has a part of the power)
    t0 = rig.t
    r, rec = keeping(rig, p, alarm)
    assert r.state == "done", r.error
    k = rec.kept
    assert k["mended"] and k["way"] == -way  # (the top of its mode: the current down; the bottom: up)
    assert rig.second_mode() == 0.0 and locked.locked() and abs(rig.off_mhz()) < 1.0
    w = only_the_current(rig)
    assert max(abs(x - I0) for x in w) <= Recenter.KEEP_MAX + p.keep_back + 1e-9
    assert way * (rig.I - I0) < -p.keep_back + 1e-9  # (keep_back further than where it was back up)
    assert rig.t - t0 < 12.0  # s
    assert "kept off the edge of its mode: its SHG signal had dipped" in r.result.summary
    assert any(n.startswith("kept off the edge of its mode") for n in rec.notes)
    assert rec.record["keep"]["mended"] and rec.record["keep"]["up_at"] == pytest.approx(rig.I + way * p.keep_back)


def test_the_wrong_way_first_it_turns_round_the_moment_its_signal_falls():
    rig, locked, p, alarm = shg_bench(1, way=1)
    alarm["way"] = 1  # (up mended it last time; this time that is the wrong way)
    r, rec = keeping(rig, p, alarm)
    w = only_the_current(rig)
    assert r.state == "done" and rec.kept["mended"] and rec.kept["way"] == -1
    assert max(w) - I0 <= Recenter.KEEP_PROBE + 1e-9  # (no further than a probe, the wrong way)
    assert max(w) - I0 < 0.8 * Recenter.KEEP_PROBE    # (not even that: it fell, no further)
    assert rig.second_mode() == 0.0 and locked.locked()


def test_a_signal_low_for_another_reason_is_left_alone_and_taken_for_what_it_is():
    rig, locked, p, alarm = shg_bench(2, into=0.0)
    locked.shg_h *= 0.95  # (its TA weaker, its SHG cavity's alignment: 5 % less, whatever the current)
    xs = []
    for _ in range(20):
        rig.t += 0.5
        xs.append(locked._intra_now())
    alarm.update(level=sorted(xs)[10], dip=1 - sorted(xs)[10] / alarm["base"])
    r, rec = keeping(rig, p, alarm)
    assert r.state == "done" and not rec.kept["mended"] and rec.kept["why"] == "the current makes no difference"
    w = only_the_current(rig)
    far = Recenter.KEEP_PROBES[-1] * Recenter.KEEP_PROBE
    assert rig.I == pytest.approx(I0) and max(abs(x - I0) for x in w) == pytest.approx(far)  # (tried, and back)
    assert "not its mode's doing" in r.result.summary and locked.locked()


def test_back_up_by_itself_nothing_is_moved():
    rig, locked, p, alarm = shg_bench(0, into=0.0)
    alarm.update(level=0.96 * alarm["base"], dip=0.04)  # (what its watch saw a moment ago)
    r, rec = keeping(rig, p, alarm)
    assert r.state == "done" and rec.kept["mended"] and rec.kept["why"] == "back up by itself"
    assert rig.dlc.log == [] and "back up by itself" in r.result.summary


def test_at_the_edge_of_its_window_it_says_so():
    rig, locked, p, alarm = shg_bench(3, way=1)
    r, rec = keeping(rig, p, alarm, lo=I0)  # (its window's bottom is where it is: down is where it would have to go)
    assert r.state == "done" and not rec.kept["mended"] and rec.kept["edge"] == "bottom"
    assert rig.I == pytest.approx(I0) and min(only_the_current(rig)) >= I0 - 1e-9
    assert "at the bottom of its window" in r.result.summary and "wants adjusting" in r.result.summary


def test_not_locked_or_hopping_on_the_way_it_fails_and_the_current_goes_back():
    rig, locked, p, alarm = shg_bench(1, way=1)
    locked.knock(30.0)
    r, rec = keeping(rig, p, alarm)
    assert r.state == "failed" and "not locked now" in r.error and rig.dlc.log == []
    rig, locked, p, alarm = shg_bench(1, way=1)
    laser, state = rig.laser, {"n": 0}

    def also(t, e):  # (three readings into it, its mode gives way: it hops, its cavity lock catches another peak)
        state["n"] += 1
        if state["n"] == 30:
            laser.nudge(-2.0 * laser.fsr / 3)
            locked.frequency(0.0)
        return None

    rig.also = also
    r, rec = keeping(rig, p, alarm)
    assert r.state == "failed" and not r.restored, r.error
    assert "keeping its mode" in r.error and "from its target (it hopped, or its lock lost it)" in r.error
    assert "the current left where it is" in r.error and 0.0 < abs(rig.I - I0) <= Recenter.KEEP_MAX  # (not put back:
    only_the_current(rig)  # that would take a laser that had not hopped off its target; its guard has it from here)


def test_its_current_set_by_hand_on_the_way_is_not_written_over():
    """Its set current is read back before each of a keeping's writes: someone at it meanwhile, and the keeping
    stops there, the current left as the hand has it (its next write, from where it was, would be a jump back)."""
    rig, locked, p, alarm = shg_bench(0, way=1)
    state = {"n": 0}

    def also(t, e):
        if abs(rig.I - I0) >= 0.03 and not state["n"]:  # (on its way: by hand, 0.2 mA on)
            state["n"] = len(rig.dlc.log)
            rig.dlc.params["laser1:dl:cc:current-set"] = state["i"] = round(rig.I - 0.2, 3)
        return None

    rig.also = also
    r, rec = keeping(rig, p, alarm)
    assert state["n"], "the hand never came"
    assert r.state == "failed" and r.error == "its current was set by hand meanwhile" and not r.restored
    assert rig.I == state["i"] and r.i_written == state["i"]  # (left as the hand has it)
    assert len(rig.dlc.log) <= state["n"] + 1  # (the write that was on its way, at most)
    rig, locked, p, alarm = shg_bench(0, way=1)  # (a controller that rounds what it is told is not a hand)
    rig.dlc.on_set["laser1:dl:cc:current-set"] = lambda v=None: rig.dlc.params.__setitem__(
        "laser1:dl:cc:current-set", round(rig.I, 2)) or rig._settle()
    r, rec = keeping(rig, p, alarm)
    assert r.state == "done" and rec.kept["mended"], r.error


def test_step_by_step_it_asks_before_it_moves_anything():
    rig, locked, p, alarm = shg_bench(0, way=-1, confirm=True)
    a, b, top = _windows(p, I0)
    rec = Recenter(p, rig.I, rig.V, a, b, V0 - p.v_window, V0 + p.v_window, cavity=True, i_top=top, keep=alarm)
    r = rig.run(rec, until=lambda r: r.asking)
    assert "has dipped" in r.asking and "slowly, its locks on" in r.asking and rig.dlc.log == []
    r.cancel("stopped")
    while r.busy:
        rig.step(r)
    assert rig.dlc.log == []


# ---------------------------------------- kept, its fringes on the wavemeter
class Fringed:
    """An ECDL with nothing but the wavemeter, locked to it by this program's lock on its piezo (which goes on during
    a keeping, as the engine has it), the wavemeter's interference patterns looked at with each reading."""

    def __init__(self, seed, into=0.3, way=1, dt=0.2, ki=None, gain=0.3):
        """ki: GHz/mA its current tunes it (as it comes: 0.1 to 0.5; the lab's 493 nm laser: 1.7), where its modes
        are picked kept; gain: the part of its error the lock makes good a reading."""
        laser, _, noise = far_ecdl(2000 + seed)
        laser.shift, laser.m = 0.0, 0
        laser.multimode = 0.2 * laser.fsr
        if ki is not None:
            laser.ki, laser.ksi = ki, ki - (laser.ki - laser.ksi)
        self.laser, self.rig = laser, Rig(laser, laser.freq(V0, I0), noise, seed=seed, dt=dt)
        self.p = HopParams(enabled=True, i_nom=I0, i_below=2.0)
        self.ccd, self.watch = SimCCD(seed=seed, amp_noise=3.0), ModeWatch()
        self.lock = Lock(LockParams(ki=gain / laser.kv, excursion=0.0))
        self.off, self.blind = 0.0, False  # MHz: the furthest it really was from its target; its patterns not read
        self.lock.engage(self.rig.V)
        self.rig.also = self.also
        t = self.run(60.0)  # (its fringes as they should be)
        if into:
            edge, full = laser.zone(way)
            laser.nudge(laser.misalignment(laser.m, self.rig.V, self.rig.act) - way * (edge + into * (full - edge)))
        self.run(40.0)

    def also(self, t, e):
        rig, laser = self.rig, self.laser
        self.off = max(self.off, abs(rig.off_mhz()))
        if math.isfinite(e) and abs(e) <= self.p.tolerance:
            u = self.lock.update(e)
            if u is not None:
                rig.set(offset=u)
        if self.blind:
            return None
        _, extra, _ = self.ccd.measure(laser.components(rig.V, rig.act), laser.power(rig.act), rig.act, rig.target,
                                       laser.f0, True)
        return self.watch.fringes.seen(extra["patterns"][0][1])

    def run(self, seconds):
        rig = self.rig
        end = rig.t + seconds
        while rig.t < end:
            rig._settle()
            t, e = rig.reading()
            q = self.also(t, e)
            if q is not None:
                self.watch.feed("fringes", t, q)
        return rig.t


@pytest.mark.parametrize("seed", [0, 1, 4])
@pytest.mark.parametrize("way", [1, -1])
def test_a_laser_locked_to_the_wavemeter_is_kept_by_its_fringes(seed, way):
    pytest.importorskip("numpy")
    b = Fringed(seed, way=way)
    rig = b.rig
    alarm = b.watch.alarm(rig.t, b.p.keep_dip)
    assert alarm is not None and alarm["signal"] == "fringes" and alarm["dip"] > b.p.keep_dip
    assert rig.second_mode() > 0.02
    v0 = rig.V
    r, rec = keeping(rig, b.p, alarm, cavity=False)
    assert r.state == "done", r.error
    assert rec.kept["mended"] and rec.kept["way"] == -way * (1 if b.laser.ki - b.laser.ksi > 0 else -1)
    assert rig.second_mode() == 0.0 and abs(rig.off_mhz()) < 30.0  # (on target all the while: its lock went on)
    assert abs(rig.V - v0) > 1e-3 and b.laser.m == 0                # (its piezo followed; the mode it was on)
    names = {name for name, _ in rig.dlc.log}
    assert names == {CURRENT, "laser1:scan:offset"}  # (the keeping's current, the lock's piezo: nothing else)
    assert "its fringes on the wavemeter had dipped" in r.result.summary


def test_fringes_that_fade_for_another_reason_are_not_its_modes_doing():
    pytest.importorskip("numpy")
    b = Fringed(2, into=0.0)
    rig = b.rig
    assert b.watch.alarm(rig.t, b.p.keep_dip) is None
    alarm = {"signal": "fringes", "base": 1.03, "level": 1.0, "noise": 0.001, "dip": 0.03, "trend": 0.0, "way": 0}
    r, rec = keeping(rig, b.p, alarm, cavity=False)  # (as if they had been 3 % deeper once)
    assert r.state == "done" and rec.kept["why"] == "the current makes no difference"
    assert rig.I == pytest.approx(I0) and b.laser.m == 0 and abs(rig.off_mhz()) < 30.0


# ---------------------------------------- what can go wrong while it is kept
def test_the_watch_looks_less_and_less_often_at_a_signal_that_comes_back_by_itself():
    w = ModeWatch()
    waits = []
    for _ in range(7):
        w.kept(100.0, True, way=0)  # (back up by itself: nothing moved)
        waits.append(w.next - 100.0)
    assert waits == [15.0, 30.0, 60.0, 120.0, 240.0, 300.0, 300.0] and w.kept_n == 0
    w.kept(100.0, True, way=-1)     # (kept: as often as it takes, again)
    assert w.next - 100.0 == ModeWatch.GAP and w.idle == 0.0 and w.kept_n == 1


def test_kept_what_its_signal_is_there_is_what_it_should_be_from_then_on():
    """Its signal goes a little with its current too (a laser's power): after a keeping has moved the current,
    what it should be is what it is there, back up, not what it was an hour ago at another current."""
    w, rng = ModeWatch(), random.Random(3)
    lv = w.signals["shg"]
    t = blocks(lv, 0.0, 60, 1.230, 0.003, rng)
    t = blocks(lv, t, 2, 1.230 * 0.97, 0.003, rng)
    assert w.alarm(t, 0.02) is not None
    w.kept(t, True, way=-1, level=1.215)  # (back up at 1.215 V, 0.1 mA lower: 1.2 % less light there)
    assert lv.base() == 1.215 and not lv.blocks
    t = blocks(lv, t + ModeWatch.GAP, 8, 1.215, 0.003, rng)
    assert w.alarm(t, 0.02) is None and lv.base() == pytest.approx(1.215, abs=0.003)  # (not 1.2 % down: as it is)
    t = blocks(lv, t, 2, 1.215 * 0.975, 0.003, rng)
    a = w.alarm(t, 0.02)
    assert a is not None and a["dip"] == pytest.approx(0.025, abs=0.004)  # (a dip from there is one)
    w.kept(t, True, way=0, level=1.0)     # (back up by itself: nothing moved, nothing taken anew)
    assert lv.base() == pytest.approx(1.215, abs=0.003)


def test_a_glitch_of_the_wavemeters_is_looked_at_again_and_it_goes_on():
    rig, locked, p, alarm = shg_bench(1, way=1)
    t0, read = rig.t, rig.reading

    def reading():  # (0.6 s of readings 300 MHz off, in the middle of its first ramp: the laser where it was)
        t, e = read()
        return t, e + (300.0 if t0 + 2.0 <= t < t0 + 2.6 else 0.0)

    rig.reading = reading
    r, rec = keeping(rig, p, alarm)
    assert r.state == "done" and rec.kept["mended"] and rig.second_mode() == 0.0, (r.error, rec.notes)
    assert any("for a moment" in n and "looked at again" in n for n in rec.notes)
    only_the_current(rig)


def hushed(rig, when):
    """From when() True on the wavemeter says nothing of the laser (its channel no longer read, the WLM stuck): time
    passes, and the runner is ticked as the window's timer would."""
    hear = rig._hear

    def _hear(runner, t, e, q):
        if when():
            runner.tick(t)
        else:
            hear(runner, t, e, q)

    rig._hear = _hear


def test_the_wavemeter_falling_silent_stops_a_keeping_where_it_is():
    pytest.importorskip("numpy")
    b = Fringed(0, way=1)
    rig = b.rig
    alarm = b.watch.alarm(rig.t, b.p.keep_dip)
    a, top_, top = _windows(b.p, I0)
    rec = Recenter(b.p, rig.I, rig.V, a, top_, V0 - b.p.v_window, V0 + b.p.v_window, i_top=top, keep=alarm)
    at = {}
    hushed(rig, lambda: abs(rig.I - I0) >= 0.07 and at.setdefault("i", rig.I) is not None)
    rig.due = rig.t + rig.dt
    r = rig.run(rec)
    assert r.state == "failed" and "no valid reading of it for" in r.error and "left where it is" in r.error
    assert not r.restored and abs(rig.I - at["i"]) <= 0.05  # (a second of ramp at most, unseen: not on to 0.55 mA)
    assert abs(rig.off_mhz()) < 30.0 and b.laser.m == 0     # (where the lock had it: on target still)


def test_readings_that_come_without_their_fringes_stop_it_too():
    pytest.importorskip("numpy")
    b = Fringed(1, way=1)
    rig = b.rig
    alarm = b.watch.alarm(rig.t, b.p.keep_dip)
    read, n = rig.reading, [0]

    def reading():
        n[0] += 1
        b.blind = n[0] > 30  # (from then on its patterns are not to be had: the readings go on)
        return read()

    rig.reading = reading
    i0 = rig.I
    r, rec = keeping(rig, b.p, alarm, cavity=False)
    assert r.state == "failed" and "came without their fringes" in r.error, r.error
    assert abs(rig.I - i0) <= Recenter.KEEP_MAX and abs(rig.off_mhz()) < 30.0


def test_a_hop_as_the_keeping_ends_is_not_called_kept():
    pytest.importorskip("numpy")
    b = Fringed(0, way=1)
    rig, laser = b.rig, b.laser
    alarm = b.watch.alarm(rig.t, b.p.keep_dip)
    a, hi, top = _windows(b.p, I0)
    rec = Recenter(b.p, rig.I, rig.V, a, hi, V0 - b.p.v_window, V0 + b.p.v_window, i_top=top, keep=alarm)
    level = rec.keep_level

    def keep_level(shg, n):
        if n == Recenter.KEEP_N:  # (its last look: the laser hops as it begins)
            laser.nudge(-1.02 * laser.fsr)
            rig._settle()
        return (yield from level(shg, n))

    rec.keep_level = keep_level
    rig.due = rig.t + rig.dt
    r = rig.run(rec)
    assert laser.m != 0 and r.state == "failed" and not (rec.kept or {}).get("mended")
    assert "from its target (it hopped, or its lock lost it)" in r.error and "left where it is" in r.error


def test_a_signal_that_only_goes_with_the_current_is_not_mended():
    """Its SHG signal 3.5 % low for another reason, and rising a little with the current as any laser's power does
    (1 % a mA), read quietly: climbing that slope is not mending it. The current goes back where it was."""
    rig, locked, p, alarm = shg_bench(2, into=0.0)
    locked.shg_slope = 0.01
    locked.shg_h *= 0.965
    locked.rng.gauss = lambda mu, sigma: 0.0002 * (2 * locked.rng.random() - 1)  # (read quietly: 0.02 %)
    xs = []
    for _ in range(20):
        rig.t += 0.5
        xs.append(locked._intra_now())
    alarm.update(level=sorted(xs)[10], dip=1 - sorted(xs)[10] / alarm["base"], noise=0.0002)
    r, rec = keeping(rig, p, alarm)
    assert r.state == "done" and not rec.kept["mended"], rec.kept
    assert rec.kept["why"] in ("it did not come back up", "the current makes no difference")
    assert rig.I == pytest.approx(I0) and locked.locked() and rig.second_mode() == 0.0
    assert max(abs(x - I0) for x in only_the_current(rig)) <= Recenter.KEEP_MAX + p.keep_back + 1e-9


def test_the_right_way_but_its_window_ends_it_stays_at_its_edge_and_says_so():
    rig, locked, p, alarm = shg_bench(3, into=0.3, way=1)
    r, rec = keeping(rig, p, alarm, lo=I0 - 0.03)  # (down is the way; 0.03 mA of it is all its window has)
    k = rec.kept
    assert r.state == "done" and not k["mended"] and k["edge"] == "bottom" and k["way"] == -1, (k, r.error)
    assert rig.I == pytest.approx(I0 - 0.03) and min(only_the_current(rig)) >= I0 - 0.03 - 1e-9  # (left there)
    assert "at the bottom of its window" in r.result.summary and "wants adjusting" in r.result.summary


def test_a_narrow_mode_is_left_in_the_middle_of_where_its_signal_is_as_it_should_be():
    """A mode with a second one beginning either side, 0.07 GHz from its middle (0.08 mA of current in all): back
    up going down, the signal dips again before the 0.12 mA further asked for here. No further, then: back to the
    middle of where it was up."""
    laser, _ = near_ecdl(2)
    laser.hysteresis, laser.multimode = 0.1 * laser.fsr, laser.fsr / 2 - 0.07
    rig, locked = _locked(laser, 2, shg=True, lockpoints=True)
    p = HopParams(enabled=True, i_nom=I0, cavity=True, shg=True, shg_lock=rig.shg_usual, i_below=2.0, keep_back=0.12)
    edge, full = laser.zone(1)
    assert edge == pytest.approx(0.07)
    laser.nudge(laser.misalignment(laser.m, rig.V + locked.dv, rig.act) - (edge + 0.1 * (full - edge)))
    xs = []
    for _ in range(30):
        rig.t += 0.5
        locked.frequency(0.0)
        xs.append(locked._intra_now())
    level = sorted(xs)[15]
    alarm = {"signal": "shg", "base": rig.shg_usual, "level": level, "noise": 0.003, "dip": 1 - level / rig.shg_usual,
             "trend": 0.0, "way": 0}
    r, rec = keeping(rig, p, alarm)
    assert r.state == "done" and rec.kept["mended"] and rec.kept["way"] == -1, (r.error, rec.kept, rec.notes)
    a, b = rec.kept["narrow"]
    assert 0.03 < abs(a - b) < 0.1  # mA: where it was up
    assert rig.I == pytest.approx(0.5 * (a + b), abs=0.006) and rig.second_mode() == 0.0
    assert abs(laser.misalignment(laser.m, rig.V + locked.dv, rig.act)) < 0.6 * edge  # (near its middle)
    assert min(only_the_current(rig)) > I0 - 0.2  # (not the 0.12 mA further: it dipped again before)
    assert any("its mode is narrow here" in n for n in rec.notes) and locked.locked()


def test_a_signal_that_dips_and_comes_back_time_and_again_has_its_current_taken_a_little_further_in():
    """At the very edge of its mode a second mode comes and goes: its watch says so, and each time its signal is
    back up when looked at. The second time running, the current goes keep_back the way that mended it last (its
    modes drift one way for hours); if that brings the signal down, back."""
    rig, locked, p, alarm = shg_bench(1, into=0.005, way=1)  # (where a second mode begins, at the top of its mode)
    alarm.update(level=0.97 * alarm["base"], dip=0.03, way=-1, again=True)
    r, rec = keeping(rig, p, alarm)
    k = rec.kept
    assert r.state == "done" and k["mended"] and k["way"] == -1 and k["why"] == "coming and going", (k, r.error)
    assert rig.I == pytest.approx(I0 - p.keep_back) and rig.second_mode() == 0.0 and locked.locked()
    assert "dips and comes back up by itself, time and again" in r.result.summary
    only_the_current(rig)
    rig, locked, p, alarm = shg_bench(1, into=0.005, way=1)
    alarm.update(level=0.97 * alarm["base"], dip=0.03, way=1, again=True)  # (up mended it last: now the wrong way)
    r, rec = keeping(rig, p, alarm)
    assert r.state == "done" and rec.kept["way"] == 0 and rec.kept["why"] == "back up by itself"
    assert rig.I == pytest.approx(I0) and max(only_the_current(rig)) <= I0 + p.keep_back + 1e-9  # (tried, and back)
    assert any("brings it down" in n for n in rec.notes)
    rig, locked, p, alarm = shg_bench(1, into=0.005, way=1)
    alarm.update(level=0.97 * alarm["base"], dip=0.03, way=-1, again=False)  # (the first time: nothing moved)
    r, rec = keeping(rig, p, alarm)
    assert r.state == "done" and rec.kept["why"] == "back up by itself" and rig.dlc.log == []


@pytest.mark.parametrize("dt,gain", [(1.0, 0.3), (0.1, 0.05)])
def test_a_laser_its_lock_follows_slowly_is_ramped_no_faster_than_that(dt, gain):
    """A wavemeter that reads it once a second, or a lock that makes good a twentieth of its error a reading, and a
    laser whose current tunes it 1.5 GHz/mA (the lab's 493 nm laser's 1.7): the keeping's ramps go a reading's
    worth a reading, and wait where the lock falls behind: it is on target all the while, and kept."""
    pytest.importorskip("numpy")
    b = Fringed(1, way=1, dt=dt, ki=-1.5, gain=gain)
    rig = b.rig
    alarm = b.watch.alarm(rig.t, b.p.keep_dip)
    assert alarm is not None
    b.off, t0 = 0.0, rig.t
    r, rec = keeping(rig, b.p, alarm, cavity=False)
    assert r.state == "done" and rec.kept["mended"], (r.error, rec.notes)
    assert b.off < b.p.tolerance and rig.second_mode() == 0.0 and b.laser.m == 0  # (never off its target)
    assert rig.t - t0 > 25.0  # s (not the six or so of a laser read ten times a second by a lock that keeps up)


# ------------------------------------------------- hours of drift (trials.py)
@pytest.mark.parametrize("kind,seed,hours", [("shg", 1, 2.0), ("shg", 5, 2.0), ("dl", 1, 1.5), ("dl", 3, 3.5),
                                             ("red", 1, 1.0), ("blue", 2, 1.0)])
def test_drifting_towards_its_hop_it_hops_left_alone_and_does_not_kept(kind, seed, hours):
    pytest.importorskip("numpy")
    alone = keep_trial(kind, seed, hours=hours, keep=False)
    kept = keep_trial(kind, seed, hours=hours, keep=True)
    assert alone.hop is not None and alone.hop < hours and alone.kept == 0
    assert kept.hop is None and kept.kept >= 1 and kept.failed == 0, kept.notes
    assert kept.off_rms < 15.0 and kept.off_max < 100.0  # MHz, as it really was: on target all the while (its lock
    #                                                       went on; within what counts as off target at the worst)


# ------------------------------------------------------------ through the engine
@pytest.fixture
def brisk(monkeypatch):
    """A watch that makes up its mind in seconds, not minutes (its blocks 1 s of readings)."""
    monkeypatch.setattr(Level, "BLOCK_S", 1.0)
    monkeypatch.setattr(Level, "BLOCK_MIN", 2)
    monkeypatch.setattr(ModeWatch, "GAP", 2.0)


def into_its_zone(sim_laser, part=0.25, way=1):
    """The simulated laser's mode drifts to where a second mode has begun (part of the way to where it hops)."""
    with sim_laser._mutex:
        e = sim_laser.ecdl
        edge, full = e.zone(way)
        e.nudge(e.misalignment(e.m, *sim_laser._vi()) - way * (edge + part * (full - edge)))


def second_mode(sim_laser) -> float:
    with sim_laser._mutex:
        comps = sim_laser.ecdl.components(*sim_laser._vi())
    return min(p for p, _ in comps) if len(comps) > 1 else 0.0


def test_the_engine_keeps_an_shg_laser_off_the_edge_of_its_mode(brisk):
    from test_engine import hop_engine, wait_for
    from wmlock import view
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    for las in demo.lasers:
        las.drift = 0.0
    try:
        dlc, shg = demo.dlcs[2], demo.laser(demo.SHG)
        e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, keep=True))
        try:
            e.engage(demo.SHG)
            assert wait_for(lambda: ch.guard.armed and ch.cavity_on and ch.shg_txt == "locked", 5)
            assert wait_for(lambda: ch.hop.shg_lock > 0 and ch.watch.signals["shg"].now() is not None, 30)
            assert e._keep_due(ch, 1e9) is None and view.keep_text(ch) == ""  # (as it should be: nothing to do)
            relocks, i0 = shg.lock.relocks, dlc.current(1)
            into_its_zone(shg, 0.25)
            assert 0.05 < second_mode(shg) < 0.2
            assert wait_for(lambda: ch.keep_said.startswith("kept off the edge of its mode"), 40), ch.keep_said
            assert wait_for(lambda: ch.state == "on" and ch.runner is None, 5)
            assert second_mode(shg) == 0.0 and shg.lock.locked() and shg.lock.relocks == relocks  # (never unlocked)
            assert 0.03 < abs(dlc.current(1) - i0) < 0.6 and ch.message == ""
            assert ch.note.startswith("its SHG signal dipping") and "kept off the edge of its mode" in ch.note
            assert ch.last["ok"] and ch.last["keep"]["mended"] and not ch.keep_edge
            assert wait_for(lambda: ch.guard.armed, 5) and ch.hop.shg_lock > 0  # (watched again, nothing forgotten)
            assert len(ch.guard.recent) == 0  # (not a recovery: none counted)
            text = view.notes(e, demo.SHG)
            assert "kept off the edge of its mode" in text
            # switched off (HopParams.keep): left alone, whatever its signal does
            p = ch.hop
            p.keep = False
            e.set_hop_params(demo.SHG, p)
            said = ch.keep_said
            into_its_zone(shg, 0.25)
            import time
            time.sleep(6.0)
            assert ch.keep_said == said and second_mode(shg) > 0.05 and ch.state == "on"
        finally:
            e.stop()
    finally:
        demo.close()


def test_the_engine_keeps_a_laser_it_locks_to_the_wavemeter_by_its_fringes(brisk):
    pytest.importorskip("numpy")
    from test_engine import hop_engine, wait_for
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    for las in demo.lasers:
        las.drift = 0.0
    try:
        dlc, las = demo.dlcs[0], demo.laser(1)
        e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, keep=True))
        try:
            e.engage(1)
            assert wait_for(lambda: ch.state == "on" and ch.guard.armed, 5)
            # its readings come in detail now (their patterns), and what is left of its fringes is watched
            assert wait_for(lambda: ch.fringes is not None and ch.watch.signals["fringes"].now() is not None, 40)
            assert ch.fringes == pytest.approx(1.0, abs=0.02)
            i0, v0 = dlc.current(1), dlc.offset(1)
            into_its_zone(las, 0.3)
            assert second_mode(las) > 0.05
            assert wait_for(lambda: ch.keep_said.startswith("kept off the edge of its mode"), 60), ch.keep_said
            assert wait_for(lambda: ch.state == "on" and ch.runner is None and ch.lock.active, 5)
            assert "its fringes on the wavemeter had dipped" in ch.keep_said
            assert second_mode(las) == 0.0 and las.ecdl.m == 0 and abs(dlc.current(1) - i0) > 0.03
            assert wait_for(lambda: abs(ch.error_mhz) < 5, 5)  # (on target: its lock held it all the while)
            assert abs(dlc.offset(1) - v0) > 1e-3 and len(ch.guard.recent) == 0 and ch.message == ""
            assert ch.guard.armed  # (its guard never let go: armed as it was)
            # it hops in the middle of a keeping: the keeping stops (the current put back), and its guard, still
            # armed, has it recovered; its lock did not follow it there with its piezo meanwhile
            assert wait_for(lambda: ch.watch.next <= __import__("time").monotonic(), 10)
            into_its_zone(las, 0.3)
            assert wait_for(lambda: ch.keeping is not None and ch.runner is not None, 60)
            v1 = dlc.offset(1)
            las.hop_mode(+1)
            assert wait_for(lambda: ch.keep_said.startswith("its mode not kept"), 20), (ch.note, ch.keep_said)
            assert "from its target (it hopped, or its lock lost it)" in ch.keep_said
            assert wait_for(lambda: len(ch.guard.recent) == 1 and "jumped" in ch.hop_why, 10), (ch.state, ch.hop_why)
            assert abs(dlc.offset(1) - v1) < 0.5  # V (6 GHz followed would have been 15 V of piezo)
            assert wait_for(lambda: ch.state == "on" and ch.lock.active and "recentered" in ch.note, 60), ch.message
            assert wait_for(lambda: abs(ch.error_mhz) < 5, 10) and len(ch.guard.recent) == 1
        finally:
            e.stop()
    finally:
        demo.close()


def test_the_engine_keeps_a_slave_its_pll_holds_its_piezo_moved_along(brisk, tmp_path):
    """Sirius, an SLS PLL holding it on its beat point, its piezo trimmed: how its fast output follows its current is
    learned for it (as for its upkeep), and a second mode beginning, its current is trimmed with its piezo moved
    along: its PLL holds it all the while."""
    from test_sls import make_engine, misalignment, no_slow_servo, wait_for

    demo = no_slow_servo(shg=True)
    try:
        e = make_engine(demo, tmp_path, beat=0.0, servo=1, trim=True, trim_gain=0.4, hop_kw={"shg": True, "keep": True})
        e.TRIM_S = 1.5
        s = e.channels[demo.SLAVE]
        try:
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.sls.trim_cur, 60), s.trim_said  # (wanted by the keeping: no upkeep is set)
            assert wait_for(lambda: s.hop.shg_lock > 0 and s.watch.signals["shg"].now() is not None, 40)
            slave = demo.laser(demo.SLAVE)
            v0 = demo.sister.offset(1)
            into_its_zone(slave, 0.25)
            assert demo.slave_shg.main() < 0.95
            assert wait_for(lambda: s.keep_said.startswith("kept off the edge of its mode"), 40), (s.note, s.keep_said)
            assert wait_for(lambda: s.state == "on" and s.runner is None, 5)
            assert demo.slave_shg.main() == 1.0 and demo.pll.losses == 0 and s.sls_guard.stage == "hold"
            assert s.note.startswith("kept off the edge of its mode") and "its piezo with it" in s.note
            assert abs(demo.sister.offset(1) - v0) > 0.01  # V (its piezo went with its current)
        finally:
            e.stop()
    finally:
        demo.close()


def kept_demo():
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    for las in demo.lasers:
        las.drift = 0.0
    return demo


def test_asked_to_unlock_while_its_mode_is_kept_it_is_unlocked_once_that_has_stopped(brisk):
    """Its lock is left on during a keeping: what unlocks it meanwhile (its channel no longer measured) must still
    do so."""
    pytest.importorskip("numpy")
    from test_engine import hop_engine, wait_for

    demo = kept_demo()
    try:
        dlc, las = demo.dlcs[0], demo.laser(1)
        e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, keep=True))
        try:
            e.engage(1)
            assert wait_for(lambda: ch.state == "on" and ch.guard.armed, 5)
            assert wait_for(lambda: ch.fringes is not None and ch.watch.signals["fringes"].now() is not None, 40)
            into_its_zone(las, 0.3)
            assert wait_for(lambda: ch.keeping is not None and ch.runner is not None and ch.keep_locked, 60)
            e.on_active([2, 3, 4, 5, 6, 7])  # (its 'Use' unticked for good in the WLM software)
            assert wait_for(lambda: ch.runner is None, 30)
            assert ch.state == "off" and not ch.lock.active and ch.message == "unlocked: channel no longer measured"
            assert ch.note.startswith("its mode not kept: unlocked: channel no longer measured")
            assert not ch.keep_locked and ch.keeping is None and ch.unlock_why is None
        finally:
            e.stop()
    finally:
        demo.close()


def test_a_hand_at_its_current_while_its_mode_is_kept_stops_the_keeping(brisk):
    """Someone sets its current at the DLC pro in the middle of a keeping: the keeping stops and leaves the current
    as the hand has it (it would otherwise write its next step from where it was, a jump back); locked as it was."""
    pytest.importorskip("numpy")
    import time

    from test_engine import hop_engine, wait_for

    demo = kept_demo()
    try:
        dlc, las = demo.dlcs[0], demo.laser(1)
        e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, keep=True))
        try:
            e.engage(1)
            assert wait_for(lambda: ch.state == "on" and ch.guard.armed, 5)
            assert wait_for(lambda: ch.fringes is not None and ch.watch.signals["fringes"].now() is not None, 40)
            i0 = dlc.current(1)
            into_its_zone(las, 0.3)
            assert wait_for(lambda: ch.keeping is not None and ch.runner is not None
                            and abs(dlc.current(1) - i0) > 0.01, 60)  # (under way: its current on the move)
            dlc.execute("(param-set! 'laser1:dl:cc:current-set %.3f)" % (i0 - 0.3))  # (by hand, at its DLC pro)
            assert wait_for(lambda: ch.runner is None, 5)
            assert ch.keep_said.startswith("its mode not kept: its current was set by hand meanwhile"), ch.keep_said
            time.sleep(1.5)
            assert dlc.current(1) == pytest.approx(i0 - 0.3, abs=0.011)  # (left as the hand has it: a write on its
            assert ch.state == "on" and ch.lock.active and ch.keeping is None  # way at most; locked as it was)
            assert ch.current_set == pytest.approx(dlc.current(1), abs=1e-3)
        finally:
            e.stop()
    finally:
        demo.close()


def test_after_a_keeping_its_current_still_follows_its_piezo_to_a_new_setpoint(brisk):
    """A laser whose feedforward adds nothing: its lock moves its current along with its piezo as it tunes it
    (engine.Follow). Set aside while its mode is kept (the keeping moves its current), it is back afterwards, from
    where the keeping left the current."""
    pytest.importorskip("numpy")
    from test_engine import detune, ff_adds_nothing, hop_engine, wait_for

    demo = kept_demo()
    try:
        dlc, las = demo.dlcs[0], demo.laser(1)
        ff_adds_nothing(dlc, 1)
        e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, keep=True, ff_set=-0.6, ff_act=0.0))
        try:
            e.engage(1)
            assert wait_for(lambda: ch.state == "on" and ch.guard.armed and ch.follow is not None, 5)
            f = ch.follow
            assert wait_for(lambda: ch.fringes is not None and ch.watch.signals["fringes"].now() is not None, 40)
            into_its_zone(las, 0.3)
            assert wait_for(lambda: ch.runner is not None and ch.keeping is not None, 60)
            assert ch.follow is None and ch.follow_kept is f  # (set aside)
            assert wait_for(lambda: ch.keep_said.startswith("kept off the edge of its mode"), 60), ch.keep_said
            assert wait_for(lambda: ch.state == "on" and ch.runner is None and ch.lock.active, 5)
            assert ch.follow is f and ch.follow_kept is None and f.at == pytest.approx(dlc.current(1), abs=1e-3)
            assert wait_for(lambda: abs(ch.error_mhz) < 5, 10)
            v0, i0 = dlc.offset(1), dlc.current(1)
            e.set_setpoint(1, detune(ch, 400))
            assert wait_for(lambda: abs(ch.error_mhz) < 10, 20)
            assert wait_for(lambda: not f.tuning, 10)
            dv = dlc.offset(1) - v0
            assert abs(dv) > 0.5 and dlc.current(1) - i0 == pytest.approx(-0.6 * dv, abs=0.03)  # (moved along)
        finally:
            e.stop()
    finally:
        demo.close()


def test_a_laser_its_dlc_pro_locks_left_on_another_peak_by_a_keeping_is_relocked_at_once(brisk, monkeypatch):
    """Its cavity lock ends up a peak further as the keeping takes its last look (its SHG signal as good there):
    the keeping does not call that kept, and the laser, watched again, is relocked at once."""
    from test_engine import hop_engine, wait_for
    from wmlock import modehop

    demo = kept_demo()
    hook = {"armed": False, "done": False}
    level = modehop.Recenter.keep_level

    def keep_level(self, shg, n):
        if hook["armed"] and n == self.KEEP_N and not hook["done"]:  # (its last look)
            hook["done"] = True
            hook["lock"].jump(1.0)  # (a peak further: sim.SimLocked.jump)
        return (yield from level(self, shg, n))

    monkeypatch.setattr(modehop.Recenter, "keep_level", keep_level)
    try:
        dlc, shg = demo.dlcs[2], demo.laser(demo.SHG)
        hook["lock"] = shg.lock
        e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, keep=True))
        try:
            e.engage(demo.SHG)
            assert wait_for(lambda: ch.guard.armed and ch.cavity_on and ch.shg_txt == "locked", 5)
            assert wait_for(lambda: ch.hop.shg_lock > 0 and ch.watch.signals["shg"].now() is not None, 30)
            hook["armed"] = True
            into_its_zone(shg, 0.25)
            assert wait_for(lambda: ch.keep_said != "" and hook["done"], 60), (ch.keep_said, ch.note)
            assert ch.keep_said.startswith("its mode not kept") and "from its target" in ch.keep_said
            assert wait_for(lambda: ch.state == "on" and ch.guard.armed and abs(ch.error_mhz) < 20, 40), ch.message
            assert re.search(r"off by \+1\.(49|50)\d GHz", ch.note) and "relocked" in ch.note  # (looked at, and
            #                                                                    relocked: no hop called)
            assert len(ch.guard.recent) == 0 and shg.lock.locked()
        finally:
            e.stop()
    finally:
        demo.close()


def test_a_slaves_guard_acting_while_its_mode_is_kept_recovers_it_as_ever(brisk, tmp_path):
    """Sirius kept, its master's lock drops and it goes dark: its guard stops the keeping and calls its recovery,
    which is one (its current jumped up at once), not another keeping."""
    from test_sls import make_engine, no_slow_servo, ta_on, wait_for

    demo = no_slow_servo(shg=True)
    try:
        e = make_engine(demo, tmp_path, beat=0.0, servo=1, trim=True, trim_gain=0.4, trim_cur=-0.25,
                        hop_kw={"shg": True, "keep": True})
        e.TRIM_S, e.BEAT_LEARN_S = 1.5, 2.0
        s, slave = e.channels[demo.SLAVE], demo.laser(demo.SLAVE)
        seen, begin = [], e._begin_recovery

        def began(ch, *args):  # (what each recovery is made with)
            begin(ch, *args)
            if ch is s and ch.runner is not None:
                seen.append((ch.hop_why, ch.runner.rec.upkeeping, ch.runner.rec.keeping is not None))

        e._begin_recovery = began
        try:
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.sls_guard is not None and s.sls_guard.stage == "hold", 10)
            assert wait_for(lambda: s.hop.shg_lock > 0 and s.watch.signals["shg"].now() is not None and s.beat_ok, 60)
            into_its_zone(slave, 0.25)
            assert wait_for(lambda: s.keeping is not None and s.runner is not None, 60), (s.note, s.keep_said)
            slave.stop_lasing()
            demo.pdh.drop(0.5)
            assert wait_for(lambda: not demo.pll.manual, 3)
            assert wait_for(lambda: s.keeping is None and s.note.startswith("its mode not kept"), 3), s.note
            assert wait_for(lambda: len(seen) >= 2, 8), seen
            assert seen[0][1:] == (True, True) and seen[1][1:] == (False, False)  # (a keeping; then a recovery)
            assert wait_for(lambda: slave.ecdl.lasing, 8) and ta_on(demo)
        finally:
            e.stop()
    finally:
        demo.close()
