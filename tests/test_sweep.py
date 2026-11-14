"""Ramps instead of steps (modehop.Sweep, HopParams.smooth, fine, fine_v: ticked here, off as they come), on the
simulated laser in simulated time
(trials.Rig: an operation on the controller takes its 2 ms, the wavemeter reads every dt, the runner is woken for each
step of a ramp as the engine's ticker wakes it): what a Sweep is, what the recoveries make of it (faster than a step
and a reading, the same safety rules), and a hop seen at once by where the readings are (Guard's jump)."""
import math
import statistics

import pytest

from wmlock import modehop
from wmlock.modehop import STEP_MAX, Guard, HopParams, Move, Recenter, Runner, Sample, Sweep, Swept
from wmlock.sim import ECDL
from wmlock.trials import I0, V0, Rig, make_case, recover

CURRENT, OFFSET = "laser1:dl:cc:current-set", "laser1:scan:offset"


class Script(Recenter):
    """A procedure of the test's own in a recovery's place: steps(rec) yields its requests."""

    def __init__(self, steps, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.steps = steps

    def run(self):
        return (yield from self.steps(self))


def laser() -> ECDL:
    """A quiet ECDL in the middle of a wide mode: -0.25 GHz/mA, 0.4 GHz/V, no hop within 2 mA."""
    return ECDL(473.0, fsr=12.0, kv=0.4, ki=-0.25, ksi=-2.0, hysteresis=0.5, multimode=0.0, seed=1)


def bench(dt=0.1, p=None, ff=None, **kw):
    las = laser()
    rig = Rig(las, las.freq(V0, I0), noise=0.5, dt=dt, **kw)
    if ff is not None:
        rig.dlc.params["laser1:dl:cc:feedforward-enabled"] = True
        rig.dlc.params["laser1:dl:cc:feedforward-factor"] = ff
    return rig, p or HopParams(enabled=True, i_nom=I0, smooth=True)


def run(rig, p, steps, lo=I0 - 2.0, hi=I0 + 2.0, v=10.0):
    rec = Script(steps, p, rig.I, rig.V, lo, hi, rig.V - v, rig.V + v)
    if rig.dlc.params["laser1:dl:cc:feedforward-enabled"]:
        rec.ff = rig.dlc.params["laser1:dl:cc:feedforward-factor"]  # (as current_setup would have found it)
    return rig.run(rec), rec


def writes(rig, name=CURRENT):
    return rig.dlc.written(name)


def steps_of(start, values):
    return [abs(b - a) for a, b in zip([start] + values, values)]


def test_a_sweep_ramps_steadily_in_fine_writes_and_keeps_every_reading():
    rig, p = bench()
    got = []

    def steps(rec):
        got.append((yield Sweep(current=I0 - 1.0, rate=1.0)))

    t0 = rig.t
    r, _ = run(rig, p, steps)
    assert r.state == "done"
    sw = got[0]
    assert isinstance(sw, Swept) and not sw.stopped and sw.current == pytest.approx(I0 - 1.0)
    w = writes(rig)
    assert len(w) == 50 and max(steps_of(I0, w)) <= p.fine + 1e-9  # (1 mA in writes of 0.02 mA)
    assert w == sorted(w, reverse=True) and w[-1] == pytest.approx(I0 - 1.0)
    reads = [s for s in sw.samples if s.e is not None]
    on_way, rest = [s for s in reads if not s.rest], [s for s in reads if s.rest]
    assert 9 <= len(on_way) <= 11 and len(rest) == 1  # (a reading every 0.1 s for a second, then one at rest)
    assert rig.t - t0 == pytest.approx(1.0 + 0.1, abs=0.12)  # (no waiting but for the last)
    # every reading with the current as it was when it was taken: the laser 250 MHz a mA, read to a few MHz
    for s in reads:
        assert isinstance(s, Sample) and s.v == V0
        assert s.e == pytest.approx(-250.0 * (s.i - I0), abs=250.0 * p.fine + 3.0)
    assert [s.i for s in reads] == sorted((s.i for s in reads), reverse=True)
    assert rest[0].i == pytest.approx(I0 - 1.0) and sw.gap == pytest.approx(0.1, abs=0.01)


def test_a_sweep_paced_by_the_readings_has_one_every_so_many_mA():
    rig, p = bench(dt=0.25)
    got = []

    def steps(rec):
        for _ in range(4):  # (the wavemeter's pace heard first)
            yield modehop.Read()
        got.append((yield Sweep(current=I0 + 1.0, per=0.1)))

    r, _ = run(rig, p, steps)
    sw = got[0]
    reads = [s for s in sw.samples if s.e is not None and not s.rest]
    assert r.state == "done" and 9 <= len(reads) <= 11  # (0.1 mA a reading: 0.4 mA/s at 4 readings a second)
    gaps = [b.i - a.i for a, b in zip(reads, reads[1:])]
    assert statistics.median(gaps) == pytest.approx(0.1, abs=0.021)
    # the wavemeter slower than a sweep may wait for: no slower than RATE's least; faster: no faster than its most
    slow, _ = bench(dt=20.0)
    slow_r = Runner(Script(lambda rec: iter(()), HopParams(enabled=True, i_nom=I0, smooth=True), I0, V0, I0 - 2,
                           I0 + 2, 60, 80),
                    slow, 0.0)
    slow_r._gaps.extend([20.0] * 5)
    slow_r._sweep_begin(Sweep(current=I0 + 1.0, per=0.1))
    assert slow_r._sweep["dt"] * slow_r._sweep["n"] == pytest.approx(1.0 / Runner.RATE[0])


def test_a_sweeps_watch_stops_it_where_things_are_not_as_expected():
    rig, p = bench()
    got, asked = [], []

    def watch(s):
        asked.append(s)
        return s.e is not None and abs(s.e) > 100.0  # (400 MHz would be the end: not past 100)

    def steps(rec):
        got.append((yield Sweep(current=I0 - 1.6, rate=1.0, watch=watch)))

    t0 = rig.t
    r, _ = run(rig, p, steps)
    sw = got[0]
    assert r.state == "done" and sw.stopped
    last = sw.samples[-1]
    assert abs(last.e) > 100.0 and not last.rest and len(asked) == len(sw.samples)
    # stopped at once: the current no further than the write on its way when the reading came
    assert I0 - 0.4 - 0.1 - 2 * p.fine <= sw.current <= I0 - 0.4 + 2 * p.fine
    assert sw.current == pytest.approx(writes(rig)[-1]) and abs(sw.current - last.i) <= p.fine + 1e-9
    assert rig.t - t0 < 0.65  # (not the 1.6 s the whole way would have taken)


def test_a_sweep_reads_parameters_on_the_way():
    rig, p = bench()
    got = []

    def steps(rec):
        got.append((yield Sweep(current=I0 + 0.5, rate=0.5, names=("dl:cc:current-act",), every=0.1, rest=False)))

    r, _ = run(rig, p, steps)
    sw = got[0]
    vals = [s for s in sw.samples if s.values is not None]
    assert r.state == "done" and 9 <= len(vals) <= 11  # (every 0.1 s for a second)
    for s in vals:
        assert s.e is None and s.values[0] == pytest.approx(s.i, abs=1e-6)  # (as the current was then)
    assert not any(s.rest for s in sw.samples)


def test_current_and_piezo_ramp_together_on_a_straight_line_within_their_windows():
    rig, p = bench()
    got = []

    def steps(rec):
        got.append((yield Sweep(current=I0 + 1.0, offset=V0 + 4.0, rate=2.0)))
        got.append((yield Sweep(current=I0 + 9.0, rate=5.0, rest=False)))   # (beyond its window: to its edge)
        got.append((yield Sweep(offset=V0 + 50.0, rate=20.0, rest=False)))  # (likewise the piezo)
        got.append((yield Sweep(current=I0 + 2.0, rate=1.0)))               # (where it is already: nothing to do)

    r, _ = run(rig, p, steps)
    assert r.state == "done"
    line = []
    i, v = I0, V0
    for name, value in rig.dlc.log:
        if name == CURRENT:
            i = value
        elif name == OFFSET:
            v = value
        line.append((i, v))
    first = [(i, v) for i, v in line if v <= V0 + 4.0 + 1e-9 and i <= I0 + 1.0 + 1e-9]
    assert max(abs((v - V0) - 4.0 * (i - I0)) for i, v in first) <= 4.0 * p.fine + 1e-9  # (4 V a mA: on the line)
    assert max(steps_of(V0, writes(rig, OFFSET))) <= p.fine_v + 1e-9
    assert got[0].current == pytest.approx(I0 + 1.0) and got[0].offset == pytest.approx(V0 + 4.0)
    assert got[1].current == pytest.approx(I0 + 2.0) and got[2].offset == pytest.approx(V0 + 10.0)
    assert max(writes(rig)) <= I0 + 2.0 + 1e-9 and max(writes(rig, OFFSET)) <= V0 + 10.0 + 1e-9
    assert [s.rest for s in got[3].samples] == [True]


def test_with_the_feedforward_on_a_piezo_ramp_moves_the_measured_current_by_small_steps():
    rig, p = bench(ff=-0.5)  # (half a mA a volt: 0.1 V a write would be 0.05 mA of measured current)

    def steps(rec):
        yield Sweep(offset=V0 + 2.0, rate=2.0)

    r, _ = run(rig, p, steps)
    assert r.state == "done"
    dv = steps_of(V0, writes(rig, OFFSET))
    assert max(dv) <= p.fine / 0.5 + 1e-9 and len(dv) == 50  # (0.04 V a write: 0.02 mA measured)


def test_a_move_is_written_in_fine_steps_too_and_as_it_was_with_fine_as_large_as_a_step_may_be():
    def steps(rec):
        yield Move(current=I0 - 0.5, offset=V0 + 1.0)

    rig, p = bench()
    r, _ = run(rig, p, steps)
    assert r.state == "done" and len(writes(rig)) == 25 and max(steps_of(I0, writes(rig))) <= 0.02 + 1e-9
    assert len(writes(rig, OFFSET)) == 10 and max(steps_of(V0, writes(rig, OFFSET))) <= 0.1 + 1e-9
    rig, p = bench(p=HopParams(enabled=True, i_nom=I0, smooth=False, fine=STEP_MAX, fine_v=0.0))
    r, _ = run(rig, p, steps)  # (as before: 0.2 mA a write, the piezo at once)
    assert r.state == "done" and len(writes(rig)) == 3 and writes(rig, OFFSET) == [V0 + 1.0]
    rig, p = bench(p=HopParams(enabled=True, i_nom=I0))  # (as it comes: by steps, whatever fine and fine_v say)
    r, _ = run(rig, p, steps)
    assert r.state == "done" and len(writes(rig)) == 3 and writes(rig, OFFSET) == [V0 + 1.0]
    rig, p = bench(p=HopParams(enabled=True, i_nom=I0, smooth=True, fine=5.0))  # (never more than STEP_MAX,
    r, _ = run(rig, p, steps)                                                     # whatever is asked)
    assert r.state == "done" and max(steps_of(I0, writes(rig))) <= STEP_MAX + 1e-9


def test_a_ramped_margins_check_that_finds_a_hop_says_how_far_it_held_at_least():
    """Ramped, its readings kept with where the ramp was: on its mode 0.3 mA short of where it hops off going up, a
    check of 0.5 mA stops at the hop, and says how far up it read the laser on its mode (less how far the ramp may have
    gone on meanwhile): no further than the hop is, and some way towards it."""
    las = laser()
    las.nudge(las.misalignment(0, V0, I0) - (las.limits()[1] - 0.3 * abs(las.ki - las.ksi)))
    rig = Rig(las, las.freq(V0, I0), noise=0.5, dt=0.1)
    p = HopParams(enabled=True, i_nom=I0, smooth=True)

    def steps(rec):
        yield from rec.read()
        rec.slope = las.ki * 1e3  # MHz/mA on its mode
        return (yield from rec.check(0.5))

    r, rec = run(rig, p, steps)
    assert r.state == "done" and r.result is False and rec.notes[-1].startswith("hop within 0.50 mA")
    assert rec.hop_d is not None and 0.0 < rec.hop_d <= 0.3 + 1e-9, rec.hop_d


def test_the_runner_says_when_it_wants_waking_for_a_ramps_next_step():
    rig, p = bench()
    rec = Script(lambda rec: (yield Sweep(current=I0 - 0.2, rate=0.1)), p, I0, V0, I0 - 2, I0 + 2, 60, 80)
    r = Runner(rec, rig, rig.t, clock=rig.now)
    wakes = []
    while r.busy and len(wakes) < 5:
        if r.wake() < math.inf:
            wakes.append(r.wake() - rig.t)
        rig.step(r)
    assert wakes and all(0 <= w <= 0.2 + 1e-9 for w in wakes)  # (a 0.02 mA step every 0.2 s)
    r.cancel("stopped")
    while r.busy:
        rig.step(r)
    assert r.wake() == math.inf and rig.I == pytest.approx(I0)  # (put back where it was)


def test_a_sweep_stopped_in_the_middle_puts_the_current_back():
    rig, p = bench()

    def steps(rec):
        yield Sweep(current=I0 - 1.0, rate=1.0)
        raise modehop.Failed("no good")

    r, _ = run(rig, p, steps)
    assert r.state == "failed" and r.restored and rig.I == pytest.approx(I0)
    assert max(steps_of(I0, writes(rig))) <= p.fine + 1e-9  # (back in fine steps as well)


# ----------------------------------------------------------------- recoveries
def safe(case, rig):
    """The safety rules, for a ramped recovery: the current within its window (up to i_top), no write of it further
    than STEP_MAX from the last (here: than fine, but for what a panic may do)."""
    p = case.p
    lo, top = p.i_nom - p.i_below, p.i_nom + max(p.jump_max, p.i_window)
    w = writes(rig)
    assert all(lo - 1e-6 <= x <= top + 1e-6 for x in w)
    assert max(steps_of(rig.start[0], w), default=0.0) <= STEP_MAX + 1e-9
    return w


@pytest.mark.parametrize("seed", [3, 5, 8, 13])
def test_a_map_ramped_recovers_what_a_map_by_steps_does_in_less_time(seed):
    """The far lasers (a DL pro with the wavemeter alone): mapped by ramps, on target on a mode that holds, as mapped
    a step and a reading at a time, in about two thirds of the time."""
    ramped, stepped = make_case("far", seed, smooth=True), make_case("far", seed)
    a, b = recover(ramped), recover(stepped)
    assert a.on_target and b.on_target, (a.error, b.error)
    assert a.margin >= min(0.3, 0.8 * b.margin) and a.second == 0.0
    assert a.seconds < 0.9 * b.seconds, (a.seconds, b.seconds)
    w = safe(ramped, ramped.rig)
    fine = [d for d in steps_of(I0, w) if d > 1e-9]
    assert statistics.median(fine) <= ramped.p.fine + 1e-9  # (ramped: nearly all its writes are fine ones)
    assert statistics.median([d for d in steps_of(I0, writes(stepped.rig)) if d > 1e-9]) > 0.05
    assert "map" in a.summary and "map" in b.summary


def test_ramped_and_by_steps_do_about_as_well_over_random_lasers():
    """A small Monte Carlo (trials.py runs the large one): every laser that can be recovered is, either way; ramped,
    the median recovery is the shorter by a third at least."""
    seeds = range(20, 32)
    a = [recover(make_case("far", s, smooth=True)) for s in seeds]
    b = [recover(make_case("far", s)) for s in seeds]
    can = [k for k, o in enumerate(a) if o.feasible]
    assert len(can) >= 8 and all(a[k].on_target for k in can) and all(b[k].on_target for k in can)
    ta, tb = (statistics.median(o[k].seconds for k in can) for o in (a, b))
    assert ta < 0.75 * tb, (ta, tb)
    assert statistics.median(a[k].margin for k in can) >= 0.9 * statistics.median(b[k].margin for k in can)


@pytest.mark.parametrize("kind,seed", [("near", 1), ("near", 4), ("shg", 2), ("strong", 5), ("strong", 8)])
def test_the_other_lasers_come_back_ramped(kind, seed):
    case = make_case(kind, seed, smooth=True)
    assert case.feasible
    o = recover(case)
    assert o.on_target, o.error
    safe(case, case.rig)


ALLOWED = {CURRENT, OFFSET} | {"laser1:" + name for name in modehop.WRITABLE}


@pytest.mark.parametrize("kind,n", [("far", 40), ("strong", 40), ("near", 24), ("shg", 24)])
def test_monte_carlo_ramped_every_write_keeps_the_safety_rules(kind, n):
    """As test_modehop's Monte Carlos (which run a step at a time, as it comes), ramped: written in fine steps, on
    the rig's clock. Every laser that can be recovered is (the walk's: all but one in twenty), and
    on every write: the current within its window (the catch's reset: up to where a laser not lasing may go), no
    write of it further than HopParams.fine from the last (none of these lasers stops lasing: no jump), nothing
    written that is not allowed."""
    done = can = 0
    for seed in range(200, 200 + n):
        case = make_case(kind, seed, smooth=True)
        rig, p = case.rig, case.p
        o = recover(case)
        can += case.feasible
        done += case.feasible and o.on_target
        lo, hi = p.i_nom - p.i_below, p.i_nom + p.i_window
        if kind == "near":  # (the catch's reset comes down from 3 mA above: as far as a laser not lasing may go)
            hi = p.i_nom + max(p.jump_max, p.i_window)
        w = writes(rig)
        assert all(lo - 1e-6 <= x <= hi + 1e-6 for x in w), (kind, seed)
        assert max(steps_of(rig.start[0], w), default=0.0) <= p.fine + 1e-9, (kind, seed)
        for name, value in rig.dlc.log:
            if isinstance(value, tuple):  # (a command)
                assert name.split(":", 1)[1] in modehop.COMMANDS, name
            else:
                assert name in ALLOWED or modehop.FALC_OFFSET.match(name), name
        assert not (o.ok and not o.on_target), (kind, seed, o.summary)  # (never said done off its target)
    assert can >= n // 4 and done >= (can - 1 if kind == "shg" and can >= 20 else can), (kind, done, can)


@pytest.mark.parametrize("latency,op", [(0.05, 0.002), (0.1, 0.002), (0.0, 0.02)])
def test_readings_that_come_late_and_a_controller_that_takes_its_time(latency, op):
    """A reading is as old as its exposure and the way here when it comes (half the time between two, or all of
    it), kept with a current the ramp has since left; and a DLC pro takes its time over each write (20 ms): the
    lasers come back all the same (the margins' check finds how late its readings are, and goes by it; the ramps'
    steps are as fine as the controller's pace allows)."""
    outs = []
    for seed in range(3, 17):
        case = make_case("far", seed, smooth=True)
        if case.feasible:
            case.rig.latency, case.rig.op_s = latency, op
            outs.append(recover(case))
    assert len(outs) >= 10 and all(o.on_target for o in outs), [o.error for o in outs if not o.on_target]
    assert statistics.median(o.seconds for o in outs) < 30.0 and statistics.median(o.margin for o in outs) > 0.3


def test_how_late_its_readings_are_is_found_by_ramping_there_and_back():
    """The margins' check, ramped (Recenter._line): readings taken 80 ms before they came, up, down and at rest,
    lie on one line once each is kept with the current as it was then; a hop does not, whatever the delay."""
    rate, slope, late = 1.5, -300.0, 0.08  # mA/s, MHz/mA, s
    knots, t, x = [(0.0, 0.0)], 0.0, 0.0
    for to in (0.5, -0.5, 0.0):  # (there and back, a rest of 0.2 s at each end)
        t += abs(to - x) / rate
        knots.append((t, to))
        t += 0.2
        knots.append((t, to))
        x = to

    def where(at):
        for (t0, x0), (t1, x1) in zip(knots, knots[1:]):
            if t0 <= at <= t1:
                return x0 + (x1 - x0) * (at - t0) / (t1 - t0) if t1 > t0 else x1
        return knots[0][1] if at < knots[0][0] else knots[-1][1]

    times = [0.05 + 0.1 * k for k in range(int(t / 0.1))]
    pts = [(at, slope * where(at - late)) for at in times]
    seen = [(at, where(at)) for at in times]  # (the current each was kept with: where the ramp was when it came)
    kn = [(0.0, 0.0)] + seen
    worst0, b0, _ = Recenter._line(kn, pts, 0.0)
    best = min((Recenter._line(kn, pts, 0.01 * k) for k in range(16)), key=lambda g: g[0])
    assert worst0 > 25.0 and best[0] < 10.0 and best[2] == pytest.approx(late, abs=0.02)  # MHz, MHz, s
    assert best[1] == pytest.approx(slope, rel=0.05)
    hop = [(at, e + (2000.0 if where(at - late) > 0.3 else 0.0)) for at, e in pts]  # (2 GHz, above +0.3 mA)
    assert min(Recenter._line(kn, hop, 0.01 * k)[0] for k in range(16)) > 500.0


def test_a_ramps_steps_are_as_fine_as_its_controller_keeps_up_with():
    def steps(rec):
        yield Sweep(current=I0 + 0.2, rate=1.0, rest=False)  # (its controller's pace heard: 20 ms a write)
        yield Sweep(current=I0 - 0.8, rate=1.0)
        yield Move(current=I0 + 0.2, read=False)

    rig, p = bench(op_s=0.02)
    t0 = rig.t
    r, _ = run(rig, p, steps)
    w = writes(rig)
    d = steps_of(I0, w)
    assert r.state == "done" and max(d) <= STEP_MAX + 1e-9
    assert d[0] == pytest.approx(0.02) and 0.02 < statistics.median(d[10:]) <= 0.1 + 1e-9  # (larger, once it is known)
    assert rig.t - t0 < 0.2 + 1.0 + 0.2 + 0.5  # s (at its pace all the same: not 100 writes of 20 ms)
    rig, p = bench()  # (a controller that answers in 2 ms: as fine as asked)
    r, _ = run(rig, p, steps)
    assert max(steps_of(I0, writes(rig))) <= p.fine + 1e-9


def test_a_ramp_that_hears_nothing_of_the_laser_stops_if_it_must():
    def steps(rec):
        got.append((yield Sweep(current=I0 - 1.0, rate=0.5, silence=0.35)))

    rig, p = bench()
    got, n, hear = [], [0], rig._hear

    def _hear(runner, t, e, q):  # (five readings, then none: time passes, the runner ticked as by the window)
        n[0] += 1
        if n[0] <= 5:
            hear(runner, t, e, q)
        else:
            runner.tick(t)

    rig._hear = _hear
    r, _ = run(rig, p, steps)
    sw = got[0]
    assert r.state == "done" and sw.stopped and sw.silent
    assert I0 - 0.5 - 0.5 * 0.4 - 0.05 <= sw.current <= I0 - 0.2  # (0.35 s after its last reading: not to the end)
    rig, p = bench()
    got, n = [], [0]
    hear2 = rig._hear
    rig._hear = lambda runner, t, e, q: (n.__setitem__(0, n[0] + 1), hear2(runner, t, e, q) if n[0] <= 5
                                         else runner.tick(t))
    r, _ = run(rig, p, lambda rec: (got.append((yield Sweep(current=I0 - 1.0, rate=0.5, rest=False))) or iter(())))
    assert got[0].current == pytest.approx(I0 - 1.0) and not got[0].stopped  # (not asked to: it goes on, unseen)


def test_what_a_ramp_reads_on_the_way_is_read_after_each_step_has_had_its_time_and_where_it_ends():
    rig, p = bench()
    got = []

    def steps(rec):
        got.append((yield Sweep(current=I0 + 0.2, rate=0.2, names=("dl:cc:current-set",), every=0.0, rest=False)))

    r, _ = run(rig, p, steps)
    vals = [s for s in got[0].samples if s.values is not None]
    assert r.state == "done" and len(vals) == 11  # (before each of its 10 steps, and once more where it ended)
    assert [round(s.values[0] - I0, 3) for s in vals] == [round(0.02 * k, 3) for k in range(11)]
    gaps = [b.t - a.t for a, b in zip(vals, vals[1:])]
    assert min(gaps) > 0.08 and max(gaps) < 0.12  # s (a step's time, 0.1 s, after the write before it)




# -------------------------------------------------------- a hop seen at once
def test_a_jump_far_off_in_one_place_is_a_hop_without_the_grace():
    g = Guard()
    t = 0.0
    for _ in range(Guard.SETTLED):
        t += 0.1
        assert not g.update(t, 1.0, 100.0, 2.0, jump=1200.0)
    assert g.armed
    # 3 GHz off, three readings in one place: a hop, 0.3 s after it (the grace would have waited 2 s)
    out = []
    for e in (3001.0, 2998.0, 3003.0):
        t += 0.1
        out.append(g.update(t, e, 100.0, 2.0, jump=1200.0))
    assert out == [False, False, True]


def test_readings_all_over_the_place_or_not_far_off_wait_for_the_grace():
    def armed():
        g = Guard()
        for k in range(Guard.SETTLED):
            g.update(0.1 * (k + 1), 0.0, 100.0, 2.0, jump=1200.0)
        return g, 0.1 * Guard.SETTLED

    g, t = armed()  # (a lock's ringing, a wavemeter's nonsense: far off, but nowhere twice)
    said = []
    for e in (3000.0, -2500.0, 4100.0, 1900.0, 3500.0, 2600.0):
        t += 0.1
        said.append(g.update(t, e, 100.0, 2.0, jump=1200.0))
    assert not any(said)
    g, t = armed()  # (off by less than a hop would take it: drifting, or a lock on the wrong peak; the grace)
    said = []
    for k in range(25):
        t += 0.1
        said.append(g.update(t, 400.0, 100.0, 2.0, jump=1200.0))
    assert said.index(True) >= 19
    g, t = armed()  # (without a jump given: the grace, as ever)
    said = []
    for k in range(25):
        t += 0.1
        said.append(g.update(t, 3000.0, 100.0, 2.0))
    assert said.index(True) >= 19
    g, t = armed()  # (back on target in between: begun again)
    for e in (3000.0, 3000.0, 2.0, 3000.0, 3000.0):
        t += 0.1
        assert not g.update(t, e, 100.0, 2.0, jump=1200.0)


# ------------------------------------------------------------ through the engine
def quiet_demo():
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    for las in demo.lasers:
        las.drift = 0.0
    return demo


def test_a_hand_at_its_current_is_not_taken_for_a_hop_and_a_hop_is_seen_in_a_fraction_of_the_grace():
    """A laser whose current tunes it 1.5 GHz/mA, locked to the wavemeter, read every 30 ms: its current set 0.25 mA
    up or down by hand jumps it 375 MHz, three readings in one place before its controller's poll has seen the
    hand. Its set current is read first: changed, no hop (its lock pulls it back). A hop is seen and its recovery
    begun well within the 2 s it used to wait."""
    import time

    from test_engine import hop_engine, wait_for

    demo = quiet_demo()
    try:
        dlc, las = demo.dlcs[0], demo.laser(1)
        with las._mutex:
            d = las.ecdl.ki - las.ecdl.ksi
            las.ecdl.ki, las.ecdl.ksi = -1.5, -1.5 - d
        e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True))
        try:
            e.engage(1)
            assert wait_for(lambda: ch.state == "on" and ch.guard.armed and ch.hop.i_nom == 150.0, 5)
            assert e._hop_jump(ch) >= 300.0
            for k in range(4):
                assert wait_for(lambda: ch.state == "on" and ch.guard.armed and abs(ch.error_mhz) < 5, 30), ch.message
                time.sleep(0.37 * (k + 1) % 0.5)  # (anywhere in its controller's half-second poll)
                i = dlc.current(1)
                dlc.execute("(param-set! 'laser1:dl:cc:current-set %.3f)" % (i + (0.25 if k % 2 == 0 else -0.25)))
                time.sleep(2.5)
                assert len(ch.guard.recent) == 0 and ch.state == "on" and las.ecdl.m == 0, (k, ch.hop_why)
            assert wait_for(lambda: ch.guard.armed and abs(ch.error_mhz) < 5, 30)
            t0 = time.monotonic()
            las.hop_mode(+1)
            assert wait_for(lambda: ch.state == "recovering", 3)
            assert time.monotonic() - t0 < 1.0 and ch.hop_why.startswith("jumped +")  # s (its grace: 2 s)
            assert wait_for(lambda: ch.state == "on", 60), ch.message
            assert wait_for(lambda: abs(ch.error_mhz) < 5, 10) and len(ch.guard.recent) == 1
        finally:
            e.stop()
    finally:
        demo.close()


def test_a_laser_that_stops_lasing_is_jumped_in_as_few_writes_as_ever_whatever_its_fine_steps():
    """Its TA switches itself off 5 s after its seed has gone: the jump that makes it lase again is no place for
    fine steps. With a controller that takes 20 ms over each command, it lases again within 3 s, in writes of
    STEP_MAX (13 of them, not 121), and comes back down steadily in fine ones."""
    import time

    from test_engine import hop_engine, wait_for

    demo = quiet_demo()
    try:
        dlc, shg = demo.dlcs[2], demo.laser(demo.SHG)
        execute = dlc.execute

        def slow(cmd):
            time.sleep(0.02)
            return execute(cmd)

        e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, smooth=True))
        try:
            e.engage(demo.SHG)
            assert wait_for(lambda: ch.guard.armed and ch.seed_name == "amp:pd:seed:power" and ch.hop.seed_ok, 8)
            dlc.execute = slow
            time.sleep(1.0)
            n0 = len(dlc.written("laser1:dl:cc:current-set"))
            t0 = time.monotonic()
            shg.stop_lasing()
            assert wait_for(lambda: shg.ecdl.lasing or not shg.lock.ta.on(), 30)
            took = time.monotonic() - t0
            up = dlc.written("laser1:dl:cc:current-set")[n0:]
            assert shg.ecdl.lasing and shg.lock.ta.on() and took < 3.5, took
            assert len(up) <= 20 and max(abs(b - a) for a, b in zip([150.0] + up, up)) <= STEP_MAX + 1e-9
            assert max(abs(b - a) for a, b in zip([150.0] + up, up)) > 0.1  # (in writes as large as they may be)
            dlc.execute = execute
            assert wait_for(lambda: ch.state == "on", 120), ch.message
            assert shg.lock.ta.on() and shg.ecdl.lasing
            down = dlc.written("laser1:dl:cc:current-set")[n0 + len(up):]
            small = [abs(b - a) for a, b in zip(down, down[1:]) if abs(b - a) > 1e-9]
            assert statistics.median(small) <= 0.05  # (the rest of it: fine)
        finally:
            dlc.execute = execute
            e.stop()
    finally:
        demo.close()
