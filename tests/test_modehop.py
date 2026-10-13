"""Mode-hop recovery against a simulated external-cavity diode laser, driven synchronously.

Every run checks the safety rules: the current never leaves its window and never changes by more
than STEP_MAX in one write."""
import math
import random

import pytest

from wmlock.modehop import STEP_MAX, Failed, Guard, HopParams, Move, Point, Read, Recenter, Runner, analyse
from wmlock.sim import ECDL, SimCavityLock

I0, V0 = 150.0, 70.0


class Bench:
    """The controller and the wavemeter: writes land at once, each reading takes dt seconds."""

    def __init__(self, laser, target, lock=None, noise=0.5, dt=0.1, seed=0, current=I0, offset=V0):
        self.laser, self.lock, self.target = laser, lock, target
        self.I, self.V = current, offset
        self.noise, self.dt, self.t = noise, dt, 0.0
        self.rng = random.Random(seed)
        self.writes, self.pending = [], []
        self.refuse = None  # (kind, exception): fail the next write of that kind, once

    def freq(self):
        if self.lock is not None:
            return self.lock.frequency(self.laser, self.V, self.I)
        return self.laser.freq(self.V, self.I)

    def reading(self):
        self.t += self.dt
        return self.t, (self.freq() - self.target) * 1e6 + self.rng.gauss(0.0, self.noise)

    def write(self, kind, value, done, failed):
        if self.refuse and self.refuse[0] == kind:
            exc, self.refuse = self.refuse[1], None
            self.pending.append(lambda: failed(exc))
            return
        self.writes.append((kind, value))
        if kind == "current":
            self.I = value
        elif kind == "offset":
            self.V = value
        else:
            self.lock.enabled = value
        self.pending.append(done)

    def run(self, rec, timeout=900.0, until=None):
        runner = Runner(rec, self, self.t, timeout)
        while runner.busy and (until is None or not until(runner)):
            if self.pending:
                self.pending.pop(0)()
            else:
                runner.on_reading(*self.reading())
        return runner

    def margin(self):
        """True distance (mA) from the operating point to the edge of the lasing mode's stable range."""
        las = self.laser
        v = self.V + (self.lock.dv if self.lock is not None else 0.0)
        return (las.fsr / 2 - las.hysteresis - abs(las.misalignment(las.m, v, self.I))) / abs(las.ki - las.ksi)

    def off_mhz(self):
        return (self.freq() - self.target) * 1e6


def recover(laser, target, p=None, lock=None, noise=0.5, kv=0.0, cavity=False, quick=True, seed=0):
    p = p or HopParams(enabled=True, i_nom=I0, cavity=cavity)
    b = Bench(laser, target, lock, noise, seed=seed)
    i_lo, i_hi = p.i_nom - p.i_window, p.i_nom + p.i_window
    rec = Recenter(p, I0, V0, i_lo, i_hi, V0 - p.v_window, V0 + p.v_window, kv=kv, cavity=cavity, quick=quick)
    runner = b.run(rec)
    currents = [I0] + [v for k, v in b.writes if k == "current"]
    assert all(i_lo - 1e-9 <= c <= i_hi + 1e-9 for c in currents), "current outside its window"
    assert all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip(currents, currents[1:])), "current jumped"
    return b, rec, runner


def hopped_laser(direction=1, seed=1):
    """A laser that was centered on its target mode, and has just hopped to the next one."""
    laser = ECDL(400.0, seed=seed)
    target = laser.freq(V0, I0)
    laser.nudge(direction * 3.9)
    laser.freq(V0, I0)
    assert laser.m == direction
    return laser, target


# -------------------------------------------------------------------- the map
def test_analyse_finds_stable_ranges_from_both_sweep_directions():
    # mode A below 151 mA, mode B above; with hysteresis: up switches at 151.3, down at 150.7
    pts, leg = [], 0
    for leg, (a, b, up) in enumerate([(150.0, 149.0, False), (149.0, 152.0, True), (152.0, 150.0, False)]):
        n = round(abs(b - a) / 0.1)
        for k in range(1, n + 1):
            i = a + (b - a) * k / n
            in_b = i > 151.3 if up else i > 150.7
            pts.append(Point(i, (6000.0 if in_b else 0.0) - 250.0 * (i - 150.0), leg, up))
    res = analyse(pts, 0.1)
    assert len(res.modes) == 2 and res.spacing == pytest.approx(6000, abs=10)
    assert res.slope == pytest.approx(-250, abs=1) and res.noise < 1
    a, b = res.modes
    assert a.stable == [(pytest.approx(149.1), pytest.approx(150.7))]  # below where down-sweeps leave B
    assert b.stable == [(pytest.approx(151.4), pytest.approx(151.9))]
    assert a.at(150.0) == pytest.approx(0.0, abs=1)


# ------------------------------------------------------------- recoveries
@pytest.mark.parametrize("direction", [1, -1])
def test_recovers_after_a_mode_hop(direction):
    laser, target = hopped_laser(direction)
    b, rec, r = recover(laser, target)
    assert r.state == "done", r.error
    assert abs(b.off_mhz()) < 5 and b.margin() > 0.4
    assert laser.m == 0 and rec.maps == 1  # back on its own mode: the current moved it there
    assert abs(r.result.current - I0) > 1.0  # ...which needed the current, not just the piezo
    assert r.result.kv == pytest.approx(400, rel=0.05) and r.result.spacing == pytest.approx(6000, rel=0.05)


def test_quick_when_a_cavity_lock_caught_the_wrong_peak():
    laser = ECDL(400.0, seed=3)
    lock = SimCavityLock(laser.freq(V0, I0))
    lock.enabled = True
    lock.frequency(laser, V0, I0)
    lock.jump(1)  # locked, but one free spectral range away
    b, rec, r = recover(laser, lock.f_res, lock=lock, cavity=True)
    assert r.state == "done" and rec.maps == 0 and rec.readings < 20
    assert [v for k, v in b.writes if k == "lock"] == [False, True]
    assert lock.state == "locked" and abs(b.off_mhz()) < 1e-6
    assert b.I == I0  # the laser itself was fine: the current was only stepped to check that


def test_cavity_locked_laser_after_a_hop():
    laser = ECDL(400.0, seed=4)
    lock = SimCavityLock(laser.freq(V0, I0))
    lock.enabled = True
    lock.frequency(laser, V0, I0)
    laser.nudge(-3.9)  # hops down a mode; the lock relocks 4 peaks away and calls that 'locked'
    assert abs(lock.frequency(laser, V0, I0) - lock.f_res) * 1e6 == pytest.approx(6000, abs=1)
    b, rec, r = recover(laser, lock.f_res, lock=lock, cavity=True)
    assert r.state == "done", r.error
    assert lock.state == "locked" and abs(b.off_mhz()) < 1e-6 and b.margin() > 0.4


def test_cavity_lock_that_keeps_catching_the_wrong_peak_is_left_off():
    laser = ECDL(400.0, seed=5)
    lock = SimCavityLock(laser.freq(V0, I0))
    lock.enabled = True
    lock.frequency(laser, V0, I0)
    lock.jump(1)
    lock.nearest = lambda f: lock.f_res + 1.5e-3  # this lock always goes for the next peak up
    b, rec, r = recover(laser, lock.f_res, lock=lock, cavity=True)
    assert r.state == "failed" and "wrong peak" in r.error
    assert not lock.enabled and not r.restored  # left where it was found best, lock off


def test_manual_recenter_maps_even_on_target():
    laser = ECDL(400.0, seed=6)
    laser.nudge(-2.0)  # still on its mode, but near an edge: 0.23 mA from where it stops being alone
    target = laser.freq(V0, I0)
    b0 = Bench(laser, target)
    assert b0.margin() < 0.3
    b, rec, r = recover(laser, target, quick=False)
    assert r.state == "done" and rec.maps >= 1
    assert abs(b.off_mhz()) < 5 and b.margin() > 0.8


def test_nothing_within_reach_fails_and_puts_everything_back():
    laser = ECDL(400.0, seed=7)
    target = laser.freq(V0, I0) + 60e-3  # 60 GHz away: another diode mode, beyond current and piezo
    b, rec, r = recover(laser, target)
    assert r.state == "failed" and r.restored and "reach" in r.error
    assert (b.I, b.V) == (I0, V0)


@pytest.mark.parametrize("kv", [400.0, -400.0, 2000.0])  # right, wrong sign, wrong size
def test_a_stale_piezo_tuning_does_not_matter(kv):
    laser, target = hopped_laser(1, seed=8)
    b, rec, r = recover(laser, target, kv=kv)
    assert r.state == "done", r.error
    assert abs(b.off_mhz()) < 5 and b.margin() > 0.3 and r.result.kv == pytest.approx(400, rel=0.1)


def test_noisy_wavemeter():
    laser, target = hopped_laser(-1, seed=9)
    b, rec, r = recover(laser, target, noise=5.0, seed=9)
    assert r.state == "done", r.error
    assert abs(b.off_mhz()) < 15 and b.margin() > 0.3


def random_case(seed):
    """A random laser, slightly off center on its target mode, that has hopped 1 or 2 modes."""
    rng = random.Random(seed)
    fsr = rng.uniform(3, 10)
    kv = rng.choice([-1, 1]) * rng.uniform(0.2, 0.8)
    laser = ECDL(400.0, fsr=fsr, kv=kv, ki=rng.uniform(-0.5, -0.1), ksv=kv * rng.uniform(0.6, 0.95),
                 ksi=rng.uniform(-4, -1), hysteresis=rng.uniform(0, 0.15) * fsr,
                 multimode=rng.uniform(0.02, 0.1) * fsr, seed=seed)
    laser.nudge(rng.uniform(-0.25, 0.25) * fsr)
    laser.m = 0
    target = laser.freq(V0, I0)
    hops = rng.choice([1, 1, 1, 2])
    a = laser.misalignment(0, V0, I0)
    laser.nudge(a + rng.choice([-1, 1]) * (fsr * (hops - 0.5) + laser.hysteresis + rng.uniform(0.02, 0.4) * fsr))
    laser.freq(V0, I0)
    return laser, target, rng.choice([0.3, 1.0, 3.0])


def best_margin(laser, p):
    """The largest margin any operating point on target within the windows has (mA)."""
    best = -math.inf
    for m in range(-4, 5):
        for k in range(-100, 101):
            di = p.i_window * k / 100
            dv = -(m * laser.fsr + laser.ki * di) / laser.kv
            if abs(dv) <= p.v_window:
                a = m * laser.fsr + (laser.kv - laser.ksv) * dv + (laser.ki - laser.ksi) * di - laser.shift
                best = max(best, (laser.fsr / 2 - laser.hysteresis - abs(a)) / abs(laser.ki - laser.ksi))
    return best


def test_monte_carlo_over_random_lasers():
    p = HopParams(enabled=True, i_nom=I0)
    feasible = succeeded = 0
    for seed in range(200):
        laser, target, noise = random_case(seed)
        best = best_margin(laser, p)
        b, rec, r = recover(laser, target, p, noise=noise, seed=seed)
        if r.state == "done":
            assert abs(b.off_mhz()) < 5 + 3 * noise, seed
            assert b.margin() > -0.1, seed  # never left on the edge of a mode
        else:
            assert r.restored and (b.I, b.V) == (I0, V0), seed
        if best >= 0.3:
            feasible += 1
            succeeded += r.state == "done"
    assert feasible > 120 and succeeded >= 0.97 * feasible


# ------------------------------------------------------------------ runner
def run_until_mapping(laser, target, bench_kwargs=None):
    p = HopParams(enabled=True, i_nom=I0)
    b = Bench(laser, target, **(bench_kwargs or {}))
    rec = Recenter(p, I0, V0, 148.0, 152.0, 60.0, 80.0)
    r = b.run(rec, until=lambda r: rec.stage.startswith("mapping") and rec.readings > 30)
    return b, rec, r


def test_cancel_puts_current_and_offset_back_gently():
    laser, target = hopped_laser(1)
    b, rec, r = run_until_mapping(laser, target)
    assert r.busy and b.I != I0
    r.cancel("stopped by the user")
    while r.busy:
        b.pending.pop(0)()
    assert r.state == "failed" and r.restored and r.error == "stopped by the user"
    assert (b.I, b.V) == (I0, V0)
    currents = [v for k, v in b.writes if k == "current"]
    assert all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip(currents, currents[1:]))


def test_refused_write_puts_things_back_but_a_lost_connection_cannot():
    laser, target = hopped_laser(1)
    b, rec, r = run_until_mapping(laser, target)
    b.refuse = ("current", ValueError("Error: -21 above current-clip"))
    while r.busy:
        if b.pending:
            b.pending.pop(0)()
        else:
            r.on_reading(*b.reading())
    assert r.state == "failed" and r.restored and "current-clip" in r.error and (b.I, b.V) == (I0, V0)

    laser, target = hopped_laser(1)
    b, rec, r = run_until_mapping(laser, target)
    b.refuse = ("current", ConnectionResetError("reset by peer"))
    while r.busy:
        if b.pending:
            b.pending.pop(0)()
        else:
            r.on_reading(*b.reading())
    assert r.state == "failed" and not r.restored and "reset" in r.error


def test_timeouts():
    laser, target = hopped_laser(1)
    p = HopParams(enabled=True, i_nom=I0)
    b = Bench(laser, target)
    r = Runner(Recenter(p, I0, V0, 148.0, 152.0, 60.0, 80.0), b, 0.0)
    r.tick(Runner.READ_TIMEOUT + 1)  # no reading ever came
    assert r.state == "failed" and "no valid readings" in r.error and r.restored  # nothing had moved

    b = Bench(laser, target)
    r = Runner(Recenter(p, I0, V0, 148.0, 152.0, 60.0, 80.0), b, 0.0)
    r.on_reading(*b.reading())  # the first reading: it starts moving
    assert r._waiting == "ack"
    r.tick(b.t + Runner.ACK_TIMEOUT + 1)  # ...but the controller never answers
    assert r.state == "failed" and "no answer" in r.error


def test_the_runner_never_writes_outside_the_window():
    class Rogue(Recenter):
        def run(self):
            yield Read()
            yield Move(current=500.0)  # a bug asking for far too much
            yield Read()

    laser = ECDL(400.0)
    b = Bench(laser, laser.freq(V0, I0))
    r = b.run(Rogue(HopParams(), I0, V0, 148.0, 152.0, 60.0, 80.0))
    currents = [v for k, v in b.writes if k == "current"]
    assert max(currents) == pytest.approx(152.0) and r.state == "done"
    assert all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip([I0] + currents, currents))


def test_procedure_errors_restore():
    class Broken(Recenter):
        def run(self):
            yield Move(current=151.0)
            raise Failed("gave up")

    laser = ECDL(400.0)
    b = Bench(laser, laser.freq(V0, I0))
    r = b.run(Broken(HopParams(), I0, V0, 148.0, 152.0, 60.0, 80.0))
    assert r.state == "failed" and r.error == "gave up" and r.restored and b.I == I0


# ------------------------------------------------------------------- guard
def test_guard_needs_the_laser_on_target_first_then_a_lasting_jump():
    g = Guard()
    assert not any(g.update(t, 5000.0, 100.0, 2.0) for t in range(10))  # never seen on target
    for t in range(10, 15):
        assert not g.update(t * 0.1, 3.0, 100.0, 2.0)
    assert g.armed
    assert not g.update(1.6, 6000.0, 100.0, 2.0) and not g.update(1.7, math.nan, 100.0, 2.0)
    assert not g.update(1.8, 6000.0, 100.0, 2.0)  # not for long enough yet
    assert not g.update(2.0, 50.0, 100.0, 2.0)   # back: a glitch
    assert not g.update(2.1, 6000.0, 100.0, 2.0) and not g.update(3.0, 6000.0, 100.0, 2.0)
    assert g.update(4.2, 6000.0, 100.0, 2.0)


def test_guard_tracks_the_wavemeter_and_limits_the_rate():
    g = Guard()
    for k in range(3000):  # the wavemeter drifts 30 MHz over 5 minutes
        g.update(k * 0.1, 30.0 * k / 3000, 100.0, 2.0, track=True)
    assert 3 < g.track < 30
    assert all(g.allow(t, 4) for t in (0, 10, 20, 30)) and not g.allow(40, 4)
    assert g.allow(3700, 4)


def test_params_round_trip():
    p = HopParams(enabled=True, cavity=True, i_nom=151.25, max_per_hour=2, kv=-410.0)
    q = HopParams.from_dict(p.to_dict())
    assert q == p and isinstance(q.max_per_hour, int) and q.cavity is True
    assert HopParams.from_dict(None) == HopParams()


def test_cancel_with_a_write_on_its_way_waits_for_it():
    laser, target = hopped_laser(1)
    b, rec, r = run_until_mapping(laser, target)
    while r._waiting != "ack":
        if b.pending:
            b.pending.pop(0)()
        else:
            r.on_reading(*b.reading())
    in_flight = b.I  # written by the bench already; its acknowledgement has not come back
    r.cancel("stopped")
    assert r.state == "stopping" and r.busy
    while r.busy:
        b.pending.pop(0)()
    currents = [v for k, v in b.writes if k == "current"]
    assert in_flight in currents and (b.I, b.V) == (I0, V0) and r.restored
    assert all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip(currents, currents[1:]))
