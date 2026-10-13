"""Mode-hop recovery against a simulated external-cavity diode laser and DLC pro, driven
synchronously: the procedure talks to a FakeDLCPro through the real client, in the same thread.

Every run checks the safety rules: the current never leaves its window and never changes by more
than STEP_MAX in one write, and nothing is written that is not allowed."""
import math
import random

import pytest

from wmlock.dlcpro import DecopError, DLCPro
from wmlock.modehop import (COMMANDS, FALC_OFFSET, STEP_MAX, WRITABLE, Failed, Guard, HopParams, Move, Point,
                            Read, Recenter, Runner, Set, analyse, perform)
from wmlock.sim import ECDL, FakeDLCPro, SimLocked

I0, V0 = 150.0, 70.0


class Loopback(DLCPro):
    """A DLCPro client for a FakeDLCPro in the same thread: no sockets."""

    def __init__(self, fake):
        super().__init__("loopback")
        self.fake = fake

    def command(self, cmd: str) -> str:
        return self.fake.execute(cmd).strip()


class Bench:
    """The laser on its controller, and the wavemeter: operations land at once (their answers come
    back one at a time), each reading takes dt seconds."""

    def __init__(self, laser, target, noise=0.5, dt=0.1, seed=0, current=I0, offset=V0, scan=False):
        self.laser, self.target = laser, target
        self.dlc = FakeDLCPro("SN-B", "bench", {1: {"current": current, "offset": offset, "scan": scan}},
                              serve=False)
        self.client = Loopback(self.dlc)
        self.lock = None
        self.noise, self.dt, self.t = noise, dt, 0.0
        self.rng = random.Random(seed)
        self.pending = []
        self.refuse = None  # (part of a name, exception): fail the next operation on it, once

    def lock_it(self, **kwargs) -> SimLocked:
        self.lock = SimLocked(self.dlc, 1, self.laser, self.target, **kwargs)
        return self.lock

    @property
    def I(self):  # noqa: E743 (as in the formulas)
        return self.dlc.params["laser1:dl:cc:current-set"]

    @property
    def V(self):
        return self.dlc.params["laser1:scan:offset"]

    def freq(self):
        if self.lock is not None:
            return self.lock.frequency()
        return self.laser.freq(self.V, self.I)

    def reading(self):
        self.t += self.dt
        return self.t, (self.freq() - self.target) * 1e6 + self.rng.gauss(0.0, self.noise)

    def do(self, op, done, failed):
        if self.refuse and op[0] != "get" and self.refuse[0] in op[1]:
            exc, self.refuse = self.refuse[1], None
            self.pending.append(lambda: failed(exc))
            return
        try:
            result = perform(self.client, 1, op)
        except DecopError as exc:
            refused = exc  # (exc itself is gone after the except block)
            self.pending.append(lambda: failed(refused))
            return
        self.pending.append(lambda: done(result))

    def writes(self, param):
        """Values written to laser1:param (or to an absolute parameter), oldest first."""
        return self.dlc.written(param if param.startswith(("falc", "io:")) else "laser1:" + param)

    def run(self, rec, timeout=900.0, until=None, confirm=True):
        runner = Runner(rec, self, self.t, timeout)
        while runner.busy and (until is None or not until(runner)):
            if self.pending:
                self.pending.pop(0)()
            elif confirm and runner.asking:
                runner.confirm()
            else:
                runner.on_reading(*self.reading())
        return runner

    def margin(self):
        """True distance (mA) from the operating point to the edge of the lasing mode's stable range."""
        las = self.laser
        v = self.V + (self.lock.dv if self.lock is not None else 0.0)
        return (las.fsr / 2 - las.hysteresis - abs(las.misalignment(las.m, v, self.I))) / abs(las.ki - las.ksi)

    def second_mode(self):
        """The power fraction in a second mode at the operating point."""
        las = self.laser
        v = self.V + (self.lock.dv if self.lock is not None else 0.0)
        las.lase(v, self.I)
        comps = las.components(v, self.I)
        return comps[1][0] if len(comps) > 1 else 0.0

    def single_mode(self):
        return self.second_mode() == 0.0

    def off_mhz(self):
        return (self.freq() - self.target) * 1e6


ALLOWED = {"laser1:dl:cc:current-set", "laser1:scan:offset"} | {"laser1:" + k for k in WRITABLE}


def check_safety(b, i_lo, i_hi, p):
    currents = [I0] + b.writes("dl:cc:current-set")
    assert all(i_lo - 1e-9 <= c <= i_hi + 1e-9 for c in currents), "current outside its window"
    assert all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip(currents, currents[1:])), "current jumped"
    falc0 = {}
    for name, value in b.dlc.log:
        if isinstance(value, tuple):  # a command
            assert name.split(":", 1)[1] in COMMANDS, name
        elif FALC_OFFSET.match(name):
            falc0.setdefault(name, value)
        else:
            assert name in ALLOWED, f"wrote {name}"


def recover(laser, target, p=None, noise=0.5, kv=0.0, cavity=False, quick=True, seed=0, bench=None):
    p = p or HopParams(enabled=True, i_nom=I0, cavity=cavity)
    b = bench or Bench(laser, target, noise, seed=seed)
    b.noise = noise
    i_lo, i_hi = p.i_nom - p.i_window, p.i_nom + p.i_window
    rec = Recenter(p, b.I, b.V, i_lo, i_hi, b.V - p.v_window, b.V + p.v_window, kv=kv, cavity=cavity, quick=quick)
    runner = b.run(rec)
    check_safety(b, i_lo, i_hi, p)
    return b, rec, runner


def hopped_laser(direction=1, seed=1):
    """A laser that was centered on its target mode, and has just hopped to the next one."""
    laser = ECDL(400.0, seed=seed)
    target = laser.freq(V0, I0)
    laser.nudge(direction * 3.9)
    laser.freq(V0, I0)
    assert laser.m == direction
    return laser, target


def locked_bench(laser, seed=0, **kwargs):
    """A laser its DLC pro has locked to the cavity resonance at its target (as by hand)."""
    target = laser.freq(V0, I0)
    b = Bench(laser, target, seed=seed, scan=kwargs.pop("scan", True))
    lock = b.lock_it(seed=seed, **kwargs)
    lock.settle()
    assert lock.locked() and abs(b.off_mhz()) < 1
    return b, lock


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
    b, lock = locked_bench(ECDL(400.0, seed=3), shg=False, lockpoints=False)
    lock.jump(1)  # locked, but one free spectral range away
    b, rec, r = recover(b.laser, b.target, cavity=True, bench=b)
    assert r.state == "done" and rec.maps == 0 and rec.readings < 30
    assert rec.relock_mode == "plain" and b.writes("dl:lock:lock-enabled") == [False, True]
    assert lock.locked() and abs(b.off_mhz()) < 1e-6
    assert b.I == I0  # the laser itself was fine: the current was only stepped to check that
    assert b.writes("scan:enabled") == [False, True]  # stopped while unlocked, as it ran then


def test_cavity_locked_laser_after_a_hop():
    b, lock = locked_bench(ECDL(400.0, seed=4), shg=False, lockpoints=False)
    b.laser.nudge(-3.9)  # hops down a mode; the lock relocks 4 peaks away and calls that 'locked'
    assert abs(b.off_mhz()) == pytest.approx(6000, abs=1) and lock.locked()
    b, rec, r = recover(b.laser, b.target, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert lock.locked() and abs(b.off_mhz()) < 1e-6 and b.margin() > 0.4


def test_cavity_lock_that_keeps_catching_the_wrong_peak_is_left_off():
    b, lock = locked_bench(ECDL(400.0, seed=5), shg=False, lockpoints=False)
    lock.jump(1)
    lock._nearest = lambda f: (lock.f_res + 1.5e-3, 1.0)  # this lock always goes for the next peak up
    b, rec, r = recover(b.laser, b.target, cavity=True, bench=b)
    assert r.state == "failed"
    assert r.error == "relocking failed 3 times, last: the cavity lock caught a peak +1500 MHz away"
    assert b.dlc.params["laser1:dl:lock:lock-enabled"] is False and not r.restored  # left where it was found best


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
    laser, hop, noise = random_laser(seed)
    target = laser.freq(V0, I0)
    laser.nudge(hop)
    laser.freq(V0, I0)
    return laser, target, noise


def random_laser(seed):
    """A random laser, slightly off center on its mode; the nudge that makes it hop 1 or 2 modes."""
    rng = random.Random(seed)
    fsr = rng.uniform(3, 10)
    kv = rng.choice([-1, 1]) * rng.uniform(0.2, 0.8)
    laser = ECDL(400.0, fsr=fsr, kv=kv, ki=rng.uniform(-0.5, -0.1), ksv=kv * rng.uniform(0.6, 0.95),
                 ksi=rng.uniform(-4, -1), hysteresis=rng.uniform(0, 0.15) * fsr,
                 multimode=rng.uniform(0.02, 0.1) * fsr, seed=seed)
    laser.nudge(rng.uniform(-0.25, 0.25) * fsr)
    laser.m = 0
    hops = rng.choice([1, 1, 1, 2])
    a = laser.misalignment(0, V0, I0)
    hop = a + rng.choice([-1, 1]) * (fsr * (hops - 0.5) + laser.hysteresis + rng.uniform(0.02, 0.4) * fsr)
    return laser, hop, rng.choice([0.3, 1.0, 3.0])


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
    currents = b.writes("dl:cc:current-set")
    assert all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip(currents, currents[1:]))


def test_refused_write_puts_things_back_but_a_lost_connection_cannot():
    laser, target = hopped_laser(1)
    b, rec, r = run_until_mapping(laser, target)
    b.refuse = ("current", DecopError("Error: -21 above current-clip"))
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
    b.pending.clear()  # ...but the controller never answers
    assert r._waiting == "ack"
    r.tick(b.t + Runner.ACK_TIMEOUT + 1)
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
    currents = b.writes("dl:cc:current-set")
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
    p = HopParams(enabled=True, cavity=True, shg=True, relock="lockpoint", i_nom=151.25, max_per_hour=2,
                  kv=-410.0, lp_kind="t", lp_code=116, shg_n=2)
    q = HopParams.from_dict(p.to_dict())
    assert q == p and isinstance(q.max_per_hour, int) and q.cavity is True and q.lp_kind == "t"
    assert HopParams.from_dict(None) == HopParams()
    assert HopParams.from_dict({"lp_code": "nonsense", "step": "0.2"}).step == 0.2  # bad values are skipped
    f = p.forget()
    assert f.kv == 0 and f.lp_code == -1 and f.lp_kind == "" and f.shg and f.i_nom == 151.25


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
    currents = b.writes("dl:cc:current-set")
    assert in_flight in currents and (b.I, b.V) == (I0, V0) and r.restored
    assert all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip(currents, currents[1:]))


# ------------------------------------------------ SHG lasers, relocked by lockpoint
SHG = HopParams(enabled=True, i_nom=I0, cavity=True, shg=True)
ORIGINAL = {"laser1:scope:channel1:signal": 0, "laser1:scope:channel2:signal": 2,
            "laser1:nlo:shg:scope:channel1:signal": 82, "laser1:nlo:shg:scope:channel2:signal": 90}


def shg_bench(seed=11, **kwargs):
    """A TA-SHG pro: its master (going multimode well before it hops, quietly for the wavemeter)
    locked by its DLC pro to the cavity peak at its target, lockpoint chosen by hand, PDH through a
    FALC pro whose input offset does not null the error signal's 9 mV; its SHG cavity locked."""
    laser = ECDL(400.0, seed=seed, multimode=0.9, hysteresis=0.4, mm_noise=0.004)
    kwargs.setdefault("pdh_dc", 0.009)
    return locked_bench(laser, seed=seed, shg=True, lockpoints=True, **kwargs)


def settings(b):
    return {k: b.dlc.params[k] for k in ORIGINAL}


def test_shg_laser_relocked_the_way_you_would_by_hand():
    b, lock = shg_bench()
    b.laser.nudge(+3.9)  # it hops; its lock relocks 4 peaks away and calls that locked
    assert b.off_mhz() == pytest.approx(6000, abs=1) and lock.locked()
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    # on the right TEM00 peak, single-mode with margins, SHG cavity locked, error-signal DC nulled
    assert lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12)
    assert b.single_mode() and b.margin() > 0.3 and lock.shg_state() == (5, "locked")
    assert abs(lock.dc()) < 0.002 and b.writes("falc1:input:offset")[-1] == pytest.approx(-0.009, abs=0.001)
    # in your order: both locks off, map scanning the SHG cavity, lock it, scan, null, pick, lock
    order = [n.replace("laser1:", "") for n, _ in b.dlc.log if "cc:current" not in n and "scan:offset" not in n]
    assert order == ["dl:lock:lock-enabled", "nlo:shg:lock:lock-enabled", "scan:enabled", "nlo:shg:scan:enabled",
                     "nlo:shg:scan:enabled", "nlo:shg:lock:lock-enabled", "scan:enabled", "falc1:input:offset",
                     "dl:lock:select-lockpoint", "dl:lock:lock-enabled"]
    assert settings(b) == ORIGINAL and b.dlc.params["laser1:scan:enabled"] is True  # as it was
    learned = r.result.learned
    assert learned["lp_code"] == ord("t") and learned["lp_kind"] == "t"
    assert learned["peak_y"] == pytest.approx(0.8, abs=0.1)  # its sampled top: 90 % of it, at least
    assert learned["falc_sign"] == 1 and learned["shg_area"] > 0 and learned["shg_side"] < 0.1
    assert "single mode" in r.result.summary and "FALC input offset" in r.result.summary
    rec_ = r.result.record
    assert rec_["maps"] and rec_["scan"]["choice"][0] is not None and rec_["sweeps"] and rec_["falc"]["dc0"] > 0.008


def test_shg_lock_jumped_a_peak_is_quick():
    b, lock = shg_bench(seed=12)
    lock.jump(-1)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done" and rec.maps == 0, r.error
    assert lock.peak == pytest.approx(b.target, abs=1e-12) and lock.shg_state()[0] == 5 and b.I == I0


def test_learned_settings_make_the_next_recovery_shorter():
    b, lock = shg_bench(seed=13)
    b.laser.nudge(-3.9)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    p = HopParams.from_dict({**SHG.to_dict(), **r.result.learned, "kv": r.result.kv, "spacing": r.result.spacing})
    b.laser.nudge(+3.9)
    b, rec2, r2 = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r2.state == "done", r2.error
    assert sum(1 for n, _ in b.dlc.log if n.endswith("select-lockpoint")) == 2  # the right type number at once
    assert lock.peak == pytest.approx(b.target, abs=1e-12) and b.single_mode()


def test_the_shg_sweep_sees_a_second_mode_the_wavemeter_does_not():
    """Masters whose modes run multimode over a wide range, invisibly to the wavemeter: with the
    SHG checks the operating point is single-mode every time."""
    seen = {True: 0, False: 0}
    for seed in range(12):
        rng = random.Random(seed)
        fsr = rng.uniform(4.5, 7.5)
        laser = ECDL(400.0, fsr=fsr, hysteresis=0.05 * fsr, multimode=rng.uniform(0.2, 0.3) * fsr, mm_noise=0.0,
                     ki=rng.uniform(-0.4, -0.15), ksi=rng.uniform(-3, -1.5), seed=seed)
        laser.nudge(rng.uniform(-0.12, 0.12) * fsr)
        laser.m = 0
        direction = rng.choice([-1, 1])
        for shg in (True, False):
            las = ECDL(400.0, fsr=fsr, hysteresis=laser.hysteresis, multimode=laser.multimode, mm_noise=0.0,
                       ki=laser.ki, ksi=laser.ksi, seed=seed)
            las.shift = laser.shift
            b, lock = locked_bench(las, seed=seed, shg=True, lockpoints=True)
            las.nudge(direction * (0.5 * fsr + las.hysteresis + 0.3))
            p = HopParams(enabled=True, i_nom=I0, cavity=True, shg=shg)
            b, rec, r = recover(las, b.target, p, cavity=True, bench=b, seed=seed)
            assert r.state == "done", (seed, shg, r.error)
            assert lock.peak == pytest.approx(b.target, abs=1e-12)
            seen[shg] += b.single_mode()
    assert seen[True] == 12 and seen[False] < 12


def test_no_tall_peak_where_the_target_is_leaves_it_there_unlocked():
    b, lock = shg_bench(seed=14)
    b.target += 700e-6  # the setpoint is between two peaks: the wavemeter and the cavity disagree
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "failed" and not r.restored and "cannot tell which cavity peak is right" in r.error
    assert not lock.locked() and b.dlc.params["laser1:scan:enabled"] is True  # unlocked, scanning, for you...
    assert lock.shg_state()[0] == 5 and settings(b) == ORIGINAL
    b.dlc.params["laser1:scan:enabled"] = False
    assert abs(b.off_mhz()) < 10  # ...around its target


def test_a_lockpoint_the_dlc_pro_will_not_take():
    from wmlock.sim import CommandError

    b, lock = shg_bench(seed=15)
    lock.jump(1)

    def never(x, y, kind):
        raise CommandError("-7 lockpoint not found")

    b.dlc.commands["laser1:dl:lock:select-lockpoint"] = never
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "failed" and "would not take the lockpoint" in r.error
    assert [args[2] for n, args in b.dlc.log if n.endswith("select-lockpoint")] == [116, 116, 0, 0, 1, 1, 2, 2, 3, 3]
    assert not lock.locked() and b.dlc.params["laser1:scan:enabled"] is True


def test_the_lockpoint_type_number_is_found():
    b, lock = shg_bench(seed=16)
    lock.jump(1)
    real = b.dlc.commands["laser1:dl:lock:select-lockpoint"]
    b.dlc.commands["laser1:dl:lock:select-lockpoint"] = lambda x, y, kind: real(x, y, 116 if kind == 0 else -1)
    p = HopParams.from_dict({**SHG.to_dict(), "lp_code": 3})  # learned wrong, say
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done" and r.result.learned["lp_code"] == 0
    assert [args[2] for n, args in b.dlc.log if n.endswith("select-lockpoint")] == [3, 3, 116, 116, 0]


def test_falc_offset_left_alone_when_the_trace_does_not_show_it():
    b, lock = shg_bench(seed=17, falc_in_trace=False)
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert b.writes("falc1:input:offset")[-1] == 0.0 and r.result.learned["falc_sign"] == 0
    assert any("does not follow" in n for n in rec.notes)


def test_falc_offset_moves_no_further_than_allowed():
    b, lock = shg_bench(seed=18, pdh_dc=0.08)
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert min(b.writes("falc1:input:offset")) == pytest.approx(-0.05) and lock.locked()


def test_step_by_step():
    b, lock = shg_bench(seed=19)
    b.laser.nudge(+3.9)
    p = HopParams.from_dict({**SHG.to_dict(), "confirm": True})
    rec = Recenter(p, I0, V0, 148.0, 152.0, 60.0, 80.0, cavity=True)
    runner, asked = Runner(rec, b, b.t), []
    while runner.busy:
        if b.pending:
            b.pending.pop(0)()
        elif runner.asking:
            asked.append(runner.asking)
            assert runner.stage and not b.pending
            runner.confirm()
        else:
            runner.on_reading(*b.reading())
    assert runner.state == "done", runner.error
    steps = ["Take the laser over: turn its cavity lock off (now locked), turn the SHG cavity's lock off",
             "Map the modes: sweep the current 148.00 … 152.00 mA in 0.1 mA steps (80 readings) at 70.00 V, "
             "scanning the SHG cavity", "Go to ", "Lock the SHG cavity (single mode", "Start the scan and find the "
             "cavity peak", "Null the error signal's DC level (+9.0 mV): FALC input offset +0.0000 → -0.0090 V",
             "Select the lockpoint at ", "Lock to the peak at "]
    assert len(asked) == len(steps) and all(a.startswith(s) for a, s in zip(asked, steps)), asked


def test_stopping_at_a_step_puts_everything_back():
    b, lock = shg_bench(seed=20)
    b.laser.nudge(+3.9)
    p = HopParams.from_dict({**SHG.to_dict(), "confirm": True})
    rec = Recenter(p, I0, V0, 148.0, 152.0, 60.0, 80.0, cavity=True)
    runner = Runner(rec, b, b.t)
    while runner.busy:
        if b.pending:
            b.pending.pop(0)()
        elif runner.asking.startswith("Lock the SHG cavity"):
            runner.cancel("stopped")
        elif runner.asking:
            runner.confirm()
        else:
            runner.on_reading(*b.reading())
    assert runner.state == "failed" and runner.restored and (b.I, b.V) == (I0, V0)
    assert b.dlc.params["laser1:dl:lock:lock-enabled"] is False  # its cavity lock stays off...
    assert b.dlc.params["laser1:nlo:shg:lock:lock-enabled"] is True  # ...all else as it was
    assert b.dlc.params["laser1:nlo:shg:scan:enabled"] is False and b.dlc.params["laser1:scan:enabled"] is True
    assert settings(b) == ORIGINAL


def test_a_failure_puts_every_setting_back():
    b, lock = shg_bench(seed=21)
    b.dlc.params["laser1:nlo:shg:scope:channel1:signal"] = 83  # the SHG scope shows the SHG power
    b.target += 60e-3  # 60 GHz away: out of reach
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "failed" and r.restored and "mode" in r.error, r.error
    assert b.writes("nlo:shg:scope:channel1:signal") == [82, 83]  # borrowed, given back
    assert b.dlc.params["laser1:nlo:shg:lock:lock-enabled"] is True and not lock.locked()
    assert b.dlc.params["laser1:nlo:shg:scan:enabled"] is False and (b.I, b.V) == (I0, V0)


def test_a_wavemeter_locked_shg_laser():
    laser, target = hopped_laser(1, seed=22)
    b = Bench(laser, target)
    lock = b.lock_it(shg=True, falc=0)  # an SHG stage, no cavity lock: this program locks it
    b.dlc.params["laser1:dl:lock:lock-enabled"] = False
    p = HopParams(enabled=True, i_nom=I0, shg=True)
    b, rec, r = recover(laser, target, p, bench=b)
    assert r.state == "done" and laser.m == 0 and lock.shg_state()[0] == 5, r.error
    assert "dl:lock:lock-enabled" not in [n.replace("laser1:", "") for n, _ in b.dlc.log]


def test_the_runner_refuses_anything_else():
    class Rogue(Recenter):
        def __init__(self, req, **kw):
            super().__init__(HopParams(falc_max=0.05), I0, V0, 148.0, 152.0, 60.0, 80.0, **kw)
            self.req = req

        def run(self):
            yield Set("falc1:input:offset", 0.0, restore=0.0)
            yield self.req

    from wmlock.modehop import Exec

    laser = ECDL(400.0)
    for req, why in ((Set("dl:cc:current-clip", 300.0), "refused to set dl:cc:current-clip"),
                     (Set("falc1:input:offset", 0.2, restore=0.0), "refused to set falc1:input:offset"),
                     (Set("scan:enabled", 1), "refused to set scan:enabled"),
                     (Exec("dl:lock:open"), "refused to run dl:lock:open")):
        b = Bench(laser, laser.freq(V0, I0))
        b.dlc.params["falc1:input:offset"] = 0.0
        r = b.run(Rogue(req))
        assert r.state == "failed" and why in r.error, r.error
        assert all(not n.endswith(("current-clip", "lock:open")) for n, _ in b.dlc.log)


def test_monte_carlo_over_random_shg_lasers():
    """Random masters, going multimode over a good part of each mode and quietly for the wavemeter,
    hopping 1 or 2 modes; a random DC level on the error signal."""
    ok = feasible = 0
    for seed in range(60):
        laser, hop, noise = random_laser(1000 + seed)
        laser.multimode, laser.mm_noise = max(laser.multimode, 0.15 * laser.fsr), 0.0
        b, lock = locked_bench(laser, seed=seed, shg=True, lockpoints=True,
                               pdh_dc=random.Random(seed).uniform(-0.03, 0.03))
        laser.nudge(hop)
        best = best_margin(laser, SHG)
        b, rec, r = recover(laser, b.target, SHG, noise=noise, cavity=True, bench=b, seed=seed)
        if r.state == "done":
            assert lock.peak == pytest.approx(b.target, abs=1e-12), seed
            assert lock.shg_state()[0] == 5 and b.second_mode() < 0.1, seed  # what an SHG sweep can see
            assert abs(lock.dc()) < 0.003, seed
        else:
            assert not lock.locked() and settings(b) == ORIGINAL, seed
        if best >= 0.5:
            feasible += 1
            ok += r.state == "done"
    assert feasible >= 25 and ok >= 0.9 * feasible, (ok, feasible)


def test_lockpoint_candidates_that_move_between_reads():
    """The free-running laser drifts while the scan runs: every scan finds the peaks a little
    further on, so a lockpoint chosen from an earlier read is 'not found'. Read just before."""
    b, lock = shg_bench(seed=23)
    lock.jump(1)
    scan = lock._master_scan

    def drifting():
        lock.wobble += 5.0  # MHz per scan, 12 mV of scan
        return scan()

    lock._master_scan = drifting
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert lock.peak == pytest.approx(b.target, abs=1e-12)
    assert [args[2] for n, args in b.dlc.log if n.endswith("select-lockpoint")] == [116]
