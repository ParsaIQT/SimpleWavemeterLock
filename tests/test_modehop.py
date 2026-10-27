"""Mode-hop recovery against a simulated external-cavity diode laser and DLC pro, driven
synchronously: the procedure talks to a FakeDLCPro through the real client, in the same thread.

Every run checks the safety rules: the current never leaves its window and never changes by more
than STEP_MAX in one write, and nothing is written that is not allowed."""
import math
import random
import re

import pytest

from wmlock.dlcpro import DecopError, DLCPro, lock_points
from wmlock.modehop import (BUSY_TRIES, COMMANDS, FALC_OFFSET, STEP_MAX, WRITABLE, Failed, Guard, HopParams, Move,
                            Point, Read, Recenter, Runner, Set, _steps, analyse, perform)
from wmlock.sim import ECDL, CommandError, FakeDLCPro, SimLocked

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

    def __init__(self, laser, target, noise=0.5, dt=0.1, seed=0, current=I0, offset=V0, scan=False, drift=0.0):
        self.laser, self.target = laser, target
        self.drift, self.wobble = drift, 0.0  # MHz: its random walk per reading (a free-running laser's), where it is
        self.start = (current, offset)
        self.dlc = FakeDLCPro("SN-B", "bench", {1: {"current": current, "offset": offset, "scan": scan}},
                              serve=False)
        self.client = Loopback(self.dlc)
        self.lock = None
        self.shg_usual = 0.0  # V: its SHG cavity's intra-cavity signal, locked, as an engine would have learned it
        self.noise, self.dt, self.t = noise, dt, 0.0
        self.rng = random.Random(seed)
        self.pending = []
        self.refuse = None  # (part of a name, exception): fail the next operation on it, once
        for name in ("dl:cc:current-set", "scan:offset"):  # the laser follows each change as it is made
            self.dlc.on_set["laser1:" + name] = self._settle

    def _settle(self, _value):
        if self.lock is None or not self.lock.locked():
            self.laser.lase(self.V + (self.lock.dv if self.lock is not None else 0.0), self.act)

    def lock_it(self, **kwargs) -> SimLocked:
        kwargs.setdefault("clock", lambda: self.t)  # (its TA's protection runs on the bench's time)
        self.lock = SimLocked(self.dlc, 1, self.laser, self.target, **kwargs)
        return self.lock

    @property
    def I(self):  # noqa: E743 (as in the formulas)
        return self.dlc.params["laser1:dl:cc:current-set"]

    @property
    def V(self):
        return self.dlc.params["laser1:scan:offset"]

    @property
    def act(self):
        """The current the laser runs at (measured)."""
        return self.dlc.current_act(1)

    def freq(self):
        if self.lock is not None:
            return self.lock.frequency(self.wobble)
        return self.laser.freq(self.V, self.act)

    def reading(self):
        self.t += self.dt
        if self.drift:
            self.wobble += self.rng.gauss(0.0, self.drift)
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
        return (las.fsr / 2 - las.hysteresis - abs(las.misalignment(las.m, v, self.act))) / abs(las.ki - las.ksi)

    def second_mode(self):
        """The power fraction in a second mode at the operating point."""
        las = self.laser
        v = self.V + (self.lock.dv if self.lock is not None else 0.0)
        las.lase(v, self.act)
        comps = las.components(v, self.act)
        return comps[1][0] if len(comps) > 1 else 0.0

    def single_mode(self):
        return self.second_mode() == 0.0

    def off_mhz(self):
        return (self.freq() - self.target) * 1e6


ALLOWED = {"laser1:dl:cc:current-set", "laser1:scan:offset"} | {"laser1:" + k for k in WRITABLE}


def measured_currents(b):
    """The current the laser ran at (measured: with the feedforward, the piezo moves it too) when the recovery
    began and after each write of current or offset."""
    d = b.dlc.params
    ref = b.dlc.ff_from.get(1, 0.0)  # (None: the feedforward adds nothing as the scan's offset moves the piezo)
    ff = d["laser1:dl:cc:feedforward-factor"] if d["laser1:dl:cc:feedforward-enabled"] and ref is not None else 0.0
    base = d["laser1:dl:cc:current-offset"] + b.dlc.bias.get(1, 0.0)
    i, v = b.start
    out = [i + base + ff * (v - (ref or 0.0))]
    for name, value in b.dlc.log:
        if name == "laser1:dl:cc:current-set":
            i = value
        elif name == "laser1:scan:offset":
            v = value
        else:
            continue
        out.append(i + base + ff * (v - (ref or 0.0)))
    return out


def check_safety(b, i_lo, i_hi, p):
    """i_lo, i_hi: the window as set currents with the piezo where it began; it is on the measured current."""
    acts = measured_currents(b)
    off = acts[0] - b.start[0]
    eps = 1e-6  # (what goes over the wire: values written to six decimals)
    assert all(i_lo + off - eps <= a <= i_hi + off + eps for a in acts), "current outside its window"
    assert all(abs(x - y) <= STEP_MAX + eps for x, y in zip(acts, acts[1:])), "measured current jumped"
    currents = [b.start[0]] + b.writes("dl:cc:current-set")
    assert all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip(currents, currents[1:])), "current jumped"
    falc0 = {}
    for name, value in b.dlc.log:
        if isinstance(value, tuple):  # a command
            assert name.split(":", 1)[1] in COMMANDS, name
        elif FALC_OFFSET.match(name):
            falc0.setdefault(name, value)
        else:
            assert name in ALLOWED, f"wrote {name}"


def recover(laser, target, p=None, noise=0.5, kv=0.0, cavity=False, quick=True, seed=0, bench=None, learned=True):
    """The windows as the engine makes them: i_nom - i_below ... i_nom + i_window, up to i_nom + jump_max
    if it stops lasing (the map method's tests let it go 2 mA below). learned: what the engine learns
    while the laser runs (its SHG signal, locked) is known."""
    p = p or HopParams(enabled=True, i_nom=I0, cavity=cavity, i_below=2.0)
    b = bench or Bench(laser, target, noise, seed=seed)
    if learned and b.shg_usual and p.shg and not p.shg_lock:
        p = HopParams.from_dict({**p.to_dict(), "shg_lock": b.shg_usual})
    b.noise = noise
    i_lo, i_hi, i_top = p.i_nom - p.i_below, p.i_nom + p.i_window, p.i_nom + p.jump_max
    rec = Recenter(p, b.I, b.V, i_lo, i_hi, b.V - p.v_window, b.V + p.v_window, kv=kv, cavity=cavity, quick=quick,
                   i_top=i_top)
    runner = b.run(rec)
    check_safety(b, i_lo, i_top if rec.panics or rec.resets_done else i_hi, p)  # (up to i_top: a panic, a reset)
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
    if lock.has_shg:
        b.shg_usual = lock._intra_now()
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
    m = re.fullmatch(r"relocking failed 3 times, last: the cavity lock caught a peak \+(\d+) MHz away", r.error)
    assert m and abs(int(m.group(1)) - 1500) <= 3, r.error  # (the wavemeter's noise aside)
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
    assert r.state == "failed" and r.restored and "piezo can bring onto the target" in r.error, r.error
    assert float(re.search(r"the nearest mode was ([\d.]+) GHz", r.error).group(1)) > 45  # (less the tuning)
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
            di = (p.i_window if k > 0 else p.i_below) * k / 100
            dv = -(m * laser.fsr + laser.ki * di) / laser.kv
            if abs(dv) <= p.v_window:
                a = m * laser.fsr + (laser.kv - laser.ksv) * dv + (laser.ki - laser.ksi) * di - laser.shift
                best = max(best, (laser.fsr / 2 - laser.hysteresis - abs(a)) / abs(laser.ki - laser.ksi))
    return best


def test_monte_carlo_over_random_lasers():
    p = HopParams(enabled=True, i_nom=I0, i_below=2.0)
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


# ---------------------------------------------- hysteresis: which way the current comes
def hysteretic_laser(h, a0, **kwargs):
    """A laser on its target mode, a0 GHz from the middle of it, whose modes hold on h GHz past where
    they would not have started: sweeping up, it hops onto a mode later than it hops off it sweeping
    down, and in between either mode can lase, whichever it was on."""
    laser = ECDL(400.0, hysteresis=h, seed=1, **kwargs)
    target = laser.freq(V0, I0)
    laser.nudge(-a0)
    laser.freq(V0, I0)
    assert laser.m == 0
    return laser, target


def test_a_mode_found_sweeping_one_way_is_used_sweeping_that_way():
    """As on your laser: sweeping up, the target's mode lases over 2.5 mA of the window; sweeping
    down, only over its last 0.1 mA. Where it lases both ways is too narrow to use: it is used as found
    sweeping up, parked by sweeping up to the middle, and holds there."""
    laser, target = hysteretic_laser(2.7, 3.5)
    b, rec, r = recover(laser, target, quick=False)
    assert r.state == "done" and rec.maps == 1, r.error
    (mode,) = [m for m in rec.modes if abs(m.at(I0)) < 1000]
    assert sum(z - a for a, z in mode.up) > 2.0 and sum(z - a for a, z in mode.stable) <= 0.1 + 1e-9
    assert any(n.startswith("parked at 148.97 mA sweeping up") for n in rec.notes), rec.notes
    parking = [pt[0] for pt in rec.record["maps"][0]["points"] if pt[2] == 3]
    assert parking[0] < 148.2 and all(y > x for x, y in zip(parking, parking[1:]))  # from the bottom, up
    assert laser.m == 0 and abs(b.off_mhz()) < 5
    assert laser.margins(b.V, b.act)[1] > 2.0  # mA to where it would hop off, either way


def test_parking_the_other_way_finds_another_mode():
    """The same current, come to from above, gives the neighbouring mode: why it parks the way it found it."""
    laser, target = hysteretic_laser(2.7, 3.5)
    b, rec, r = recover(laser, target, quick=False)
    assert r.state == "done" and laser.m == 0
    for x in _steps(b.act, 152.0, 0.1) + _steps(152.0, b.act, 0.1):  # over the top, and back down to it
        laser.lase(b.V, x)
    assert laser.m == -1 and laser.margins(b.V, b.act)[1] > 0  # (it would hold that one too)


def test_a_failed_recovery_leaves_the_laser_on_the_mode_it_began_on():
    """With hysteresis, putting the current back is not enough: the laser can come back on another
    mode. Giving up, it goes back the way that gives the mode it was on, as the first map saw."""
    laser, f0 = hysteretic_laser(2.2, -3.0)
    start = laser.freq(V0, I0)
    b, rec, r = recover(laser, f0 + 60e-3, quick=False)  # 60 GHz away: out of reach
    assert r.state == "failed" and r.restored and (b.I, b.V) == (I0, V0), r.error
    assert any(n.startswith("back at 150.00 mA sweeping down") for n in rec.notes), rec.notes
    assert laser.m == 0 and b.freq() == pytest.approx(start, abs=1e-9)  # as it was
    laser.lase(V0, 148.0)  # (down to the bottom of the window and straight back: the next mode)
    laser.lase(V0, I0)
    assert laser.m == 1


def test_scattered_readings_are_taken_for_two_modes():
    """On two modes at once the wavemeter reads now one, now the other: readings scattered about the
    mode's line are cut out of its ranges, as where the SHG sweep shows two modes."""
    rng = random.Random(3)
    pts = []
    for leg, (a, b, up) in enumerate(((148.0, 152.0, True), (152.0, 148.0, False)), start=1):
        for k in range(41):
            i = a + (b - a) * k / 40
            pts.append(Point(i, -250.0 * (i - 150.0) + rng.gauss(0, 30.0 if i > 151.0 else 1.0), leg, up))
    res = analyse(pts, 0.1)
    (m,) = res.modes
    assert res.noise < 3 and m.up[0][0] == pytest.approx(148.0) and m.down[0][0] == pytest.approx(148.0)
    widest = max(m.hold, key=lambda r: r[1] - r[0])
    assert widest[0] == pytest.approx(148.0) and 150.8 <= widest[1] <= 151.05
    assert sum(z - a for a, z in m.hold if a > 151.0) < 0.5  # what is left beyond: scraps


def test_currents_are_shown_as_measured():
    """The DLC pro measures 0.6 mA more than it is set to: the nominal current, the window and all it
    says are measured currents; what it writes are set currents."""
    laser = ECDL(400.0, i0=150.6, seed=1)  # (it lases on the measured current)
    target = laser.freq(V0, 150.6)
    laser.nudge(3.9)
    laser.freq(V0, 150.6)
    b = Bench(laser, target)
    b.dlc.bias[1] = 0.6
    p = HopParams(enabled=True, i_nom=150.6)
    rec = Recenter(p, I0, V0, 148.0, 152.0, 60.0, 80.0, act_offset=0.6)
    assert rec.nominal == pytest.approx(150.0)
    r = b.run(rec)
    check_safety(b, 148.0, 152.0, p)
    assert r.state == "done", r.error
    assert laser.m == 0 and abs(b.off_mhz()) < 5
    assert r.result.summary.startswith("recentered 150.60 → ") and "mA measured" in r.result.summary
    assert rec.final_act == pytest.approx(b.act) and rec.notes[0].startswith("map 1 at 70.00 V")
    assert "148.60" in rec.notes[0] or "152.60" in rec.notes[0] or "sweeping" in rec.notes[0]
    assert min(pt[0] for pt in rec.record["maps"][0]["points"]) == pytest.approx(148.6)


def strong_case(seed):
    """A random laser with strong hysteresis (up to 0.6 FSR, more one way than the other), on its
    target mode anywhere it can lase: recentering it."""
    rng = random.Random(seed)
    fsr = rng.uniform(3, 10)
    kv = rng.choice([-1, 1]) * rng.uniform(0.2, 0.8)
    laser = ECDL(400.0, fsr=fsr, kv=kv, ki=rng.uniform(-0.5, -0.1), ksv=kv * rng.uniform(0.6, 0.95),
                 ksi=rng.uniform(-4, -1), hysteresis=rng.uniform(0.15, 0.6) * fsr,
                 multimode=rng.uniform(0.02, 0.1) * fsr, seed=seed, skew=rng.uniform(-0.5, 0.5))
    lo, hi = laser.limits()
    target = laser.freq(V0, I0)
    laser.nudge(-rng.uniform(lo + 0.2, hi - 0.2))
    laser.freq(V0, I0)
    return laser, target, rng.choice([0.3, 1.0, 3.0])


def test_monte_carlo_over_strongly_hysteretic_lasers():
    done = 0
    for seed in range(80):
        laser, target, noise = strong_case(seed)
        start = laser.freq(V0, I0)
        b, rec, r = recover(laser, target, noise=noise, seed=seed, quick=False)
        if r.state == "done":
            done += 1
            assert abs(b.off_mhz()) < 5 + 3 * noise, seed
            laser.lase(b.V, b.act)
            assert laser.margins(b.V, b.act)[1] > 0.25, seed  # well clear of where it hops off
            assert len(laser.components(b.V, b.act)) == 1, seed  # one mode
        else:
            assert r.restored and (b.I, b.V) == (I0, V0), seed
            if rec.home is not None:
                assert b.freq() == pytest.approx(start, abs=1e-9), seed  # on the mode it began on
    assert done >= 0.85 * 80, done


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
    while b.pending:  # (it asks for the seed power first: this laser has none)
        b.pending.pop(0)()
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
                  kv=-410.0, lp_kind="t", lp_code=116, shg_n=2, method="map", seed="amp:seed-limits:power",
                  seed_ok=19.5, jump=3.0)
    q = HopParams.from_dict(p.to_dict())
    assert q == p and isinstance(q.max_per_hour, int) and q.cavity is True and q.lp_kind == "t"
    assert HopParams.from_dict(None) == HopParams()
    assert HopParams.from_dict({"lp_code": "nonsense", "step": "0.2"}).step == 0.2  # bad values are skipped
    f = p.forget()
    assert f.kv == 0 and f.lp_code == -1 and f.lp_kind == "" and f.shg and f.i_nom == 151.25 and f.seed_ok == 0
    assert f.method == "map" and f.seed == "amp:seed-limits:power" and f.jump == 3.0
    assert HopParams(shg=True).how() == "catch" and HopParams(method="walk").how() == "walk"
    assert HopParams().how() == "map" and HopParams(shg=True, method="map").how() == "map"
    o = HopParams(shg=True, opt_i=150.2, opt_v=71.5, opt_shg=1.23, opt_t="2026-10-19 21:03", shg_ratio=0.97).forget()
    assert o.optimum and (o.opt_i, o.opt_v, o.opt_shg) == (150.2, 71.5, 1.23) and o.shg_ratio == 0  # it was set by hand
    assert not HopParams().optimum and HopParams.from_dict(o.to_dict()) == o
    assert HopParams().seed_names == ("amp:pd:seed:power", "amp:seed-limits:power")
    assert HopParams(seed="off").seed_names == ()
    assert HopParams(seed=" dl:pd:seed:power ").seed_names == ("dl:pd:seed:power",)
    assert HopParams().seed_least(20.0, 8.0) == 10.0 and HopParams().seed_least(0.0, 8.0) == 8.0
    assert HopParams().seed_least(0.0) == 0.0  # no telling
    old = HopParams.from_dict({"cavity": True, "lock_ok": 0.2})  # learned as an earlier version did, and wrong at that:
    assert old.trans_locked == 0.0 and old.trans_zero == 0.0 and "lock_ok" not in old.to_dict()  # learned afresh


def test_the_transmission_input_and_levels():
    from wmlock.relock import transmission_input, usable_zero, volts

    assert transmission_input(True, 0, 0) == "io:fine-1:value-act"  # Fine In 1, both
    assert transmission_input(True, 1, 0) == "io:fine-2:value-act"  # its lock detection's, when that is on
    assert transmission_input(False, 1, 0) == "io:fine-1:value-act"  # else the lock input
    assert transmission_input(True, 42, 2) == "io:fast-3:value-act"  # (an input without a present value: the other)
    assert transmission_input(None, None, None) is None
    assert usable_zero(0.01, 0.06) == 0.01 and usable_zero(-0.004, 0.05) == -0.004
    assert usable_zero(0.04, 0.06) == 0.0 and usable_zero(0.01, 0.0) == 0.0  # not well below locked; no telling
    assert volts(0.0493, 0.05) == "0.0493" and volts(0.8) == "0.800" and volts(0.0012, 0.06) == "0.0012"
    assert volts(1.23) == "1.230" and volts(0.0) == "0.000" and volts(0.005) == "0.00500"


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
SHG = HopParams(enabled=True, i_nom=I0, cavity=True, shg=True, method="walk")
SHG_CATCH = HopParams(enabled=True, i_nom=I0, cavity=True, shg=True)  # by default (auto), the catch
SHG_MAP = HopParams(enabled=True, i_nom=I0, cavity=True, shg=True, method="map", i_below=2.0, v_window=10.0)
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


def lab_bench(seed=21, drift=0.0, **kwargs):
    """The SHG laser as in the lab (SimLocked.LAB): its reference cavity's lines a sample or so wide in a scan of
    0.31 V, its PDH sidebands 20 MHz out (the dispersion's tails some 0.04 V either side of the carrier between
    them), two weaker lines 52 MHz either side; the error signal's DC level `pdh_dc`, nulled when the lockpoint was
    picked by hand (none), unless given."""
    laser = ECDL(400.0, seed=seed, multimode=0.9, hysteresis=0.4, mm_noise=0.004)
    b = Bench(laser, laser.freq(V0, I0), seed=seed, scan=True, drift=drift)
    b.dlc.params["laser1:scan:amplitude"] = SimLocked.LAB_SCAN
    lock = b.lock_it(seed=seed, shg=True, **{"lockpoints": True, **SimLocked.LAB, **kwargs})
    lock.settle()
    assert lock.locked() and abs(b.off_mhz()) < 1
    b.shg_usual = lock._intra_now()
    return b, lock


@pytest.mark.parametrize("p", [SHG, SHG_MAP], ids=["walk", "map"])
def test_shg_laser_relocked_the_way_you_would_by_hand(p):
    b, lock = shg_bench()
    b.laser.nudge(+3.9)  # it hops; its lock relocks 4 peaks away and calls that locked
    assert b.off_mhz() == pytest.approx(6000, abs=1) and lock.locked()
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    # on the right TEM00 peak, single-mode with margins, SHG cavity locked, error-signal DC nulled
    assert lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12)
    assert b.single_mode() and b.margin() > 0.3 and lock.shg_state() == (5, "locked")
    assert abs(lock.dc()) < 0.002 and b.writes("falc1:input:offset")[-1] == pytest.approx(-0.009, abs=0.001)
    # in your order: both locks off, scan the SHG cavity and find a single-mode mode on target, lock the SHG cavity,
    # scan (its lock points shown: found afresh before each look at them), null, pick, lock
    # (the input offset a little, then more: how much it moves the error signal is not known yet)
    order = [n.replace("laser1:", "") for n, _ in b.dlc.log if "cc:current" not in n and "scan:offset" not in n]
    falc = order.count("falc1:input:offset")
    assert 2 <= falc <= 4 and order == [
        "dl:lock:lock-enabled", "nlo:shg:lock:lock-enabled", "scan:enabled", "nlo:shg:scan:enabled",
        "nlo:shg:scan:enabled", "nlo:shg:lock:lock-enabled", "scan:enabled", "dl:lock:find-candidates"] + [
        "falc1:input:offset"] * falc + ["dl:lock:find-candidates", "dl:lock:select-lockpoint", "dl:lock:lock-enabled"]
    assert settings(b) == ORIGINAL and b.dlc.params["laser1:scan:enabled"] is True  # as it was
    learned = r.result.learned
    assert learned["lp_code"] == ord("t") and learned["lp_kind"] == "t"
    assert learned["peak_y"] == pytest.approx(0.8, abs=0.1)  # its sampled top: 90 % of it, at least
    assert learned["falc_sign"] == 1 and learned["falc_gain"] == pytest.approx(1.0, rel=0.25)
    assert learned["shg_area"] > 0 and learned["shg_side"] < 0.1
    assert learned["seed_ok"] == pytest.approx(20.0, abs=0.5)  # its TA's seed, lasing as usual
    assert "single mode" in r.result.summary and "FALC input offset" in r.result.summary
    rec_ = r.result.record
    assert rec_["scan"]["choice"][0] is not None and rec_["sweeps"] and rec_["falc"]["dc0"] > 0.008
    if p.method == "map":
        assert rec_["maps"] and "recentered" in r.result.summary
    else:  # the piezo walked the mode it hopped to onto the target: 6 GHz, some 15 V
        assert not rec_["maps"] and rec_["walk"] and "walked the piezo 70.00 → 5" in r.result.summary
        assert abs(b.I - I0) <= 1.0 and rec.method == "walk"


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
            if shg:
                assert r.state == "done", (seed, r.error)
                assert lock.peak == pytest.approx(b.target, abs=1e-12)
            seen[shg] += r.state == "done" and b.single_mode()  # (without, two modes may not even hold the lock)
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
    assert [args[2] for n, args in b.dlc.log if n.endswith("select-lockpoint")] == [116, 116, 0, 0, 1, 1, 2, 2, 3, 3]
    # the lockpoint picked by hand is still there, at the target: locked with that, as you would
    assert r.state == "done" and lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12), r.error
    assert any("would not take the lockpoint: locking with the lockpoint the DLC pro has" in n for n in rec.notes)

    b, lock = shg_bench(seed=15)
    lock.jump(1)
    lock.lockpoint = None  # none selected: left for you
    b.dlc.commands["laser1:dl:lock:select-lockpoint"] = never
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "failed" and "would not take the lockpoint" in r.error and "select the peak by hand" in r.error
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


def test_falc_offset_that_subtracts_is_learned_then_nulled_in_one_step():
    b, lock = shg_bench(seed=24, falc_sign=-1)
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done" and r.result.learned["falc_sign"] == -1, r.error
    first = b.writes("falc1:input:offset")
    assert len(first) >= 2 and first[-1] == pytest.approx(0.009, abs=0.001) and abs(lock.dc()) < 0.002
    lock.pdh_dc = 0.02  # it drifts again: 11 mV more
    lock.jump(1)
    p = HopParams.from_dict({**SHG.to_dict(), **r.result.learned})
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert len(b.writes("falc1:input:offset")) == len(first) + 1  # knowing which way: one step
    assert abs(lock.dc()) < 0.002


def test_falc_offset_moves_no_further_than_allowed():
    b, lock = shg_bench(seed=18, pdh_dc=0.08)
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert min(b.writes("falc1:input:offset")) == pytest.approx(-0.05) and lock.locked()


@pytest.mark.parametrize("scan", [True, False], ids=["scanning-before", "not-scanning-before"])
def test_falc_offset_nulled_in_a_relock_without_lockpoint_too(scan):
    """Locking without a lockpoint, it relocks by re-enabling the lock, but has the scan on for a moment first, as
    you would: the error signal's DC level is nulled, and the transmission's zero measured."""
    laser = ECDL(400.0, seed=11, multimode=0.9, hysteresis=0.4, mm_noise=0.004)
    b, lock = locked_bench(laser, seed=11, shg=True, lockpoints=False, pdh_dc=0.009, t_dc=0.05, scan=scan)
    lock.jump(1)
    p = HopParams.from_dict({**SHG.to_dict(), "trans_locked": 0.85})  # (learned while it ran)
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert rec.relock_mode == "plain" and lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12)
    assert abs(lock.dc()) < 0.002 and b.writes("falc1:input:offset")[-1] == pytest.approx(-0.009, abs=0.001)
    assert r.result.learned["trans_zero"] == pytest.approx(0.05, abs=0.01) and "FALC input offset" in r.result.summary
    order = [n.replace("laser1:", "") for n, _ in b.dlc.log if "cc:current" not in n and "scan:offset" not in n]
    tail = order[order.index("nlo:shg:lock:lock-enabled", 2) + 1:]  # after the SHG cavity is locked
    assert tail == ["scan:enabled"] + ["falc1:input:offset"] * tail.count("falc1:input:offset") + (
        [] if scan else ["scan:enabled"]) + ["dl:lock:lock-enabled"]
    assert b.dlc.params["laser1:scan:enabled"] is scan  # as it was


@pytest.mark.parametrize("gain, sign", [(1.0, 0), (8.0, 0), (-3.0, 0), (-9.6, -1)])
def test_the_lab_error_signal_sits_at_zero_neatly_afterwards(gain, sign):
    """The error signal as on the lab's screen, 20 mV of DC on it, and a FALC input offset that shows in it `gain`
    times over (not known yet; the lab's: -9.6, its sign learned): afterwards it sits at zero, and on the way it never
    goes much further from it than where it began (a little of the way first, as if one to one, tells how much).
    Knowing that, the next time it goes straight there."""
    b, lock = lab_bench(pdh_dc=0.02, falc_gain=gain)
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, HopParams.from_dict({**SHG.to_dict(), "falc_sign": sign}), cavity=True,
                        bench=b)
    assert r.state == "done" and lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12), r.error
    assert abs(lock.shown_dc()) < 0.0012
    assert max(abs(0.02 + gain * o) for o in b.writes("falc1:input:offset")) < 0.027
    assert r.result.learned["falc_gain"] == pytest.approx(gain, rel=0.2)
    lock.pdh_dc = 0.032  # it drifts: 12 mV again
    lock.jump(1)
    n = len(b.writes("falc1:input:offset"))
    b, rec, r = recover(b.laser, b.target, HopParams.from_dict({**SHG.to_dict(), **r.result.learned}), cavity=True,
                        bench=b)
    assert r.state == "done" and abs(lock.shown_dc()) < 0.0012, r.error
    assert len(b.writes("falc1:input:offset")) - n <= 2


def test_a_trace_lost_while_nulling_puts_the_offset_back(monkeypatch):
    """As in the lab's log: relocking without a lockpoint, the FALC offset was moved, then the DLC pro refused the
    scope's data as busy, and the offset was left moved (+50 mV: half a volt on the trace, the offset showing ten
    times over, inverted), the notes saying it had not been. Now it is put back where it was."""
    monkeypatch.setattr("wmlock.modehop.BUSY_WAIT", 0.0)
    b, lock = lab_bench(seed=29, pdh_dc=0.02, falc_gain=-9.6, lockpoints=False)
    lock.jump(1)
    b.dlc.busy_checks.append(lambda kind, name: kind == "get" and name == "laser1:scope:data"
                             and len(b.writes("falc1:input:offset")) >= 1)
    b, rec, r = recover(b.laser, b.target, HopParams.from_dict({**SHG.to_dict(), "falc_sign": -1}), cavity=True,
                        bench=b)
    assert r.state == "done" and lock.locked(), r.error
    assert b.dlc.params["falc1:input:offset"] == 0.0 and abs(lock.shown_dc() - 0.02) < 1e-9
    assert any("no error-signal trace after moving the FALC input offset" in n for n in rec.notes)
    assert r.result.learned.get("falc_sign", -1) == -1  # (not taken for an offset the trace does not follow)


def test_a_write_the_controller_refuses_as_busy_is_tried_again_and_never_put_back(monkeypatch):
    """As in the lab's log: the DLC pro refused to turn the cavity lock off as the recovery took the laser over
    ("-1032 data acquisition busy"), and putting things back then turned it off, which it never was. Refused a few
    times, it is tried again; refused every time, the recovery fails and the lock is left on, as it was."""
    monkeypatch.setattr("wmlock.modehop.BUSY_WAIT", 0.0)
    b, lock = shg_bench(seed=28)
    lock.jump(1)
    refused = []

    def twice(kind, name):
        if kind == "set" and name == "laser1:dl:lock:lock-enabled" and len(refused) < 2:
            refused.append(name)
            return True
        return False

    b.dlc.busy_checks.append(twice)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done" and lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12), r.error
    assert len(refused) == 2

    b, lock = shg_bench(seed=28)
    lock.jump(1)
    refused.clear()

    def every_try(kind, name):  # (and it would take the next)
        if kind == "set" and name == "laser1:dl:lock:lock-enabled" and len(refused) < BUSY_TRIES:
            refused.append(name)
            return True
        return False

    b.dlc.busy_checks.append(every_try)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "failed" and "-1032 data acquisition busy" in r.error and len(refused) == BUSY_TRIES
    assert b.writes("dl:lock:lock-enabled") == [] and b.dlc.params["laser1:dl:lock:lock-enabled"] is True
    assert lock.locked()  # (on its wrong peak still: nothing was turned off)


def test_a_learned_falc_gain_that_no_longer_holds_is_found_out_afresh():
    """Learned 8 V/V, but someone has turned the FALC's input gain down since (1 V/V): the step there moves the error
    signal an eighth of what it should, which shows, and it goes on with what it saw."""
    b, lock = lab_bench(seed=27, pdh_dc=0.02, falc_gain=1.0)
    lock.jump(1)
    p = HopParams.from_dict({**SHG.to_dict(), "falc_gain": 8.0, "falc_sign": 1})
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done" and lock.locked(), r.error
    assert abs(lock.shown_dc()) < 0.0012 and r.result.learned["falc_gain"] == pytest.approx(1.0, rel=0.2)


def test_the_lab_error_signal_already_at_zero_is_left_alone():
    """Nulled by hand when the lockpoint was picked: the error signal's DC level, looked at in three traces, is
    within its tolerance, and the input offset is not touched (the dispersion's tails between the carrier and its
    sidebands are no DC level)."""
    b, lock = lab_bench(seed=22)
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done" and lock.locked(), r.error
    assert b.writes("falc1:input:offset") == [] and "left alone" in r.result.summary


def test_narrow_peaks_sampled_a_sample_wide_are_told_and_locked():
    """The lab's cavity with lines narrower still (0.1 MHz: in a scan, under half a sample wide, it shows anything
    from a sixth of its height to all of it), the master drifting: the peak picked by hand looked as tall as it is,
    the scans of the relock may not (with these, the tallest looked 0.34 and 0.39 of that, and the relock gave up,
    unlocked and scanning). The right peak is told from a few scans (the tallest look of each peak counts), and the
    lockpoint selected on the peak chosen as the next scan shows it, however tall it looks there."""
    for seed in (20, 36):
        b, lock = lab_bench(seed=seed, lw=0.1, drift=0.15)
        lock.jump(1)
        b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
        assert r.state == "done" and lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12), (seed, r.error)
        assert not any("locking with the lockpoint the DLC pro has" in n for n in rec.notes), seed


def stop_when(b, until, then=None):
    """Relock it, stop it when until(rec) is so (then() first), and let it put things back."""
    rec = Recenter(optimum(b, kv=400.0), b.I, b.V, I0 - 1.0, I0 + 2.0, b.V - 25.0, b.V + 25.0, cavity=True,
                   i_top=I0 + 5.0)
    r = b.run(rec, until=lambda r: until(rec))
    assert r.busy
    if then is not None:
        then()
    r.cancel("stopped")
    while r.busy:
        if b.pending:
            b.pending.pop(0)()
        else:
            r.on_reading(*b.reading())
    return r


@pytest.mark.parametrize("owned", [True, False])
def test_stopped_while_its_cavity_lock_tries_to_lock_everything_goes_back(owned):
    """The 739 nm one, 23 October: stopped by hand while its cavity lock was enabled, trying to lock, its scan could
    not be put back (its DLC pro does not let its scan be switched while its lock is enabled, and the lock was put back,
    off, last). The cavity lock goes off first now."""
    b, lock = near_bench(seed=48)
    lock.scan_owned = owned
    lock._engage = lambda near=None: None  # (enabled, it never locks)
    lock.jump(1)
    d = b.dlc.params
    r = stop_when(b, lambda rec: d["laser1:dl:lock:lock-enabled"] and rec.stage == "locking the cavity")
    assert r.state == "failed" and r.error == "stopped" and not r.trouble, (r.error, r.trouble)
    assert d["laser1:dl:lock:lock-enabled"] is False and d["laser1:scan:enabled"] is True
    assert (b.I, b.V) == pytest.approx((I0, V0))


def test_a_setting_that_cannot_be_put_back_is_named_with_the_reason():
    b, lock = near_bench(seed=48)

    def refuse(_on):
        raise CommandError("-9 no (simulated)")

    r = stop_when(b, lambda rec: rec.stage.startswith(("into", "parking")),
                  then=lambda: b.dlc.on_set.update({"laser1:nlo:shg:scan:enabled": refuse}))
    assert r.error == "stopped; could not put back nlo:shg:scan:enabled (Error: -9 no (simulated))", r.error


def test_a_relock_that_fails_with_the_offset_moved_goes_back_to_it():
    """The cavity lock would not take with the input offset nulled (whatever the reason): the next try puts it back
    where it was when the recovery began, with which it was locked before, as you would by hand."""
    b, lock = shg_bench(seed=26)
    lock.jump(1)
    real = lock._engage

    def picky(near=None):
        if abs(b.dlc.params["falc1:input:offset"]) > 1e-6:
            lock.peak, lock.dv = None, 0.0  # with the offset moved it does not lock
            return
        real(near)

    lock._engage = picky
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done" and lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12), r.error
    assert b.dlc.params["falc1:input:offset"] == 0.0 and abs(lock.dc() - 0.009) < 1e-9
    assert any("would not lock at the lockpoint" in n for n in rec.notes)
    assert any("back to +0.0000 V, as it was locked before" in n for n in rec.notes)


STEPS = {
    "map": (148.0, 60.0, 80.0, [
        "Map the modes: by way of 152.00 mA go to 148.00 mA, then sweep the current up to 152.00 mA and back "
        "down in 0.1 mA steps (81 readings) at 70.00 V, scanning the SHG cavity",
        "Park at 151."]),
    "walk": (149.0, 45.0, 95.0, [
        "Walk the piezo onto the target: +6.00 GHz away, about -15.5 V in 0.5 V steps, looking at the mode after "
        "each"]),
}


@pytest.mark.parametrize("method", ["walk", "map"])
def test_step_by_step(method):
    b, lock = shg_bench(seed=19)
    b.laser.nudge(+3.9)
    p = HopParams.from_dict({**SHG.to_dict(), "confirm": True, "method": method})
    i_lo, v_lo, v_hi, middle = STEPS[method]
    rec = Recenter(p, I0, V0, i_lo, 152.0, v_lo, v_hi, cavity=True)
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
    probes = [a for a in asked if a.startswith("See how the FALC input offset moves the error signal's DC level (+")]
    asked = [a for a in asked if a not in probes]  # (a little of the way first: how much it moves it is not known)
    steps = (["Take the laser over: turn its cavity lock off (now locked), turn the SHG cavity's lock off"] + middle
             + ["Lock the SHG cavity (single mode", "Start the scan and find the cavity peak",
                "Null the error signal's DC level (+", "Select the lockpoint at ", "Lock to the peak at "])
    assert 1 <= len(probes) <= 2 and len(asked) == len(steps), asked
    assert all(a.startswith(s) for a, s in zip(asked, steps)), asked
    if method == "map":
        assert "mA, sweeping up from 148.00 mA as the map found the mode there (" in asked[2]


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


@pytest.mark.parametrize("method", ["walk", "map"])
def test_a_wavemeter_locked_shg_laser(method):
    laser, target = hopped_laser(1, seed=22)
    b = Bench(laser, target)
    lock = b.lock_it(shg=True, falc=0, seed=22)  # an SHG stage, no cavity lock: this program locks it
    b.dlc.params["laser1:dl:lock:lock-enabled"] = False
    p = HopParams(enabled=True, i_nom=I0, shg=True, method=method, i_below=2.0)
    b, rec, r = recover(laser, target, p, bench=b)
    assert r.state == "done" and lock.shg_state()[0] == 5, r.error
    assert abs(b.off_mhz()) < 5 and b.single_mode()
    assert laser.m == (0 if method == "map" else 1)  # walked: the mode it hopped to, onto the target
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


@pytest.mark.parametrize("p", [SHG, SHG_MAP], ids=["walk", "map"])
def test_monte_carlo_over_random_shg_lasers(p):
    """Random masters, going multimode over a good part of each mode and quietly for the wavemeter,
    hopping 1 or 2 modes; a random DC level on the error signal."""
    ok = feasible = 0
    for seed in range(60):
        laser, hop, noise = random_laser(1000 + seed)
        laser.multimode, laser.mm_noise = max(laser.multimode, 0.15 * laser.fsr), 0.0
        b, lock = locked_bench(laser, seed=seed, shg=True, lockpoints=True,
                               pdh_dc=random.Random(seed).uniform(-0.03, 0.03))
        laser.nudge(hop)
        best = best_margin(laser, p)
        b, rec, r = recover(laser, b.target, p, noise=noise, cavity=True, bench=b, seed=seed)
        if r.state == "done":
            assert lock.peak == pytest.approx(b.target, abs=1e-12), seed
            assert lock.shg_state()[0] == 5 and b.second_mode() < 0.11, seed  # what an SHG sweep can see
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


def test_lock_points_are_found_only_when_asked():
    """Like a DLC pro with its lock points not shown: the scan runs, but no candidates until find-candidates; a
    relock by lockpoint asks before every look at them."""
    b, lock = shg_bench(seed=27)
    lock.jump(1)
    b.dlc.params["laser1:dl:lock:lock-enabled"] = False
    lock._lock_enabled(False)
    b.dlc.params["laser1:scan:enabled"] = True
    assert lock_points(b.dlc.params["laser1:dl:lock:candidates"]()).get("c") == []  # nothing shown
    b.dlc.execute("(exec 'laser1:dl:lock:find-candidates)")
    assert len(lock_points(b.dlc.params["laser1:dl:lock:candidates"]())["c"]) >= 2
    b.dlc.params["laser1:scan:enabled"] = False
    b.dlc.params["laser1:dl:lock:lock-enabled"] = True
    lock.settle()
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done" and lock.peak == pytest.approx(b.target, abs=1e-12), r.error
    finds = [k for k, (n, _) in enumerate(b.dlc.log) if n.endswith("find-candidates")]
    picks = [k for k, (n, _) in enumerate(b.dlc.log) if n.endswith("select-lockpoint")]
    assert finds and picks and all(any(f < k for f in finds) for k in picks)


def test_plain_relock_ticks_lock_without_lockpoint():
    """Relock by re-enabling the lock, on a DLC pro set to lock at a selected lockpoint: it would not find
    its resonance by itself, so lock without lockpoint is ticked first (and stays ticked, locked so)."""
    b, lock = shg_bench(seed=28)
    lock.jump(1)
    p = HopParams.from_dict({**SHG.to_dict(), "relock": "plain"})
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert b.writes("dl:lock:lock-without-lockpoint") == [True] and not lock.lockpoints
    assert lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12) and abs(r.result.lock_e) < 3


def test_a_plain_relock_must_see_its_lock_locked_and_its_transmission_up():
    """Relocking without a lockpoint, the wavemeter alone would take a laser parked on target, its lock on hold, for
    one relocked (in the lab's log, a relock that the next ten seconds found at 46 % of its transmission). So its
    lock must report locked, and its transmission come up as usual; here the first time it stays on hold: tried
    again, and relocked."""
    laser = ECDL(400.0, seed=30, multimode=0.9, hysteresis=0.4, mm_noise=0.004)
    b, lock = locked_bench(laser, seed=30, shg=True, lockpoints=False, pdh_dc=0.009)
    lock.jump(1)
    real, held = lock._lock_enabled, []

    def on_hold_once(on):
        real(on)
        if on and not held:
            held.append(True)
            lock.peak = None  # on hold: not locked (and parked at the target, the wavemeter cannot tell)
            lock._search_until = math.inf

    b.dlc.on_set["laser1:dl:lock:lock-enabled"] = on_hold_once
    p = HopParams.from_dict({**SHG.to_dict(), "trans_locked": 0.8})
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done" and lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12), r.error
    assert any(n.startswith("the cavity lock would not lock") for n in rec.notes) and held
    assert any(n.startswith("relocked, transmission 0.8") for n in rec.notes)


def test_plain_relock_that_fails_unticks_it_again():
    b, lock = shg_bench(seed=29)
    lock.jump(1)
    lock._nearest = lambda f: (lock.f_res + 1.5e-3, 1.0)  # this lock always goes for the next peak up
    p = HopParams.from_dict({**SHG.to_dict(), "relock": "plain"})
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "failed" and re.search(r"caught a peak \+150\d MHz away", r.error), r.error
    assert b.writes("dl:lock:lock-without-lockpoint") == [True, False] and lock.lockpoints  # as it was


# ------------------------------------------------ not lasing: its seed power, and a panic
def lasing_bench(i_stop=148.7, i_start=152.4, seed=11, **kwargs):
    """The SHG bench, its master stopping lasing below i_stop (mA) and starting again only above
    i_start; its TA switches itself off after 2 s with less than 8 mW of seed."""
    b, lock = shg_bench(seed=seed, **kwargs)
    b.laser.i_stop, b.laser.i_start = i_stop, i_start
    return b, lock


def timed_currents(b):
    """[(bench time, current)] of every current write from now on."""
    out, hook = [], b.dlc.on_set["laser1:dl:cc:current-set"]

    def logged(value):
        out.append((b.t, value))
        hook(value)

    b.dlc.on_set["laser1:dl:cc:current-set"] = logged
    return out


def test_not_lasing_jumps_the_current_up_then_brings_it_down_slowly():
    """As by hand: not lasing, the current goes up quickly, 2 mA above usual, then 3, ... until it
    lases (a few seconds without seed and its TA would switch itself off), then slowly back down."""
    b, lock = lasing_bench()
    b.laser.stop()  # the current dipped: it does not lase, and at its current will not again
    times = timed_currents(b)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert b.laser.lasing and abs(b.off_mhz()) < 1 and lock.locked() and lock.ta.on() and lock.ta.trips == 0
    currents = [i for _, i in times]
    top = currents.index(max(currents))
    assert max(currents) == pytest.approx(153.0)  # +2 mA was not enough, +3 was
    assert times[top][0] - times[0][0] <= 0.6  # up in no time (it waited half a second at +2 mA)
    down = [(t, i) for t, i in times[top:] if i >= 150.0 - 1e-9]
    assert down[-1][1] == pytest.approx(150.0) and down[-1][0] - times[top][0] >= 5.5  # 3 mA down at 0.5 mA/s
    assert rec.notes[1].startswith("not lasing (when the recovery began): seed power 0.")
    assert any(n.startswith("lasing again at 150.00 mA") for n in rec.notes)
    assert r.result.summary.startswith("relocked, stopped lasing, lasing again; walked")
    assert b.writes("dl:lock:lock-enabled")[0] is False  # its cavity lock off first: it would fight the jump


def test_a_ta_that_switched_itself_off_is_left_to_you():
    b, lock = lasing_bench()
    b.dlc.params["laser1:amp:seed-limits:power-min-shutdown-delay"] = 0.3  # too quick to save
    b.laser.stop()
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "failed" and not r.restored and "TA switched itself off" in r.error
    assert b.laser.lasing and b.I == pytest.approx(I0) and lock.ta.trips == 1  # lasing, where it should


def test_not_lasing_even_at_the_top_puts_the_current_back_slowly():
    b, lock = lasing_bench(i_start=156.0)  # it would need 6 mA more
    b.laser.stop()
    times = timed_currents(b)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "failed" and r.restored and r.error.startswith("not lasing even at 155.00 mA"), r.error
    assert max(i for _, i in times) == pytest.approx(155.0) and b.I == pytest.approx(I0)
    top = max(range(len(times)), key=lambda k: times[k][1])
    assert times[-1][0] - times[top][0] >= 0.4 * 24  # back down slowly, 0.2 mA every 0.4 s


def test_it_stops_lasing_on_the_way_and_is_never_let_that_low_again():
    """Sweeping the current down for a nearer mode, it stops lasing: the current up at once, lasing again,
    and never let that low again; the piezo walks the mode it is on onto the target instead."""
    laser = ECDL(400.0, seed=33, multimode=0.9, hysteresis=0.4, mm_noise=0.004, ksv=0.4)
    b, lock = locked_bench(laser, seed=33, shg=True, lockpoints=True, pdh_dc=0.009)
    laser.i_stop, laser.i_start = 149.5, 151.0
    laser.nudge(-4.5)  # one mode down, 6 GHz: further than reach, and the current down would bring one nearer
    p = HopParams.from_dict({**SHG.to_dict(), "reach": 5.0})
    times = timed_currents(b)
    b, rec, r = recover(laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert b.laser.lasing and lock.locked() and abs(b.off_mhz()) < 1 and rec.panics == 1
    floor = rec.i_lo  # where it stopped (asked after every step down), plus half a mA
    assert 149.5 + 0.5 - 0.1 <= floor <= 149.5 + 0.5
    currents = [i for _, i in times]
    top = currents.index(max(currents))  # the jump up: from then on, never below
    assert currents[top] >= 151.0 and min(currents[top:]) >= floor - 1e-9
    assert any(n.startswith("not lasing (at 149.") and "looking for a mode within 5 GHz" in n for n in rec.notes)
    assert any(n.startswith("no mode nearer: walking the piezo -6.0") for n in rec.notes), rec.notes


def test_a_map_that_stops_it_lasing_maps_again_above():
    b, lock = lasing_bench(i_stop=148.7, i_start=151.0)
    b.laser.nudge(+3.9)
    b, rec, r = recover(b.laser, b.target, SHG_MAP, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert rec.panics == 1 and rec.i_lo == pytest.approx(148.6 + 0.5) and b.laser.lasing and lock.locked()
    assert min(pt[0] for pt in rec.record["maps"][-1]["points"]) >= 149.1 - 1e-9  # the next map, above


def test_cancelled_while_not_lasing_it_goes_back_by_way_of_the_top():
    b, lock = lasing_bench()
    b.laser.stop()
    p = HopParams.from_dict(SHG.to_dict())
    rec = Recenter(p, I0, V0, 149.0, 152.0, 45.0, 95.0, cavity=True, i_top=155.0)
    r = b.run(rec, until=lambda r: rec.I > 151.0)  # jumping up
    r.cancel("stopped")
    while r.busy:
        if b.pending:
            b.pending.pop(0)()
        else:
            r.tick(b.reading()[0])
    currents = b.writes("dl:cc:current-set")
    assert max(currents) == pytest.approx(155.0) and b.I == pytest.approx(I0) and r.restored
    assert b.laser.lasing  # by way of the top: lasing again, then down to where it was


def test_no_seed_power_no_watching():
    laser, target = hopped_laser(1)
    b, rec, r = recover(laser, target)  # a DL pro without a TA: nothing to read
    assert r.state == "done" and rec.seed_name == "" and rec.lasing is None
    b, lock = lasing_bench()
    b.laser.nudge(+3.9)
    p = HopParams.from_dict({**SHG.to_dict(), "seed": "off"})
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done" and rec.seed_name == "" and not any("seed" in n for n, _ in b.dlc.log)


def test_its_emission_off_there_is_nothing_to_do():
    b, lock = lasing_bench()
    b.dlc.params["laser1:emission"] = False
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "failed" and r.error == "its emission is off" and r.restored
    assert b.writes("dl:cc:current-set") == [] and lock.locked()  # nothing touched


# ----------------------------------------------------------------- the walk
def test_the_walk_trims_the_current_when_the_mode_gets_worse():
    """With a piezo that moves the grating more than the mode (and no feedforward), walking far takes
    the mode towards where it goes multimode: as by hand, the current is trimmed on the way."""
    laser = ECDL(400.0, seed=31, multimode=0.9, hysteresis=0.4, mm_noise=0.004, ksv=0.5)
    b, lock = locked_bench(laser, seed=31, shg=True, lockpoints=True, pdh_dc=0.009)
    laser.nudge(-3.9)
    b, rec, r = recover(laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert rec.trims >= 3 and any(n.startswith("the mode as it was at ") for n in rec.notes), rec.notes
    assert rec.record["walk"] and b.second_mode() < 0.1 and lock.locked()  # (as good as it was, give or take)


def test_the_walk_hops_to_a_nearer_mode_first():
    """Two modes away (12 GHz), beyond 8 GHz: the current up until a nearer mode lases, then the walk
    (its piezo moving the grating with the mode, as with the right feedforward)."""
    laser = ECDL(400.0, seed=32, multimode=0.9, hysteresis=0.4, mm_noise=0.004, ksv=0.4)
    b, lock = locked_bench(laser, seed=32, shg=True, lockpoints=True, pdh_dc=0.009)
    laser.nudge(2 * 6.0 - 1.0)  # (sweeping the current up from here, it hops one mode back)
    assert b.off_mhz() == pytest.approx(12000, abs=600)
    b, rec, r = recover(laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert any(n.startswith("hopped at 15") and "GHz from the target" in n for n in rec.notes), rec.notes
    assert lock.locked() and abs(b.off_mhz()) < 1 and b.single_mode()


def test_a_laser_that_is_fine_is_relocked_and_nothing_else():
    """Relocking a laser that is fine (on target, single-mode: its lock was turned off, say): it is looked at,
    and the locks relocked, step by step as asked; the current is never moved (not even to look either side
    of it), nor the piezo to measure its tuning, known from before."""
    b, lock = shg_bench(seed=22)
    p = HopParams.from_dict({**SHG.to_dict(), "kv": 400.0, "confirm": True})
    rec = Recenter(p, I0, V0, 149.0, 152.0, 45.0, 95.0, cavity=True, i_top=155.0)
    runner, asked = Runner(rec, b, b.t), []
    while runner.busy:
        if b.pending:
            b.pending.pop(0)()
        elif runner.asking:
            asked.append(runner.asking)
            runner.confirm()
        else:
            runner.on_reading(*b.reading())
    assert runner.state == "done", runner.error
    assert b.writes("dl:cc:current-set") == [] and b.writes("scan:offset") == []
    asked = [a for a in asked if not a.startswith("See how the FALC input offset moves")]  # (first, a little of it)
    steps = ["Take the laser over", "Lock the SHG cavity (single mode", "Start the scan", "Null the error signal",
             "Select the lockpoint", "Lock to the peak"]
    assert len(asked) == len(steps) and all(a.startswith(x) for a, x in zip(asked, steps)), asked
    assert "the laser itself was fine" in runner.result.summary and lock.locked() and b.single_mode()
    learned = runner.result.learned
    assert learned["trans_locked"] == pytest.approx(0.8, abs=0.02)  # its transmission, locked as it should...
    assert learned["trans_zero"] == pytest.approx(0.0, abs=0.003)  # ...and between the peaks, scanning
    assert learned["shg_lock"] == pytest.approx(b.shg_usual, abs=0.02)


def test_the_transmission_zero_is_measured_while_the_master_scans():
    """Its transmission on Fine In 1 as on a real one: 0.05 V locked on a DC offset of 0.01 V. While the master scans
    for the peak, the level between the peaks is its zero; locked, the median of a few looks its level locked (the
    engine watches it against both); and the relock's check, locked as high as the peak above zero, holds."""
    b, lock = shg_bench(seed=22, t00=0.05, t_dc=0.01)
    p = HopParams.from_dict({**SHG.to_dict(), "kv": 400.0})
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    learned = r.result.learned
    assert learned["trans_zero"] == pytest.approx(0.01, abs=0.002)
    assert learned["trans_locked"] == pytest.approx(0.06, abs=0.003)
    assert learned["peak_y"] == pytest.approx(0.06, abs=0.006)
    notes = " | ".join(text for _, text in r.result.record["notes"])
    assert re.search(r"transmission off resonance, scanning: 0\.0[01]\d\d V \(its zero\)", notes), notes
    assert re.search(r"locked to the right peak, transmission 0\.0[56]\d\d V", notes), notes


@pytest.mark.parametrize("ease", [0.001, 0.0], ids=["eased", "trimmed"])
def test_two_modes_alike_are_found_out_by_the_shg_lock(ease):
    """Two modes of nearly the same power look like one clean mode in an SHG sweep, if nothing is known yet of
    how this laser looks single-mode; but its SHG cavity then locks at little more than half its usual signal:
    the master is at the top of its mode. The current is eased down off the top, its SHG signal locked (a little
    at a time while it rises); without that (ease_step 0), trimmed to where the sweep's main peaks are biggest."""
    b, lock = shg_bench(seed=21)
    laser = b.laser
    a = laser.misalignment(laser.m, b.V + lock.dv, b.act)
    edge, full = laser.zone(1)
    laser.nudge(a - (edge + 0.78 * (full - edge)))  # 39 % of its power in the next mode, without a hop
    assert 0.38 < b.second_mode() < 0.4
    p = HopParams.from_dict({**SHG.to_dict(), "ease_step": ease})
    b, rec, r = recover(laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert b.second_mode() < 0.05 and lock.locked() and lock.shg_state()[0] == 5
    if ease:
        assert any(n.startswith("off the top of its mode: its SHG signal dipping (0.76") for n in rec.notes), rec.notes
        ease = r.result.record["ease"]
        assert ease["to"] < ease["from"] - 0.3 and ease["signal"][1] > 1.15
        return
    assert any(n.startswith("the SHG cavity locked, but its intra-cavity signal is ")
               and n.endswith(" of usual: the master is not single-mode") for n in rec.notes), rec.notes
    assert any(n.startswith("the SHG sweep's main peaks ") for n in rec.notes), rec.notes


def test_a_piezo_tuning_learned_wrong_is_measured_again():
    """The piezo tuning it knows has the wrong sign (the piezo was rewired, say): the first step of the walk
    takes the laser further from the target; its tuning is measured, and the walk goes the right way."""
    b, lock = shg_bench(seed=23)
    b.target += 1.5e-3  # the next cavity peak up: 1.5 GHz to walk
    p = HopParams.from_dict({**SHG.to_dict(), "kv": -400.0})
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert any(n.startswith("the piezo tuned it +") and n.endswith("not -400: measured again") for n in rec.notes)
    assert r.result.kv == pytest.approx(400, rel=0.1) and abs(b.off_mhz()) < 1 and lock.locked()


def test_not_lasing_is_seen_by_the_shg_sweep_when_its_seed_cannot_tell():
    """Its master stops lasing on the way (sweeping the current down for a nearer mode), but still sends
    the TA 70 % of its power, broadband: the seed photodiode cannot tell. The wavemeter loses it, and the SHG
    sweep flat-lines: up at once until the sweep shows it again, then slowly down, never that low again."""
    laser = ECDL(400.0, seed=33, multimode=0.9, hysteresis=0.4, mm_noise=0.004, ksv=0.4)
    b, lock = locked_bench(laser, seed=33, shg=True, lockpoints=True, pdh_dc=0.009)
    laser.i_stop, laser.i_start, laser.ase = 149.5, 151.0, 0.7
    laser.nudge(-4.5)
    p = HopParams.from_dict({**SHG.to_dict(), "reach": 5.0})
    b, rec, r = recover(laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert laser.lasing and lock.locked() and abs(b.off_mhz()) < 1 and rec.panics == 1 and lock.ta.trips == 0
    assert any(n.startswith("not lasing (at 149.") and "seed power 13." in n for n in rec.notes)  # (above 8 mW)
    assert any(n.endswith("the SHG sweep shows it again") for n in rec.notes) and 149.8 <= rec.i_lo <= 150.0


def test_not_lasing_from_the_start_is_seen_by_the_shg_sweep():
    """Not lasing when the recovery begins, its seed power fooled: the first SHG sweep is flat. The current
    jumps up until the sweep shows resonances (+4 mA), and back down to nominal: where it stopped is not known,
    so nothing is put out of bounds."""
    b, lock = lasing_bench(i_start=153.0)
    b.laser.ase = 0.7
    b.laser.stop()
    times = timed_currents(b)
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert b.laser.lasing and lock.locked() and abs(b.off_mhz()) < 1 and rec.panics == 1
    assert max(i for _, i in times) == pytest.approx(154.0) and rec.i_lo == pytest.approx(149.0)
    assert rec.notes[2].startswith("at 150.00 mA the SHG sweep is flat: it does not lase"), rec.notes


def test_a_flat_shg_sweep_with_its_ta_off_is_left_to_you():
    b, lock = shg_bench(seed=30)
    lock.jump(1)
    b.dlc.params["laser1:amp:cc:enabled"] = False  # no light reaches the SHG cavity
    b, rec, r = recover(b.laser, b.target, SHG, cavity=True, bench=b)
    assert r.state == "failed" and r.error.startswith("its TA is off (no light reaches the SHG cavity)"), r.error
    assert rec.panics == 0 and b.writes("dl:cc:current-set") == []  # the current never jumped for it


def test_monte_carlo_walks_with_lasers_that_stop_lasing():
    """Random SHG masters that stop lasing 0.7 to 2 mA below their current and start again 1.5 to 4.5
    mA above: whatever happens, never left without lasing; mostly recovered."""
    done = 0
    for seed in range(40):
        rng = random.Random(500 + seed)
        laser, hop, noise = random_laser(2000 + seed)
        laser.multimode, laser.mm_noise = max(laser.multimode, 0.15 * laser.fsr), 0.0
        laser.i_stop, laser.i_start = I0 - rng.uniform(0.7, 2.0), I0 + rng.uniform(1.5, 4.5)
        b, lock = locked_bench(laser, seed=seed, shg=True, lockpoints=True, pdh_dc=rng.uniform(-0.03, 0.03))
        laser.nudge(hop * rng.choice([0.5, 1.0]))
        if rng.random() < 0.3:
            laser.stop()
        b, rec, r = recover(laser, b.target, SHG, noise=noise, cavity=True, bench=b, seed=seed)
        laser.lase(b.V + lock.dv, b.act)
        assert laser.lasing or not laser.lasing and r.state == "failed" and "even at" in r.error, seed
        if r.state == "done":
            done += 1
            assert lock.locked() and abs(b.off_mhz()) < 1 and b.second_mode() < 0.16, seed  # (what the SHG
            assert lock.ta.on(), seed                                                        # sweep can see)
    assert done >= 30, done


# ------------------------------------------------------------ feedforward
def ff_bench(ff=-0.15, seed=12, i_stop=None, i_start=None):
    """The SHG bench, its DLC pro's feedforward on: the piezo moves the current, ff mA/V (as measured, which is the
    set current with the piezo where it begins); its master stopping lasing below i_stop and starting again only
    above i_start (measured)."""
    laser = ECDL(400.0, seed=seed, multimode=0.9, hysteresis=0.4, mm_noise=0.004, i_stop=i_stop, i_start=i_start)
    b = Bench(laser, laser.freq(V0, I0), seed=seed, scan=True)
    d = b.dlc.params
    d["laser1:dl:cc:feedforward-enabled"], d["laser1:dl:cc:feedforward-factor"] = True, ff
    b.dlc.bias[1] = -ff * V0
    lock = b.lock_it(seed=seed, shg=True, lockpoints=True, pdh_dc=0.009)
    lock.settle()
    assert lock.locked() and abs(b.off_mhz()) < 1 and b.act == pytest.approx(I0)
    b.shg_usual = lock._intra_now()
    return b, lock


def test_the_feedforward_does_not_take_the_current_out_of_its_window_or_below_where_it_lases():
    """The night it went wrong: walking the piezo 16 V onto the target after a hop, the feedforward (-0.15 mA/V)
    takes the current 2.4 mA down, through where the laser stops lasing (149.8 mA, nominal 150). The window and the
    floor are on the measured current, so the current set follows the piezo; and as by hand, the current is parked
    high before the walk (as high as the SHG sweep stays as good), and again whenever it has come down 0.3 mA: it
    never stops lasing, and is relocked on target."""
    b, lock = ff_bench(-0.15, i_stop=149.8, i_start=152.5)
    b.laser.nudge(-3.9)  # it hops: -6 GHz, 16 V of piezo away
    b, rec, r = recover(b.laser, b.target, HopParams.from_dict({**SHG.to_dict(), "kv": 400.0}), cavity=True, bench=b)
    assert r.state == "done", r.error
    assert rec.panics == 0 and b.laser.lasing and lock.locked() and abs(b.off_mhz()) < 1 and b.single_mode()
    assert rec.ff == -0.15 and rec.notes[0].startswith("feedforward -0.1500 mA/V: the piezo moves the current too")
    acts = measured_currents(b)
    assert min(acts) >= 149.8 and max(acts) <= 152.0 + 1e-9  # (check_safety: and never a jump)
    assert b.I > 153.0 and b.act <= 152.0  # set high, for the measured current to stay put
    parks = [n for n in rec.notes if n.startswith("parked the current high")]
    assert len(parks) >= 3, rec.notes
    assert re.match(r"relocked, walked the piezo 70\.00 → 8[56]\.\d\d V", r.result.summary), r.result.summary


def test_the_floor_where_it_stopped_lasing_is_on_the_measured_current():
    """Stopped lasing at 149.8 mA measured with the piezo moved 10 V, the feedforward -0.15 mA/V: the floor from then
    on (150.3) is a measured current, a set current 1.5 mA higher there, 1.5 mA lower with the piezo back."""
    rec = Recenter(HopParams(i_nom=I0), I0, V0, 149.0, 152.0, 45.0, 95.0, i_top=155.0)
    rec.ff, rec.V = -0.15, V0 + 10.0
    assert rec.window() == pytest.approx((150.5, 153.5, 156.5)) and rec.mA(151.5) == pytest.approx(150.0)
    rec.raise_floor(151.3 + rec.STOP_MARGIN)  # (it stopped at 151.3 set: 149.8 measured)
    assert rec.i_lo == pytest.approx(151.8) and rec.mA(rec.i_lo) == pytest.approx(150.3)
    assert rec.window(V0)[0] == pytest.approx(150.3) and rec.nominal == pytest.approx(151.5)
    assert rec.allowed(V0 + 10.0) == pytest.approx((151.5, 156.5))  # (as far down as where it began, 150 at V0)


def test_a_current_left_above_its_window_is_brought_down_into_it():
    """The morning after: left lasing 3.5 mA above nominal, on the next mode and multimode, off target. A recovery
    starts from there (it used to refuse: outside the window): down into the window slowly, asking whether it
    lases, then on down as by hand until the SHG sweep is single-mode, into the mode, and walked onto the target."""
    b, lock = ff_bench(-0.15, i_stop=149.6, i_start=152.5)
    b.dlc.execute("(param-set! 'laser1:dl:lock:lock-enabled #f)")
    for k in range(1, 18):  # (by hand, as a panic would have)
        b.dlc.execute(f"(param-set! 'laser1:dl:cc:current-set {I0 + 0.2 * k:.3f})")
    b.dlc.execute(f"(param-set! 'laser1:dl:cc:current-set {I0 + 3.5:.3f})")
    b.laser.lase(b.V, b.act)
    assert abs(b.off_mhz()) > 6000
    b.start, b.dlc.log = (b.I, b.V), []
    p = HopParams.from_dict({**SHG.to_dict(), "kv": 400.0, "shg_lock": b.shg_usual})
    rec = Recenter(p, b.I, b.V, I0 - 1.0, I0 + 2.0, V0 - 25.0, V0 + 25.0, cavity=True, i_top=I0 + 5.0)
    r = b.run(rec)
    assert r.state == "done", r.error
    assert lock.locked() and abs(b.off_mhz()) < 1 and b.single_mode() and b.act <= I0 + 2.0
    assert re.match(r"the current was 153\.50 mA, above its window \(149\.00 … 152\.00 mA\): down into it, and on "
                    r"down to 15[01]\.\d0 mA: single mode", next(n for n in rec.notes if "above its window" in n))
    check_safety(b, I0 - 1.0, I0 + 3.5, p)  # (within the window, or between it and where it began; never a jump)


def test_a_current_left_below_its_window_is_brought_up_into_it():
    laser, target = hopped_laser(1)
    b = Bench(laser, target, current=147.5)
    laser.lase(b.V, b.act)
    rec = Recenter(HopParams(enabled=True, i_nom=I0, i_below=2.0), 147.5, V0, 148.0, 152.0, 60.0, 80.0)
    r = b.run(rec)
    assert r.state == "done", r.error
    assert rec.notes[0] == "the current was 147.50 mA, below its window (148.00 … 152.00 mA): up into it"
    check_safety(b, 147.5, 152.0, rec.p)


def test_the_runner_keeps_the_measured_current_in_step_with_the_feedforward():
    """Whatever the procedure asks, with the feedforward on: a current outside both the window and where it began is
    refused, and an offset move is made in steps that move the measured current 0.2 mA at most; on failure, current
    and offset go back together, in steps that do neither."""
    class Rogue(Recenter):
        def run(self):
            self.ff = -0.15
            yield Read()
            yield Move(offset=80.0)  # 1.5 mA of feedforward: in steps
            yield Move(current=150.6)
            yield Read()
            raise Failed("gave up")

    b, lock = ff_bench(-0.15)
    lock.dlc.execute("(param-set! 'laser1:dl:lock:lock-enabled #f)")
    r = b.run(Rogue(HopParams(), I0, V0, 148.0, 152.0, 60.0, 90.0))
    assert r.state == "failed" and r.error == "gave up" and r.restored and (b.I, b.V) == (I0, V0)
    offsets = [V0] + b.writes("scan:offset")
    assert all(abs(x - y) <= STEP_MAX / 0.15 + 1e-9 for x, y in zip(offsets, offsets[1:])) and 80.0 in offsets
    acts = measured_currents(b)
    assert all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip(acts, acts[1:]))

    class Greedy(Rogue):
        def run(self):
            self.ff = -0.15
            yield Read()
            self.I = 152.5  # (a bug: past the window, the set current moved by hand)
            yield Move(current=152.6)

    b, lock = ff_bench(-0.15)
    r = b.run(Greedy(HopParams(), I0, V0, 148.0, 152.0, 60.0, 90.0))
    assert max(b.writes("dl:cc:current-set")) <= 152.0 + 1e-9  # (move() keeps it inside: the Runner, at the edge)


def test_cancelled_mid_walk_with_the_feedforward_everything_goes_back_in_small_steps():
    b, lock = ff_bench(-0.15, i_stop=149.8, i_start=152.5)
    b.laser.nudge(-3.9)
    p = HopParams.from_dict({**SHG.to_dict(), "kv": 400.0, "shg_lock": b.shg_usual})
    rec = Recenter(p, I0, V0, I0 - 1.0, I0 + 2.0, V0 - 25.0, V0 + 25.0, cavity=True, i_top=I0 + 5.0)
    r = b.run(rec, until=lambda run: rec.V > V0 + 8.0)
    assert r.busy and b.V > V0 + 7.0 and b.I > I0 + 1.0
    r.cancel("stopped by the user")
    while r.busy:
        if b.pending:
            b.pending.pop(0)()
        else:
            r.on_reading(*b.reading())
    assert r.state == "failed" and r.restored and (b.I, b.V) == (I0, V0) and b.laser.lasing
    acts = measured_currents(b)
    assert min(acts) >= 149.8 and all(abs(x - y) <= STEP_MAX + 1e-9 for x, y in zip(acts, acts[1:]))


# ------------------------------------------------- the catch: back onto the right mode
def near_bench(seed=40, ff=0.0, i_stop=None, i_start=None, collapse=False, **ecdl):
    """A TA-SHG pro like the 739 nm one: its master's modes 2.2 GHz (4 pm) apart, going multimode near their edges,
    locked by hand to the cavity peak at its target (as is its SHG cavity); optionally its feedforward on (ff mA/V,
    as ff_bench), stopping lasing below i_stop, starting again above i_start, and its light broadband where a second
    mode begins (collapse: SimLocked)."""
    kw = dict(fsr=2.2, hysteresis=0.3, multimode=0.4, mm_noise=0.004)
    kw.update(ecdl)
    laser = ECDL(400.0, seed=seed, i_stop=i_stop, i_start=i_start, **kw)
    b = Bench(laser, laser.freq(V0, I0), seed=seed, scan=True)
    if ff:
        d = b.dlc.params
        d["laser1:dl:cc:feedforward-enabled"], d["laser1:dl:cc:feedforward-factor"] = True, ff
        b.dlc.bias[1] = -ff * V0
    lock = b.lock_it(seed=seed, shg=True, lockpoints=True, pdh_dc=0.009, collapse=collapse)
    lock.settle()
    assert lock.locked() and abs(b.off_mhz()) < 1 and b.act == pytest.approx(I0)
    b.shg_usual = lock._intra_now()
    return b, lock


def optimum(b, **kwargs) -> HopParams:
    """The catch (the default with an SHG stage), the optimum taken where the bench began, as clicking Relock on it
    would have."""
    return HopParams.from_dict({**SHG_CATCH.to_dict(), "opt_i": I0, "opt_v": V0, "opt_shg": b.shg_usual,
                                "opt_t": "2026-10-19 21:03", **kwargs})


def hop_up(b, lock, by=1.6):
    """Its selection filter drifts: it hops to the next mode up in frequency (739.053 → 739.049 nm), and its lock
    catches another peak."""
    b.laser.nudge(+by)
    lock.frequency()
    assert b.laser.m == 1


def test_after_a_hop_to_the_near_neighbour_it_is_caught_back_on_the_right_mode():
    """The night it failed: its modes 2.2 GHz apart, it hops to the next one up. Walked, the piezo takes that mode onto
    the target, 5.5 V from where it was. Caught, as by hand: the current up, then down a step at a time until it is
    on the right mode, its SHG peak within 10 % of the optimum's; on into the mode's best and back up to the top end
    of it, the piezo onto the target; the SHG cavity locked, the error signal's DC nulled, and locked."""
    b, lock = near_bench()
    hop_up(b, lock)
    b, rec, r = recover(b.laser, b.target, optimum(b, method="walk"), cavity=True, bench=b)
    assert r.state == "done" and b.laser.m == 1 and abs(b.V - V0) == pytest.approx(5.5, abs=0.3), r.error

    b, lock = near_bench()
    hop_up(b, lock)
    b, rec, r = recover(b.laser, b.target, optimum(b, reset_up=0.0), cavity=True, bench=b)  # (the goes alone)
    assert r.state == "done", r.error
    assert b.laser.m == 0 and abs(b.V - V0) < 1.5 and b.I > I0  # the right mode, the piezo about where it was
    assert lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12) and b.single_mode()
    assert lock.shg_state() == (5, "locked") and abs(lock.dc()) < 0.002
    currents = b.writes("dl:cc:current-set")
    top = currents.index(max(currents))
    assert max(currents) == pytest.approx(I0 + 2.0) and currents[:top] == sorted(currents[:top])  # up quickly...
    down = currents[top:currents.index(min(currents[top:]))]
    assert all(0.09 < x - y < 0.11 for x, y in zip(down, down[1:]))  # ...then down a step at a time
    go = r.result.record["catch"][0]
    assert go["caught"] and go["right"] == 0 and go["points"][-1][2] >= 0.9
    assert re.match(r"relocked, caught the right mode at 151\.\d0 mA \(1 go up and down\), walked the piezo 70\.00 → "
                    r"70\.\d\d V, .*SHG peak (0\.9\d|1\.0\d) of the optimum's", r.result.summary), r.result.summary
    order = [n.replace("laser1:", "") for n, _ in b.dlc.log if "cc:current" not in n and "scan:offset" not in n]
    falc = order.count("falc1:input:offset")  # (a little of the way first: how much the offset moves it)
    assert 2 <= falc <= 4 and order == [
        "dl:lock:lock-enabled", "nlo:shg:lock:lock-enabled", "scan:enabled", "nlo:shg:scan:enabled",
        "nlo:shg:scan:enabled", "nlo:shg:lock:lock-enabled", "scan:enabled", "dl:lock:find-candidates"] + [
        "falc1:input:offset"] * falc + ["dl:lock:find-candidates", "dl:lock:select-lockpoint", "dl:lock:lock-enabled"]
    assert settings(b) == ORIGINAL
    learned = r.result.learned
    assert learned["shg_ratio"] == pytest.approx(1.0, abs=0.05)  # its sweep shows the peak as its signal locked
    assert r.result.kv == pytest.approx(400, rel=0.05)


def test_not_caught_where_it_is_it_is_caught_from_the_optimum_piezo_voltage_and_walked_back():
    """Hysteresis can keep it off the right mode wherever the current goes in its window: it hops back only 1.14 mA
    up, the window ends 0.95 mA up. Three goes, then from the optimum's piezo voltage + 30 V, where it is caught (12
    GHz from the target there, as the piezo tuning says); walked back 30 V, the current parked high in the mode now
    and then on the way (the piezo takes the mode away from it), and it is on target, on the right mode."""
    b, lock = near_bench(hysteresis=0.8, multimode=0.3, ksv=0.32)
    hop_up(b, lock, 2.3)
    b, rec, r = recover(b.laser, b.target, optimum(b, i_window=0.95, v_window=40.0, kv=400.0), cavity=True, bench=b)
    assert r.state == "done", r.error
    assert b.laser.m == 0 and abs(b.off_mhz()) < 1 and lock.locked() and b.single_mode()
    goes = r.result.record["catch"]
    assert [g["caught"] for g in goes] == [False, False, False, True] and goes[3]["v"] == pytest.approx(V0 + 30)
    assert goes[3]["right"] == pytest.approx(12000, rel=0.05)
    assert "not caught in 3 goes at 70.00 V: piezo to 100.00 V (the optimum's 70.00 V +30 V)" in rec.notes
    assert sum(n.startswith("parked the current high") for n in rec.notes) >= 2
    assert re.match(r"relocked, caught the right mode at 150\.\d\d mA, 100\.00 V \(4 goes up and down\), walked the "
                    r"piezo 70\.00 → 70\.\d\d V", r.result.summary), r.result.summary


def below_second_mode(b, lock) -> float:
    """mA from where it runs up to where a second mode starts to lase with it (negative: it has)."""
    las = b.laser
    v = b.V + lock.dv
    las.lase(v, b.act)
    return (las.zone(1.0)[0] - las.misalignment(las.m, v, b.act)) / (las.ki - las.ksi)


def test_a_laser_on_its_mode_has_its_current_parked_at_the_top_end_of_its_best_all_the_same():
    """Its lock caught the next cavity peak, 1.5 GHz off: the laser is on its mode, its SHG peak as good as it should
    be. Its current goes to the top end of where the mode is at its best all the same, as by hand (up a step at a time
    while its SHG peak stays at its best, then two back), then the piezo onto the target, and it is relocked."""
    b, lock = near_bench(seed=41)
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0), cavity=True, bench=b)
    assert r.state == "done", r.error
    currents = b.writes("dl:cc:current-set")
    top = currents.index(max(currents))
    assert currents[:top + 1] == pytest.approx([I0 + 0.1 * k for k in range(1, top + 2)])  # up a step at a time...
    assert currents[top + 1:] == pytest.approx([currents[top] - 0.1, currents[top] - 0.2])  # ...then two back
    assert b.I > I0 + 0.2 and 0 <= below_second_mode(b, lock) < 0.25 and b.single_mode()
    assert lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12)
    assert any(re.match(r"on the right mode, its SHG peak (0\.9\d|1\.0\d) of the optimum's: the current to the top "
                        r"end of where it is at its best, then the piezo onto the target", n) for n in rec.notes)
    assert re.match(r"relocked, on the right mode: the current to the top end of where it is at its best, 150\.00 → "
                    r"150\.\d0 mA measured, the piezo 70\.00 → 70\.\d\d V", r.result.summary), r.result.summary


def test_a_flat_shg_sweep_the_wavemeter_reads_through_is_the_top_of_its_mode():
    """Sirius, 27 October: parking its current high, its SHG sweep went flat where a second mode began (its light
    broadband: coherence collapse at the top of its mode) while the wavemeter still read it and its seed power stayed
    up. Taken for a laser that does not lase, its current jumped 3.5 mA up, and every recovery after failed from there.
    A flat sweep the wavemeter reads through is the top of its mode: the current back off it, and it is relocked."""
    b, lock = near_bench(seed=41, collapse=True)
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0), cavity=True, bench=b)
    assert r.state == "done", (r.error, rec.notes)
    assert rec.panics == 0 and max(b.writes("dl:cc:current-set")) < I0 + 1.0, rec.notes
    assert lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12) and b.single_mode()
    assert 0 <= below_second_mode(b, lock) < 0.25
    assert any(n.startswith("parked the current high") for n in rec.notes), rec.notes


def test_a_relock_brings_a_current_the_feedforward_sank_back_up_to_the_top_end_of_its_best():
    """The 739 nm TA-SHG of 23 October: relock after relock its piezo drifted up, and its feedforward took the current
    down with it, to near the bottom of its mode (there its cavity lock would not hold, and its SHG looked worse), but
    still on the right mode, its SHG peak within 10 % of the optimum's: each relock moved nothing but the piezo. Now the
    current goes back up to the top end of where the mode is at its best first; relock after relock it stays there."""
    b, lock = near_bench(seed=47, ff=-0.145)
    b.laser.nudge(+0.6)  # (0.06 mA above where a second mode starts below it, 0.74 mA below the one above)
    assert 0.7 < below_second_mode(b, lock) < 0.8 and b.single_mode()
    p = optimum(b, kv=400.0)
    for k in range(3):
        lock.jump(1 if k % 2 == 0 else -1)
        b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b, seed=k)
        assert r.state == "done", r.error
        assert b.laser.m == 0 and lock.locked() and abs(b.off_mhz()) < 1 and b.single_mode()
        assert 0 <= below_second_mode(b, lock) < 0.25 and b.act > I0 + 0.45, (k, b.act)
    assert sum(n.startswith("parked the current high") for n in rec.notes) <= 1


def test_step_by_step_the_catch_asks_before_it_parks_the_current_high():
    b, lock = near_bench(seed=41)
    lock.jump(1)
    p = optimum(b, kv=400.0, confirm=True)
    rec = Recenter(p, b.I, b.V, I0 - 1.0, I0 + 2.0, b.V - 25.0, b.V + 25.0, cavity=True, i_top=I0 + 5.0)
    runner, asked = Runner(rec, b, b.t), []
    while runner.busy:
        if b.pending:
            b.pending.pop(0)()
        elif runner.asking:
            asked.append(runner.asking)
            runner.confirm()
        else:
            runner.on_reading(*b.reading())
    assert runner.state == "done", runner.error
    assert asked[0].startswith("Take the laser over") and asked[1] == (
        "On the right mode: the current up a step at a time while its SHG peak stays at its best, then back two steps "
        "(the top end of where the mode is at its best), then the piezo onto the target")
    assert any(a.startswith("Lock the SHG cavity") for a in asked[2:]), asked


def test_parking_high_looks_twice_at_a_peak_that_falls_and_never_lowers_the_current():
    """A sweep reads the SHG peak to a few per cent only (±7 % on the 739 nm one, sweep to sweep): one that reads low
    must not stop it short of the top end of the mode's best (it is looked at again first, and compared with the
    middle of what it read on the way, not the best, which noise puts too high); and a relock never leaves the current
    lower than it found it, or noise would walk it down relock after relock. From near the bottom of its mode, and
    from near the top."""
    ends = []
    for seed in range(40, 52):
        for nudge in (+0.6, -0.5):
            b, lock = near_bench(seed=seed)
            lock.shg_jitter = 0.06
            b.laser.nudge(nudge)
            lock.jump(1)
            b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0), cavity=True, bench=b, seed=seed)
            assert r.state == "done" and lock.locked() and b.laser.m == 0, (seed, nudge, r.error)
            assert b.act >= I0 - 1e-9, (seed, nudge, b.act)
            ends.append(below_second_mode(b, lock))
    assert sum(0 <= x < 0.25 for x in ends) >= 0.9 * len(ends), ends


def test_its_right_mode_beyond_the_window_the_mode_it_is_on_is_walked_onto_the_target():
    """Modes 6 GHz apart, the window 0.8 mA up: the right one comes back only 0.86 mA up (with 2 mA it is caught).
    Not caught, the laser goes back where it began and the mode it is on is walked onto the target instead,
    as the walk would: locked on another mode rather than not at all."""
    b, lock = shg_bench(seed=11)
    b.laser.nudge(+3.9)
    p = HopParams.from_dict({**SHG_CATCH.to_dict(), "i_window": 0.8})
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert r.result.summary.startswith("relocked, the right mode not caught, walked the mode it was on instead: "
                                       "walked the piezo 70.00 → "), r.result.summary
    assert lock.locked() and lock.peak == pytest.approx(b.target, abs=1e-12) and b.single_mode()
    assert re.search(r"going up and sweeping down 3 times,? in 149\.00 … 150\.80 mA", rec.fell_back), rec.fell_back

    b, lock = shg_bench(seed=11)
    b.laser.nudge(+3.9)
    b, rec, r = recover(b.laser, b.target, SHG_CATCH, cavity=True, bench=b)
    assert r.state == "done" and not rec.fell_back and b.laser.m == 0 and lock.locked(), r.error


def test_an_optimum_it_cannot_come_back_to_says_so():
    """The optimum taken with more SHG power than there is now (its TA weaker since, say): the right mode's peak
    never comes within 10 % of it. The message says so, and that the optimum may want taking again; with 30 %
    allowed, it is caught."""
    b, lock = near_bench(seed=43)
    hop_up(b, lock)
    b, rec, r = recover(b.laser, b.target, optimum(b, opt_shg=1.3 * b.shg_usual, kv=400.0), cavity=True, bench=b)
    assert r.state == "done" and rec.fell_back.startswith("its SHG peak on the right mode was no more than 0.7"), \
        rec.fell_back
    assert "if the optimum has changed, take it again (right-click › Take the optimum now)" in rec.fell_back
    b, lock = near_bench(seed=43)
    hop_up(b, lock)
    b, rec, r = recover(b.laser, b.target, optimum(b, opt_shg=1.3 * b.shg_usual, kv=400.0, catch_min=0.7),
                        cavity=True, bench=b)
    assert r.state == "done" and not rec.fell_back and b.laser.m == 0 and b.single_mode(), r.error


def test_a_scope_that_shows_the_peaks_on_another_scale_is_learned_by_the_first_relock():
    """The SHG scope shows the intra-cavity signal at half the scale its photodiode reads it locked: the first catch
    finds the peak no more than 0.5 of the optimum's everywhere and walks the mode it is on; its relock learns how a
    sweep shows the peak against the signal locked, and the next hop is caught."""
    b, lock = near_bench(seed=46)
    real = b.dlc.params["laser1:nlo:pd:shg-int:photodiode"]
    b.dlc.params["laser1:nlo:pd:shg-int:photodiode"] = lambda: 2.0 * real()
    hop_up(b, lock)
    p = optimum(b, opt_shg=2.0 * b.shg_usual, shg_lock=2.0 * b.shg_usual, kv=400.0)
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done" and rec.fell_back.startswith("its SHG peak on the right mode was no more than 0.5"), \
        (r.error, rec.fell_back)
    ratio = r.result.learned["shg_ratio"]
    assert ratio == pytest.approx(0.49, abs=0.03)
    b, lock = near_bench(seed=46)  # (as it was, and the same hop)
    real = b.dlc.params["laser1:nlo:pd:shg-int:photodiode"]
    b.dlc.params["laser1:nlo:pd:shg-int:photodiode"] = lambda: 2.0 * real()
    hop_up(b, lock)
    b, rec, r = recover(b.laser, b.target, HopParams.from_dict({**p.to_dict(), "shg_ratio": ratio}), cavity=True,
                        bench=b)
    assert r.state == "done" and not rec.fell_back and b.laser.m == 0 and lock.locked() and b.single_mode(), r.error


def test_how_its_sweep_shows_the_peak_against_the_signal_locked_is_learned():
    """Its photodiode reads the signal locked 20 % lower than the SHG scope shows the peaks: the sweep's peaks are
    compared with the optimum's as the scope would show it, from then on."""
    b, lock = near_bench(seed=44)
    real = b.dlc.params["laser1:nlo:pd:shg-int:photodiode"]
    b.dlc.params["laser1:nlo:pd:shg-int:photodiode"] = lambda: 0.8 * real()
    hop_up(b, lock)
    p = optimum(b, opt_shg=0.8 * b.shg_usual, shg_lock=0.8 * b.shg_usual, kv=400.0)
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done" and b.laser.m == 0 and b.single_mode(), r.error
    assert r.result.learned["shg_ratio"] == pytest.approx(1.26, abs=0.04)


@pytest.mark.parametrize("ase", [0.0, 0.7], ids=["seed-tells", "seed-fooled"])
def test_a_laser_that_stopped_lasing_lases_again_and_is_caught(ase):
    """Not lasing when the recovery begins: its seed power says so, or (sending its TA 70 % of its power, broadband)
    the wavemeter's silence and its flat SHG sweep do. The current jumps up, comes back down slowly, and it is
    caught on its own mode."""
    b, lock = near_bench(seed=42, i_stop=I0 - 1.3, i_start=I0 + 2.4)
    b.laser.stop()
    b.laser.ase = ase
    b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0), cavity=True, bench=b)
    assert r.state == "done", r.error
    assert rec.panics == 1 and b.laser.lasing and lock.locked() and abs(b.off_mhz()) < 1 and b.single_mode()
    assert b.laser.m == 0 and abs(b.V - V0) < 1.5


def test_the_catch_with_the_feedforward():
    """With the DLC pro's feedforward on, the window is on the measured current, wherever the piezo goes."""
    b, lock = near_bench(seed=45, ff=-0.15, hysteresis=0.8, multimode=0.3, ksv=0.32, i_stop=I0 - 1.2,
                         i_start=I0 + 2.5)
    hop_up(b, lock, 2.3)
    p = optimum(b, i_window=0.95, v_window=40.0, reset_up=0.0)  # (its piezo tuning measured: the feedforward's share)
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b)
    assert r.state == "done", r.error
    assert rec.panics == 0 and b.laser.lasing and b.laser.m == 0 and lock.locked() and abs(b.off_mhz()) < 1
    acts = measured_currents(b)
    assert min(acts) >= I0 - 1.0 - 1e-6 and max(acts) <= I0 + 0.95 + 1e-6  # (values written to six decimals)


def test_monte_carlo_catches_over_random_lasers_with_near_modes():
    """Random masters, their modes 1.8 to 3 GHz apart, hopping one mode either way: caught back on the right mode
    wherever that can be single-mode on target within the window (and the piezo has the room for its tries from the
    optimum's +- 30 V); else walked on the mode they are on. Never left unlocked."""
    right = feasible = 0
    for seed in range(40):
        rng = random.Random(7000 + seed)
        fsr = rng.uniform(1.8, 3.0)
        laser = ECDL(400.0, fsr=fsr, hysteresis=rng.uniform(0.1, 0.35) * fsr, multimode=rng.uniform(0.1, 0.25) * fsr,
                     ki=rng.uniform(-0.4, -0.15), ksi=rng.uniform(-3.0, -1.5), ksv=rng.uniform(0.28, 0.36),
                     mm_noise=0.004, seed=seed)
        b, lock = locked_bench(laser, seed=seed, shg=True, lockpoints=True, pdh_dc=rng.uniform(-0.03, 0.03))
        lo, hi = laser.limits()
        a, margin = laser.misalignment(laser.m, V0 + lock.dv, b.act), rng.uniform(0.02, 0.3) * fsr
        laser.nudge((a - lo + margin) if rng.random() < 0.5 else -(hi - a + margin))
        lock.frequency()
        assert laser.m != 0, seed
        mis = [laser.misalignment(0, V0, I0 + 0.01 * k) for k in range(-100, 201)]
        can = any(abs(a) < laser.zone(a)[0] for a in mis)  # the right mode single-mode on target in the window
        b, rec, r = recover(laser, b.target, optimum(b, v_window=40.0), noise=rng.uniform(0.3, 3.0), cavity=True,
                            bench=b, seed=seed)
        assert r.state == "done", (seed, r.error)
        assert lock.locked() and abs(b.off_mhz()) < 1 and b.second_mode() < 0.1, seed
        feasible += can
        right += can and laser.m == 0
    assert feasible >= 30 and right >= 0.95 * feasible, (right, feasible)


def test_stopped_lasing_far_up_the_piezo_the_piezo_goes_home_and_that_is_not_tried_again():
    """Record 84 (the 739 nm TA-SHG, 24 October). Not caught where it was, the catch tried from the optimum's piezo
    voltage +30 V. That far up, its feedback fades and it stopped lasing (its SHG sweep flat, its seed power fooled).
    The panic jumped the current up until it lased, then on the way back down it found it stops lasing above its
    window: failed, and left at the top of its current, 30 V up the piezo. Now a panic with the piezo that far from
    where it began blames the piezo. It goes home first (the current still up), learns no floor there, and is never
    tried there again."""
    b, lock = near_bench(seed=45, hysteresis=0.8, multimode=0.3, ksv=0.32, i_stop=I0 - 1.3, i_start=I0 + 2.4,
                         dark_v=(V0 + 20.0, 0.25))
    b.laser.ase = 0.9
    hop_up(b, lock, 2.3)
    p = optimum(b, i_window=0.95, v_window=40.0, kv=400.0, opt_shg=1.15 * b.shg_usual)
    b, rec, r = recover(b.laser, b.target, p, cavity=True, bench=b, seed=45)
    assert r.state == "done", r.error
    assert rec.panics == 1 and b.laser.lasing and lock.locked() and abs(b.off_mhz()) < 1
    assert abs(b.V - V0) < 6 and b.act < I0 + 0.95  # (not left up the piezo, nor high)
    assert rec.mA(rec.i_lo) == pytest.approx(I0 - 1.0)  # (no floor learned)
    assert any(n.startswith("not lasing (at ") and n.endswith(", current up at once, and the piezo back from 100.00 to "
                                                              "70.00 V, where it lased (not tried there again)")
               for n in rec.notes), rec.notes
    assert "100.00 V (the optimum's piezo voltage 70.00 V +30 V): it stopped lasing there: not tried again" in rec.notes
    assert all(g["v"] != pytest.approx(V0 + 30) for g in r.result.record["catch"][4:])


def stuck_bench(seed=45, **kw):
    """A hop that its window cannot undo. Its modes hold on far longer going up than going down (skewed hysteresis),
    and a dip of the current hopped it onto the next mode, which holds over the whole window (+0.7 mA). This is
    record 84's goes 1 to 3: up to the top of its window and down found nothing."""
    b, lock = near_bench(seed=seed, hysteresis=2.0, skew=0.8, multimode=0.3, **kw)
    d = b.dlc.params
    lock._lock_enabled(False)  # (as if its lock let go)
    d["laser1:dl:cc:current-set"] = I0 - 1.0
    b.laser.lase(V0, b.act)
    d["laser1:dl:cc:current-set"] = I0
    b.laser.lase(V0, b.act)
    lock._lock_enabled(True)
    lock.frequency()
    assert b.laser.m == 1 and lock.locked() and abs(b.off_mhz()) > 1000
    return b, lock


def test_a_hop_its_window_cannot_undo_is_undone_by_a_reset_as_by_hand():
    """As by hand, after a hop: the piezo and the current to the optimum's, the current 3 mA up, down a mA at a time
    to 1 mA above, then on down 0.1 mA at a time, a reading each. Coming from that far above, it lands on its right
    mode; then on as ever, the current to the top end of where the mode is at its best (here the top of its window),
    and relocked. The goes up and down (to the top of its window) cannot bring it back: the mode it is on is walked
    onto the target instead."""
    b, lock = stuck_bench()
    b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0, i_window=0.7), cavity=True, bench=b)
    assert r.state == "done", r.error
    assert b.laser.m == 0 and lock.locked() and abs(b.off_mhz()) < 1 and b.single_mode()
    assert rec.resets_done == 1 and rec.tries == 0
    assert rec.topped == pytest.approx((I0, I0 + 0.7)) and b.act == pytest.approx(I0 + 0.7)  # (parked high after it)
    assert r.result.summary.startswith("relocked, back on its right mode by a reset (the piezo to 70.00 V, the current "
                                       "up 3 mA and down to 150.00 mA), then to the top end of where it is at its "
                                       "best, 150.70 mA"), r.result.summary
    reset = r.result.record["reset"]
    assert len(reset) == 1 and reset[0]["on"]
    assert [x for x, _ in reset[0]["points"]] == pytest.approx([I0 + 2, I0 + 1] + [I0 + 0.1 * k
                                                                                   for k in range(9, -1, -1)])
    currents = b.writes("dl:cc:current-set")
    top = currents.index(max(currents))
    assert max(currents) == pytest.approx(I0 + 3.0) and currents[:top] == sorted(currents[:top])  # up at once...
    down = currents[top:currents.index(min(currents[top:]))]
    assert down == sorted(down, reverse=True)  # ...then down
    assert "reset 1: on the right mode at 150.00 mA, 70.00 V, -0.000 GHz from the target, its SHG peak 1.00 of the " \
           "optimum's" in rec.notes

    b, lock = stuck_bench()
    b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0, i_window=0.7, reset_up=0.0), cavity=True, bench=b)
    assert r.state == "done" and b.laser.m == 1 and rec.fell_back and rec.resets_done == 0, r.error
    assert [g["caught"] for g in r.result.record["catch"]][:3] == [False] * 3


def test_the_reset_goes_to_the_optimum_s_piezo_voltage_with_the_feedforward():
    """The optimum was taken 2.4 V lower on the piezo, its feedforward on (-0.145 mA/V). The reset goes there, the
    current reckoned as measured (150.00 mA there, as the optimum's), then walks the piezo back onto the target."""
    b, lock = stuck_bench(ff=-0.145)
    b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0, i_window=0.7, opt_v=V0 - 2.4), cavity=True, bench=b)
    assert r.state == "done", r.error
    assert b.laser.m == 0 and lock.locked() and abs(b.off_mhz()) < 1 and b.single_mode()
    reset = r.result.record["reset"][0]
    assert reset["on"] and reset["v"] == pytest.approx(V0 - 2.4) and reset["points"][-1][0] == pytest.approx(I0)
    assert r.result.summary.startswith("relocked, back on its right mode by a reset (the piezo to 67.60 V, the "
                                       "current up 3 mA and down to 150.00 mA), then to the top end of where it is at "
                                       "its best, 150.70 mA, the piezo 67.60 → 70.3"), r.result.summary


def test_monte_carlo_resets_over_masters_whose_modes_hold_on_longer_one_way():
    """Random masters, their modes 1.8 to 3 GHz apart, holding on far longer going up than going down (hysteresis
    0.6 to 0.95 of the spacing, skewed 0.5 to 0.9), hopped onto the next mode by a dip of the current (0.6 to 1.5
    mA). The reset brings every one back on its right mode, locked on target, single-mode; the goes alone (to the
    top of the window, 0.7 mA up), hardly any."""
    def bench(seed):
        rng = random.Random(9000 + seed)
        fsr = rng.uniform(1.8, 3.0)
        b, lock = near_bench(seed=seed, fsr=fsr, skew=rng.uniform(0.5, 0.9), hysteresis=rng.uniform(0.6, 0.95) * fsr,
                             multimode=0.3)
        d, dip = b.dlc.params, rng.uniform(0.6, 1.5)
        lock._lock_enabled(False)
        d["laser1:dl:cc:current-set"] = I0 - dip
        b.laser.lase(V0, b.act)
        d["laser1:dl:cc:current-set"] = I0
        b.laser.lase(V0, b.act)
        lock._lock_enabled(True)
        lock.frequency()
        return b, lock

    hopped = right = alone = 0
    for seed in range(24):
        b, lock = bench(seed)
        if b.laser.m == 0:
            continue  # (the dip did not hop it)
        hopped += 1
        b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0, i_window=0.7), cavity=True, bench=b, seed=seed)
        assert r.state == "done" and lock.locked() and abs(b.off_mhz()) < 1 and b.single_mode(), (seed, r.error)
        right += b.laser.m == 0
        b, lock = bench(seed)
        b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0, i_window=0.7, reset_up=0.0), cavity=True, bench=b,
                            seed=seed)
        assert r.state == "done" and lock.locked(), (seed, r.error)
        alone += b.laser.m == 0
    assert hopped >= 12 and right == hopped and alone <= hopped // 4, (hopped, right, alone)


def test_step_by_step_the_reset_asks_first():
    b, lock = stuck_bench()
    p = optimum(b, kv=400.0, i_window=0.7, confirm=True)
    rec = Recenter(p, b.I, b.V, I0 - 1.0, I0 + 0.7, b.V - 25.0, b.V + 25.0, cavity=True, i_top=I0 + 5.0)
    runner, asked = Runner(rec, b, b.t), []
    while runner.busy:
        if b.pending:
            b.pending.pop(0)()
        elif runner.asking:
            asked.append(runner.asking)
            runner.confirm()
        else:
            runner.on_reading(*b.reading())
    assert runner.state == "done" and b.laser.m == 0, runner.error
    assert asked[0].startswith("Take the laser over") and asked[1] == (
        "Reset, as by hand: the piezo to 70.00 V and the current to 150.00 mA, then up 3 mA, down a mA at a time to 1 "
        "mA above it, and on down 0.1 mA at a time to 150.00 mA, where it should be on its right mode again")
    assert asked[2].startswith("Back on its right mode: the current up a step at a time while its SHG peak stays at "
                               "its best"), asked


# --------------------------------------------- the top of its mode: its SHG signal locked dips
@pytest.mark.parametrize("method, into, walk", [("catch", 0.0, -0.6), ("catch", -0.03, -0.3), ("walk", -0.03, -0.6)])
def test_relocked_over_the_top_of_its_mode_it_is_eased_off_it_by_its_shg_signal(method, into, walk):
    """Parked at the top of its mode, the piezo walked down (its feedforward takes the current up) leaves it just over
    the top: a second mode begins, and its SHG signal locked dips below what the mode gives, a hop next. Relocked so,
    the current goes down a little at a time while its signal rises, and is left 0.02 mA below where it stopped
    rising: single-mode. Without that, it is handed back with a second mode."""
    def bench():
        b, lock = near_bench(seed=47, ff=-0.145)
        las = b.laser
        las.nudge(-(below_second_mode(b, lock) - into) * (las.ki - las.ksi))
        las.f0 -= walk * 0.4e-3  # (it drifted: the piezo must go `walk` V to the target, the current with it)
        lock.jump(1)
        return b, lock

    b, lock = bench()
    b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0, method=method), cavity=True, bench=b)
    assert r.state == "done", r.error
    assert lock.locked() and b.laser.m == 0 and b.single_mode() and 0.015 < below_second_mode(b, lock) < 0.2
    ease = r.result.record["ease"]
    assert ease["to"] < ease["from"] - 0.02 and ease["signal"][1] > ease["signal"][0]
    assert any(n.startswith("off the top of its mode: its SHG signal dipping (") for n in rec.notes), rec.notes
    log = [(n.replace("laser1:", ""), v) for n, v in b.dlc.log]
    eased = []  # (the current's writes just after its SHG cavity locked)
    for k in (j for j, (n, v) in enumerate(log) if n == "nlo:shg:lock:lock-enabled" and v):
        for n, v in log[k + 1:]:
            if n != "dl:cc:current-set":
                break
            eased.append(v)
    assert len(eased) > 10 and max(abs(x - y) for x, y in zip(eased, eased[1:])) <= 0.02 + 1e-9  # (a little at a time)

    b, lock = bench()
    b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0, method=method, ease_step=0.0), cavity=True, bench=b)
    assert r.state == "done" and "ease" not in r.result.record and b.second_mode() > 0.02, r.error


def test_a_laser_whose_shg_signal_is_as_usual_is_not_eased():
    """Its SHG signal locked as high as usual: not at the top of its mode, nothing to ease (nor any probing)."""
    b, lock = near_bench(seed=41)
    lock.jump(1)
    b, rec, r = recover(b.laser, b.target, optimum(b, kv=400.0), cavity=True, bench=b)
    assert r.state == "done" and "ease" not in r.result.record, r.error
    assert not any(n.startswith("off the top of its mode") for n in rec.notes)


# --------------------------------------------- the upkeep: the top of its mode found now and then, locked
def upkeep_bench(below, seed=47):
    """Locked, as by hand, `below` mA under where a second mode begins (negative: over the top already)."""
    b, lock = near_bench(seed=seed, ff=-0.145)
    las = b.laser
    las.nudge(-(below_second_mode(b, lock) - below) * (las.ki - las.ksi))
    lock.frequency()
    assert lock.locked() and below_second_mode(b, lock) == pytest.approx(below, abs=1e-3)
    return b, lock


def upkeep(b, p=None, until=None):
    p = p or optimum(b, kv=400.0)
    rec = Recenter(p, b.I, b.V, I0 - 1.0, I0 + 2.0, b.V - 25.0, b.V + 25.0, cavity=True, i_top=I0 + 5.0, upkeep=True)
    return rec, b.run(rec, until=until)


@pytest.mark.parametrize("below", [0.3, 0.05, -0.02])
def test_the_upkeep_finds_the_top_of_its_mode_and_keeps_it_below(below):
    """Now and then, locked (both locks left on): the current up 0.005 mA at a time until its SHG signal dips (a
    second mode beginning), then down until it is back up, and 0.02 mA more. From well below the top, from near it,
    and from over it already: it ends just below where a second mode begins, still locked, on target."""
    b, lock = upkeep_bench(below)
    rec, r = upkeep(b)
    assert r.state == "done", r.error
    assert lock.locked() and lock.shg_state() == (5, "locked") and abs(b.off_mhz()) < 1 and b.single_mode()
    assert 0.015 < below_second_mode(b, lock) < 0.06
    currents = [b.start[0]] + b.writes("dl:cc:current-set")
    assert max(abs(x - y) for x, y in zip(currents, currents[1:])) <= 0.005 + 1e-9  # (steps its cavity lock follows)
    assert [n for n, _ in b.dlc.log if "cc:current-set" not in n] == []  # (nothing else written)
    up = r.result.record["upkeep"]
    assert up["top"] > up["to"] and r.result.summary.startswith(
        f"the top of its mode at {up['top']:.3f} mA, the current 150.000 → {up['to']:.3f} mA")
    assert not r.result.summary.startswith("relocked")


def test_the_upkeep_with_no_dip_within_reach_leaves_the_current_where_it_was():
    b, lock = upkeep_bench(0.8)  # (more than UP_MAX below the top)
    rec, r = upkeep(b)
    assert r.state == "done" and b.I == pytest.approx(b.start[0]) and lock.locked(), r.error
    assert r.result.summary == "the top of its mode not found within 0.5 mA: left at 150.000 mA"


def test_the_upkeep_puts_the_current_back_when_its_shg_lock_is_lost_or_was_not_locked():
    b, lock = upkeep_bench(0.3)
    real, reads = b.dlc.params["laser1:nlo:pd:shg-int:photodiode"], [0]

    def intra():  # (its SHG cavity loses its lock after 40 readings)
        reads[0] += 1
        return real() if reads[0] < 40 else 0.05

    b.dlc.params["laser1:nlo:pd:shg-int:photodiode"] = intra
    rec, r = upkeep(b)
    assert r.state == "failed" and r.error.startswith("its SHG signal fell to 0.050 V (it was 1.2"), r.error
    assert b.I == pytest.approx(b.start[0]) and lock.locked()
    currents = [b.start[0]] + b.writes("dl:cc:current-set")
    assert max(abs(x - y) for x, y in zip(currents, currents[1:])) <= 0.005 + 1e-9

    b, lock = upkeep_bench(0.3)
    b.dlc.params["laser1:nlo:shg:lock:lock-enabled"] = False
    rec, r = upkeep(b)
    assert r.state == "failed" and r.error.startswith("not locked now (its cavity lock “locked”, its SHG cavity "), \
        r.error
    assert b.writes("dl:cc:current-set") == []


def test_no_upkeep_for_a_current_outside_its_window():
    """Its nominal current changed since it was relocked, say: the upkeep does not drag the current into the window
    (that is a recovery's to do, slowly); nothing is written."""
    b, lock = upkeep_bench(0.3)
    rec = Recenter(optimum(b, kv=400.0), b.I, b.V, I0 + 0.5, I0 + 2.0, b.V - 25.0, b.V + 25.0, cavity=True,
                   i_top=I0 + 5.0, upkeep=True)
    r = b.run(rec)
    assert r.state == "failed" and r.error == ("the current, 150.000 mA, is outside its window (150.50 … 152.00 mA): "
                                               "no upkeep"), r.error
    assert b.writes("dl:cc:current-set") == [] and lock.locked()


def test_monte_carlo_upkeeps():
    """The upkeep on 10 lasers, each from 0.4 mA below the top of its mode to 0.05 mA over it (one of them with no
    easing after a relock, ease_step 0): single-mode at the end, locked on target, 0.01 to 0.07 mA below where a
    second mode begins, never more than 0.005 mA a write. A shallow dip rises slowly going down: two looks that
    bring next to nothing must not stop it short, as in the end-to-end run that left it 0.02 mA below where it
    began instead."""
    ends = []
    for seed in range(40, 50):
        for below in (0.4, 0.04, -0.02, -0.05):
            b, lock = upkeep_bench(below, seed=seed)
            rec, r = upkeep(b, optimum(b, kv=400.0, ease_step=0.0 if seed == 49 else 0.001))
            assert r.state == "done", (seed, below, r.error)
            end = below_second_mode(b, lock)
            assert lock.locked() and b.single_mode() and abs(b.off_mhz()) < 1 and 0.01 < end < 0.07, (seed, below, end)
            currents = [b.start[0]] + b.writes("dl:cc:current-set")
            assert max(abs(x - y) for x, y in zip(currents, currents[1:])) <= 0.005 + 1e-9
            ends.append(end)
    assert sorted(ends)[len(ends) // 2] < 0.035, sorted(ends)


# ------------------------------------------- the catch with nothing but the wavemeter (a plain DL pro)
WM_CATCH = HopParams(enabled=True, i_nom=I0, method="catch")


def plain_near(by=1.6, seed=40, **ecdl):
    """A DL pro with nothing but the wavemeter to tell how it is (no cavity, no SHG stage, no photodiode), a bit mode
    hoppy: its modes 2.2 GHz apart, going multimode near their edges. Its filter drifts by `by` GHz from the middle of
    its target mode: it hops to the next mode (up for by > 0)."""
    kw = dict(fsr=2.2, hysteresis=0.3, multimode=0.4, mm_noise=0.004)
    kw.update(ecdl)
    laser = ECDL(400.0, seed=seed, **kw)
    target = laser.freq(V0, I0)
    laser.nudge(by)
    laser.freq(V0, I0)
    return laser, target


def test_a_dl_pro_with_only_the_wavemeter_is_caught_back_and_parked_in_the_middle_of_its_mode():
    """As the SHG catch, by hand: the current up, then down a step at a time until it is on the right mode. With no
    SHG sweep to say where the mode is at its best, that is where it hopped onto it: its edge (left there, it would
    hop again with the next drift). So, by the wavemeter alone, where the mode holds is found: on down until it hops
    off, back up until it hops onto it again (the other mode holds on a while), and it is parked in the middle of
    where it is alone, from below; then the piezo onto the target. Nothing else is touched."""
    laser, target = plain_near()
    assert laser.m == 1
    b, rec, r = recover(laser, target, WM_CATCH)
    assert r.state == "done", r.error
    assert laser.m == 0 and abs(b.off_mhz()) < 5  # the right mode, on target
    assert b.margin() > 0.4  # (alone 0.46 mA either way of its middle at most)
    notes = "\n".join(rec.notes)
    assert re.search(r"caught the right mode at 151\.30 mA, 70\.00 V \(go 1\)", notes), notes
    assert re.search(r"the mode it was caught on: alone 150\.\d\d–151\.30 mA; parked at 150\.\d\d mA, from below "
                     r"\(0\.\d\d mA from where it ends", notes), notes
    assert re.match(r"caught the right mode at 151\.30 mA \(1 go up and down\), walked the piezo 70\.00 → 70\.5\d V, "
                    r"current 150\.00 → 150\.\d\d mA measured \(\d\d readings\); parked 0\.[34]\d mA inside its mode",
                    r.result.summary), r.result.summary
    c = r.result.record["center"]
    assert c["edge"] and c["off"] and c["back"] < c["op"] < c["top"] == pytest.approx(151.3)
    currents = b.writes("dl:cc:current-set")
    top = currents.index(max(currents))
    assert max(currents) == pytest.approx(I0 + 2.0) and currents[:top] == sorted(currents[:top])  # up quickly...
    low = currents.index(min(currents))
    assert all(0.09 < x - y < 0.11 for x, y in zip(currents[top:low], currents[top + 1:low + 1]))  # ...down steps
    assert min(currents) < c["back"] - 0.3  # (it hopped off going down...)
    assert {n for n, _ in b.dlc.log} == {"laser1:dl:cc:current-set", "laser1:scan:offset"}  # (...nothing but these)


def test_left_where_it_was_caught_a_mode_hoppy_laser_would_sit_at_its_edge():
    """Why it is parked: where the catch found it, the laser is at the edge of the mode (hardly any margin), and
    the map, sweeping the whole window both ways, parks it short of the middle."""
    laser, target = plain_near()
    b, rec, r = recover(laser, target, HopParams.from_dict({**WM_CATCH.to_dict(), "method": "map"}), learned=False)
    assert r.state == "done" and laser.m == 0 and 0.2 < b.margin() < 0.4 and rec.readings > 80, r.error
    laser, target = plain_near()
    b, rec, r = recover(laser, target, WM_CATCH)
    assert b.margin() > 0.4 and rec.readings < 60  # (the catch: a better place, in fewer readings)
    edge = laser.margins(b.V, rec.caught[1])[0]  # (where it was caught, as the catch found it)
    assert edge < 0.1


def test_too_little_of_the_right_mode_in_the_window_has_the_modes_mapped_instead():
    """It drifted the other way: the right mode comes back only at the bottom of the window, too little of it for any
    margin. Caught again it would only be the same: the modes are mapped instead, and the one that holds best is put
    on the target, as the map does."""
    laser, target = plain_near(by=-2.2)
    b, rec, r = recover(laser, target, WM_CATCH)
    assert r.state == "done", r.error
    assert rec.narrow.startswith("the right mode is lasing on it 149.00–149.20 mA, the window's bottom on: too little "
                                 "of it for a margin of 0.15 mA either way"), rec.narrow
    assert rec.tries == 1 and rec.maps >= 1 and abs(b.off_mhz()) < 5 and b.margin() > 0.1
    assert r.result.summary.startswith("the right mode not caught, mapped instead: recentered 150.00 → "), \
        r.result.summary


def test_recenter_now_puts_a_laser_on_its_mode_in_the_middle_of_it():
    """On its right mode and on target, but near the edge of the mode (where it drifted to, locked): Recenter now looks
    at where it is in the mode too, by the wavemeter, and parks it in the middle (else only the piezo would move)."""
    laser = ECDL(400.0, seed=40, fsr=2.2, hysteresis=0.3, multimode=0.4, mm_noise=0.004)
    laser.nudge(0.7)  # (0.1 GHz short of where another mode could lase)
    target = laser.freq(V0, I0)
    b = Bench(laser, target, seed=40)
    assert laser.m == 0 and b.margin() < 0.1
    b, rec, r = recover(laser, target, WM_CATCH, quick=False, bench=b)
    assert r.state == "done", r.error
    assert laser.m == 0 and abs(b.off_mhz()) < 5 and b.margin() > 0.35
    assert "on the right mode: Recenter now, so where it is in the mode is looked at too, by the wavemeter" in rec.notes


def near_case(seed):
    """A random plain laser with near modes (1.8 to 3 GHz apart), on its target mode, that has hopped one mode either
    way."""
    rng = random.Random(9000 + seed)
    fsr = rng.uniform(1.8, 3.0)
    laser = ECDL(400.0, fsr=fsr, hysteresis=rng.uniform(0.05, 0.3) * fsr, multimode=rng.uniform(0.1, 0.25) * fsr,
                 ki=rng.uniform(-0.4, -0.15), ksi=rng.uniform(-3.0, -1.5), ksv=rng.uniform(0.28, 0.36),
                 mm_noise=rng.choice([0.004, 0.03]), seed=seed)
    laser.nudge(rng.uniform(-0.15, 0.15) * fsr)
    laser.m = 0
    target = laser.freq(V0, I0)
    lo, hi = laser.limits()
    a, margin = laser.misalignment(laser.m, V0, I0), rng.uniform(0.02, 0.3) * fsr
    laser.nudge((a - lo + margin) if rng.random() < 0.5 else -(hi - a + margin))
    laser.freq(V0, I0)
    return laser, target, rng.choice([0.3, 1.0, 3.0])


def test_monte_carlo_catches_by_the_wavemeter_alone():
    """Random plain lasers, mode hoppy (modes 1.8 to 3 GHz apart), hopping one mode either way: caught back on the
    right mode and parked well inside it wherever that can be had; else mapped. Never left at the edge of a mode,
    and in half the readings the map takes."""
    p = HopParams.from_dict({**WM_CATCH.to_dict(), "i_below": 2.0})
    right = feasible = inside = 0
    readings = []
    for seed in range(40):
        laser, target, noise = near_case(seed)
        best = best_margin(laser, p)
        b, rec, r = recover(laser, target, p, noise=noise, seed=seed)
        assert r.state == "done", (seed, r.error)
        assert abs(b.off_mhz()) < 5 + 3 * noise, seed
        m = b.margin()
        assert m > -0.15, seed  # never on the edge of a mode
        inside += m > 0.1
        readings.append(rec.readings)
        if best >= 0.3:
            feasible += 1
            right += laser.m == 0
    assert feasible >= 25 and right >= 0.95 * feasible, (right, feasible)
    assert inside >= 36 and sorted(readings)[20] < 80, (inside, sorted(readings))


# ------------------------------------------------- what the feedforward does, seen
def test_a_feedforward_that_does_nothing_is_seen_at_the_first_piezo_move():
    """On a DL pro whose piezo the scan's offset moves (its scan off) the feedforward follows the scan alone: its factor
    says -0.3 mA/V, but the current stays where it is set as the piezo moves (the lab's 493 and 650 nm lasers). A
    recovery sees that at its first piezo move, reckons the current so from then on (its window on the current as it
    is), moves the current along itself as it tunes the piezo (as the feedforward set is meant to), and keeps what it
    saw for the next."""
    laser = ECDL(400.0, seed=3)
    target = laser.freq(V0, I0) + 0.6e-3  # THz: 600 MHz above it, on its mode
    b = Bench(laser, target)
    d = b.dlc.params
    d["laser1:dl:cc:feedforward-enabled"], d["laser1:dl:cc:feedforward-factor"] = True, -0.3
    b.dlc.ff_from[1] = None
    b, rec, r = recover(laser, target, HopParams(enabled=True, i_nom=I0), bench=b)  # (its window kept: check_safety)
    assert r.state == "done" and abs(b.off_mhz()) < 5 and rec.maps == 0
    assert rec.ff == 0.0 and rec.learned["ff_set"] == -0.3 and rec.learned["ff_act"] == 0.0
    assert any("it adds nothing as the piezo moves" in n for n in rec.notes)
    assert rec.mA(rec.I) == pytest.approx(b.act, abs=1e-6)  # (as it is)
    assert b.V > V0 + 0.5 and b.I - I0 == pytest.approx(-0.3 * (b.V - V0), abs=0.06)  # (moved along, as it tuned)


def test_what_its_feedforward_does_is_kept_for_the_next_recovery():
    seen = HopParams(enabled=True, i_nom=I0, ff_set=-0.3, ff_act=0.0)
    since = HopParams(enabled=True, i_nom=I0, ff_set=-0.5, ff_act=0.0)  # (seen at another factor: looked at again)
    for p, probe in ((seen, False), (since, True)):
        laser = ECDL(400.0, seed=3)
        b = Bench(laser, laser.freq(V0, I0) + 0.6e-3)
        d = b.dlc.params
        d["laser1:dl:cc:feedforward-enabled"], d["laser1:dl:cc:feedforward-factor"] = True, -0.3
        b.dlc.ff_from[1] = None
        b, rec, r = recover(laser, b.target, p, bench=b)
        assert r.state == "done" and rec.ff == 0.0
        assert any("as its recoveries saw, it adds nothing" in n for n in rec.notes) is not probe
        assert ("ff_act" in rec.learned) is probe


def test_the_feedforward_that_does_what_its_factor_says_is_seen_so():
    laser = ECDL(400.0, seed=3)
    b = Bench(laser, laser.freq(V0, I0) + 0.6e-3)
    d = b.dlc.params
    d["laser1:dl:cc:feedforward-enabled"], d["laser1:dl:cc:feedforward-factor"] = True, -0.3
    b.dlc.ff_from[1] = V0  # (it moves the current as its factor says, from where it began)
    b, rec, r = recover(laser, b.target, HopParams(enabled=True, i_nom=I0), bench=b)
    assert r.state == "done" and rec.ff == -0.3 and rec.learned["ff_act"] == -0.3
    assert rec.mA(rec.I) == pytest.approx(b.act, abs=1e-6) and rec.ff_soft == 0.0
    assert b.I == pytest.approx(I0)  # (the DLC pro moved it as the piezo went: not moved besides)


def test_put_back_from_the_bottom_of_its_window_the_current_never_leaves_it():
    """10:26 on 27 October: a recovery that failed with the piezo 5.3 V up and the current at the bottom of its
    window could not put them back: the current's first step down came before the piezo's (which, through the
    feedforward, brings the current back up), so that it would have left its window meanwhile, and that write was
    refused: the laser was left there. The piezo first, then."""
    class Gone(Recenter):
        def run(self):
            yield from self.current_setup()
            yield from self.move(offset=self.V0 + 5.3, read=False)
            yield from self.move(current=self.i_lo, read=False)
            raise Failed("no mode")

    b = Bench(ECDL(400.0, seed=3), 0.0)
    d = b.dlc.params
    d["laser1:dl:cc:feedforward-enabled"], d["laser1:dl:cc:feedforward-factor"] = True, -0.6
    b.dlc.ff_from[1] = V0
    p = HopParams(enabled=True, i_nom=I0, ff_set=-0.6, ff_act=-0.6)
    r = b.run(Gone(p, I0, V0, I0 - 1.0, I0 + 2.0, V0 - 25.0, V0 + 25.0))
    assert r.state == "failed" and r.error == "no mode" and r.restored
    assert (b.I, b.V) == (I0, V0)
    acts = measured_currents(b)
    assert min(acts) >= I0 - 1.0 - 1e-6 and max(acts) <= I0 + 2.0 + 1e-6


def test_a_tuning_far_off_what_it_went_by_is_taken_from_what_it_did():
    """10:26 on 27 October: the piezo's tuning its model gave (0.9 GHz/V) was less than half what it does: each step
    overshot the target further than the last, and a tuning off by more than twice was never taken from what it did.
    Now a step that moved it more than any scatter tells sets it: on target in a few steps."""
    laser = ECDL(400.0, kv=0.4, seed=3)
    target = laser.freq(V0, I0) + 0.328e-3
    b = Bench(laser, target)
    p = HopParams(enabled=True, i_nom=I0)
    rec = Recenter(p, I0, V0, I0 - 1.0, I0 + 2.0, V0 - 25.0, V0 + 25.0, kv=186.0)
    rec.kv_measured = True  # (as its model's tuning is taken)
    r = b.run(rec)
    assert r.state == "done" and rec.maps == 0 and abs(b.off_mhz()) < 5
    assert len(b.writes("scan:offset")) <= 4 and rec.kv == pytest.approx(400.0, rel=0.2)
