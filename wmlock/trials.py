"""Random lasers, simulated, and what the program does with them: trials of its mode-hop recoveries and of its keeping
a laser off the edge of its mode, in numbers.

    python -m wmlock.trials recover far --n 100         # a kind of laser (below), hopped, recovered: how often, how fast
    python -m wmlock.trials recover far --n 100 --set smooth=true   # the same, its sweeps ramped (as it comes: steps)
    python -m wmlock.trials recover all --n 60 --json out.json
    python -m wmlock.trials keep shg --n 30 --hours 6   # lasers drifting towards their hops: kept off them, or not
    python -m wmlock.trials lasers model --n 5          # what the random lasers are

The lasers (sim.py's, each from a seed, so that a trial can be run again):
  far     an ECDL whose external modes are 3 to 10 GHz apart, one selection filter, a little hysteresis; hopped one
          or two modes (the map's lasers: a DL pro with nothing but the wavemeter to go by)
  strong  the same with strong hysteresis, more one way than the other (where a mode lases sweeping up has little to
          do with where it lases sweeping down)
  shg     a TA-SHG pro's master, its DLC pro locking it to a cavity (PDH, a FALC pro), its SHG cavity locked to it;
          modes 3 to 10 GHz apart, multimode over a good part of each; hopped (the walk's and the map's lasers)
  near    the same whose next mode is close by (1.8 to 3 GHz), the optimum taken: hopped one mode (the catch's lasers)
  blue    the lab's 493 nm DL pro as its map's model has it (lasermodel.py: its chip's modes, 87 GHz apart, picking
          among its external modes, 8.3 GHz apart, in a window 270 GHz wide), the laser itself a little unlike the model
          the program has of it; its chip's modes drifting until it hops (the model's lasers)
  red     the lab's 650 nm DL pro likewise (a window only 27 GHz wide)
  model   a random laser of that kind (random(): cavity, chip, window, hysteresis, pull, lag all drawn), the program's
          model of it as a fit of its map would have it

A Rig is the laser on a fake DLC pro (sim.FakeDLCPro, spoken to through the real client) with the wavemeter reading it
every dt seconds, in simulated time: an operation on the controller takes op_s, a scope trace scope_s, and the
procedure is driven as the engine drives it (modehop.Runner: its pauses and ramps to the millisecond), so that how long
a recovery takes is what it would take on the laser, as far as the simulation goes. Nothing here is used by the
program itself.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import random
import statistics
import sys
import tempfile
import time
from dataclasses import asdict, dataclass, field

from .dlcpro import DecopError, DLCPro
from .modehop import HopParams, Recenter, Runner, perform
from .sim import ECDL, FakeDLCPro, ModelLaser, SimLocked

I0, V0 = 150.0, 70.0  # mA, V: where the simulator's ECDLs run
KINDS = ("far", "strong", "shg", "near", "blue", "red", "model")


class Loopback(DLCPro):
    """A DLCPro client for a FakeDLCPro in the same thread: no sockets."""

    def __init__(self, fake):
        super().__init__("loopback")
        self.fake = fake

    def command(self, cmd: str) -> str:
        return self.fake.execute(cmd).strip()


class Rig:
    """The laser on its controller and the wavemeter, in simulated time (the module's notes). laser: sim.ECDL or
    sim.ModelLaser (made with clock=rig.now for its chip's lag); target: THz the readings are from. The io of a
    modehop.Runner (do, think)."""

    def __init__(self, laser, target, noise=0.5, dt=0.1, seed=0, current=I0, offset=V0, scan=False, drift=0.0,
                 op_s=0.002, scope_s=0.05, think_s=1.0, fine=0.005, latency=0.0):
        """op_s: s an operation on its controller takes (a real DLC pro: some ms); latency: s a reading is old when
        it comes (its exposure, the wavemeter's sums, the way here: the laser has moved on meanwhile)."""
        self.laser, self.target = laser, target
        self.noise, self.dt, self.op_s, self.scope_s, self.think_s, self.fine = noise, dt, op_s, scope_s, think_s, fine
        self.latency = latency
        self.drift, self.wobble = drift, 0.0  # MHz: its random walk a reading (a free-running laser's), where it is
        self.start = (current, offset)
        self.dlc = FakeDLCPro("SN-R", "rig", {1: {"current": current, "offset": offset, "scan": scan}}, serve=False)
        self.client = Loopback(self.dlc)
        self.lock = None
        self.shg_usual = 0.0  # V: its SHG signal locked, as an engine would have learned it
        self.t = 0.0
        self.due = dt         # when the wavemeter's next reading comes
        self.rng = random.Random(seed)
        self.events = []      # (when, what to call): answers on their way
        self.ops = self.reads = 0
        self.told = False     # the runner is told the time as answers land (one without a clock of its own: run)
        self.also = None      # (t, e) -> what is left of its fringes (or None): called with each reading a run gets
        #                       (the engine's part meanwhile: its wavemeter lock going on, its fringes looked at)
        for name in ("dl:cc:current-set", "scan:offset"):  # the laser follows each change as it is made
            self.dlc.on_set["laser1:" + name] = self._settle

    # ------------------------------------------------------------ the laser
    def now(self) -> float:
        return self.t

    def _settle(self, _value=None):
        if self.lock is None or not self.lock.locked():
            self.laser.lase(self.V + (self.lock.dv if self.lock is not None else 0.0), self.act)

    def lock_it(self, **kwargs) -> SimLocked:
        """Its DLC pro locks it to a cavity with a resonance at its target (sim.SimLocked, on the rig's time)."""
        kwargs.setdefault("clock", self.now)
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

    def set(self, current=None, offset=None):
        """By hand: its set current and its piezo, at once (as someone at the controller would)."""
        if offset is not None:
            self.client.set("laser1:scan:offset", offset)
        if current is not None:
            self.client.set("laser1:dl:cc:current-set", current)

    def freq(self) -> float:
        if self.lock is not None:
            return self.lock.frequency(self.wobble)
        return self.laser.freq(self.V, self.act)

    def off_mhz(self) -> float:
        """MHz from its target, as it really is (not as the wavemeter reads it: on two modes, that is noise)."""
        if self.lock is not None and self.lock.locked():
            return (self.freq() - self.target) * 1e6
        las = self.laser
        v = self.V + (self.lock.dv if self.lock is not None else 0.0)
        if isinstance(las, ECDL):
            las.lase(v, self.act)
            return (las.f0 + las.mode_ghz(v, self.act) * 1e-3 - self.target) * 1e6 if las.lasing else math.nan
        ghz = las.lase(v, self.act)
        return (las.f0 + ghz * 1e-3 - self.target) * 1e6 if ghz is not None else math.nan

    def reading(self):
        """The wavemeter's next reading: (s, MHz from the target; NaN: none)."""
        self.t = max(self.t, self.due)
        self.due = self.t + self.dt
        self.reads += 1
        if self.drift:
            self.wobble += self.rng.gauss(0.0, self.drift)
        return self.t, (self.freq() - self.target) * 1e6 + self.rng.gauss(0.0, self.noise)

    def second_mode(self) -> float:
        """The part of its power in a second mode, where it is."""
        v = self.V + (self.lock.dv if self.lock is not None else 0.0)
        self.laser.lase(v, self.act)
        comps = self.laser.components(v, self.act)
        return min(p for p, _ in comps) if len(comps) > 1 else 0.0

    def margin(self) -> float:
        """mA from where it is to where its mode ends (hops off, or another could lase as well), the nearer way, as
        the laser really is (an ECDL: to the edge of where its mode is alone; a model's laser: to where it hops)."""
        las = self.laser
        v = self.V + (self.lock.dv if self.lock is not None else 0.0)
        if isinstance(las, ECDL):
            return (las.fsr / 2 - las.hysteresis - abs(las.misalignment(las.m, v, self.act))) / abs(las.ki - las.ksi)
        return min(holds(las, v, self.act))

    # ------------------------------------------------------- the Runner's io
    def do(self, op, done, failed):
        self.ops += 1
        cost = self.scope_s if op[0] == "get" and any("scope:data" in n for n in op[1]) else self.op_s
        try:
            result = perform(self.client, 1, op)
        except DecopError as exc:
            self.events.append((self.t + cost, lambda exc=exc: failed(exc)))
            return
        self.events.append((self.t + cost, lambda: done(result)))

    def think(self, fn, done, failed):
        """Worked out on the side (the engine's thread of its own): its answer think_s later."""
        self.events.append((self.t + self.think_s, lambda: done(fn())))

    def run(self, rec, timeout=None, until=None) -> Runner:
        """Drive the procedure to its end (or until(runner)): the answers on their way as they land, the readings
        as they come, and the runner woken whenever a pause or a ramp of its is due."""
        timeout = timeout or getattr(rec, "timeout", 900.0)
        try:
            runner = Runner(rec, self, self.t, timeout, clock=self.now)
            self.told = False
        except TypeError:  # (a Runner from before it had a clock: told the time here whenever an answer lands, so
            runner = Runner(rec, self, self.t, timeout)  # that its waits are as long as they say, as the other's)
            self.told = True
        while runner.busy and (until is None or not until(runner)):
            self.step(runner)
        return runner

    def step(self, runner):
        """The next thing that happens: an answer lands, the runner's pause is over, or a reading comes."""
        when = min((e[0] for e in self.events), default=math.inf)
        wake = _wake(runner)
        if when <= min(self.due, wake):
            k = min(range(len(self.events)), key=lambda j: self.events[j][0])
            when, call = self.events.pop(k)
            self.t = max(self.t, when)
            if self.told:
                runner.now = max(runner.now, self.t)
            call()
        elif runner.asking:
            runner.confirm()
        elif wake < self.due:
            self.t = max(self.t, wake, self.t + self.fine * (wake <= self.t))
            runner.tick(self.t)
        else:
            t, e = self.reading()
            q = self.also(t, e) if self.also is not None else None
            if self.latency > 0:  # (read now, heard of later: the laser as it was)
                self.events.append((t + self.latency, lambda: self._hear(runner, t + self.latency, e, q)))
            else:
                self._hear(runner, t, e, q)

    @staticmethod
    def _hear(runner, t, e, q):
        if not runner.busy:
            return
        if q is None:
            runner.on_reading(t, e)
        else:
            runner.on_reading(t, e, q)


def _wake(runner) -> float:
    """When the runner next wants to be woken (a pause, a wait, a ramp's next step): inf if it waits for nothing of
    the kind."""
    wake = getattr(runner, "wake", None)
    if wake is not None:
        return wake() if callable(wake) else wake
    if runner._waiting in ("pause", "wait"):  # (a Runner from before it said)
        return runner._until
    return math.inf


def holds(laser, v: float, i: float, reach: float = 1.5, fine: float = 0.02) -> tuple:
    """(mA down, mA up) a model's laser (sim.ModelLaser) can have its current moved from (v, i) before the mode it lases
    on hops, slowly (its chip as warm as its current): by its own rule, from the state it is in."""
    if hasattr(laser, "holds"):
        return laser.holds(v, i, reach, fine)
    if laser.lase(v, i) is None:  # (a ModelLaser from before it said)
        return math.nan, math.nan
    m, state = laser.model, laser._state
    now = laser.clock() if laser.clock is not None else 0.0
    t = m.t0 + (now - laser._began if laser.clock is not None else 0.0)
    T = laser.t_ref + laser.T if laser.t_ref is not None else None
    out = []
    for way in (-1.0, 1.0):
        st, got = (state[0], i, 0.0, i if m.kind == "current" else v, state[4]), math.inf
        for k in range(1, int(round(reach / fine)) + 1):
            _, _, st = m.step(i + way * k * fine, v, t, st, clock=1e6 * k, T=T)
            if st[0] != state[0]:
                got = (k - 1) * fine
                break
        out.append(got)
    return out[0], out[1]


# ------------------------------------------------------------------ the lasers
def far_ecdl(seed: int):
    """(laser, the nudge of its selection filter that makes it hop 1 or 2 modes, the wavemeter's noise MHz): a random
    ECDL, slightly off the middle of its mode."""
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


def strong_ecdl(seed: int):
    """(laser, target THz, noise): one with strong, skewed hysteresis, that has hopped."""
    rng = random.Random(4000 + seed)
    fsr = rng.uniform(3, 10)
    kv = rng.choice([-1, 1]) * rng.uniform(0.2, 0.8)
    laser = ECDL(400.0, fsr=fsr, kv=kv, ki=rng.uniform(-0.5, -0.1), ksv=kv * rng.uniform(0.6, 0.95),
                 ksi=rng.uniform(-4, -1), hysteresis=rng.uniform(0.15, 0.6) * fsr, skew=rng.uniform(-0.6, 0.6),
                 multimode=rng.uniform(0.02, 0.1) * fsr, seed=seed)
    laser.nudge(rng.uniform(-0.2, 0.2) * fsr)
    laser.m = 0
    target = laser.freq(V0, I0)
    lo, hi = laser.limits()
    a = laser.misalignment(0, V0, I0)
    laser.nudge((a - lo + rng.uniform(0.05, 0.5) * fsr) if rng.random() < 0.5 else -(hi - a + rng.uniform(0.05, 0.5) * fsr))
    laser.freq(V0, I0)
    return laser, target, rng.choice([0.3, 1.0, 3.0])


def near_ecdl(seed: int):
    """An SHG master whose next mode is close by (as the 739 nm one's, 4 pm): (laser, its random generator)."""
    rng = random.Random(7000 + seed)
    fsr = rng.uniform(1.8, 3.0)
    laser = ECDL(400.0, fsr=fsr, hysteresis=rng.uniform(0.1, 0.35) * fsr, multimode=rng.uniform(0.1, 0.25) * fsr,
                 ki=rng.uniform(-0.4, -0.15), ksi=rng.uniform(-3.0, -1.5), ksv=rng.uniform(0.28, 0.36),
                 mm_noise=0.004, seed=seed)
    return laser, rng


def lab493():
    """The lab's 493 nm DL pro as its map's model has it (rounded): lasermodel.LaserModel."""
    from . import lasermodel as lm

    return lm.LaserModel(
        f_ref=607.4237, i0=64.40, v0=67.16, t0=0.0,
        ext=lm.Comb(8.2619, p0=24.795, p_i=-1.7114, p_ii=-0.00285, p_v=1.2845, p_vv=0.00882, p_leg=-0.2757),
        chip=lm.Comb(87.379, p0=7.521, p_i=-4.186, p_ii=-0.0073, p_v=-0.1174, p_vv=-0.0005),
        lo=-88.76, hi=180.62, h_up=0.0006, h_down=0.0065, offset=0.712, pull=0.40, lag=0.298, w_v=0.2516,
        w_i=-0.1805, i_th=28.24, slope=0.512, power_dp=-0.57, right=0.77, step_s=0.252, step_x=0.2,
        i_span=[43.0, 87.0], v_span=[49.16, 89.16],
        alt={"lo": -88.48, "hi": 179.80, "h_up": 0.0001, "h_down": 0.0047, "offset": 0.606, "pull": 0.233,
             "lag": 0.537, "dc": 87.143, "ci": -4.186, "w_v": 0.607, "w_i": -0.724, "right": 0.767})


def lab650():
    """The lab's 650 nm DL pro likewise."""
    from . import lasermodel as lm

    return lm.LaserModel(
        f_ref=461.2938, i0=61.39, v0=66.99, t0=0.0,
        ext=lm.Comb(3.3886, p0=11.185, p_i=-0.2486, p_ii=0.00032, p_v=0.4018, p_vv=0.002, p_leg=-0.1196),
        chip=lm.Comb(18.373, p0=-5.236, p_i=-1.3615, p_ii=-0.0014, p_v=0.0149, p_vv=0.00013),
        lo=-9.06, hi=17.73, h_up=0.0203, h_down=0.0404, offset=0.086, pull=0.078, lag=0.041, w_v=0.6524,
        w_i=-0.1526, i_th=49.2, slope=75.4, power_dp=-22.2, right=0.85, step_s=0.084, step_x=0.2024,
        i_span=[52.9, 70.0], v_span=[36.8, 97.1], v_set_to_act=-0.5065,
        alt={"lo": -8.84, "hi": 18.85, "h_up": 0.0071, "h_down": 0.0141, "offset": -0.048, "pull": 0.154,
             "lag": 0.094, "dc": 18.431, "ci": -1.3833, "w_v": 0.6696, "w_i": -0.4538, "right": 0.8375})


def random_model(seed: int):
    """A random Littrow ECDL as a model has one (lasermodel.LaserModel): its external cavity 17 to 50 mm long (modes 3
    to 9 GHz apart), its chip's own modes a blue diode's (60 to 110 GHz apart) or a red one's (15 to 45 GHz), the
    current moving the chip's modes 2 to 5 times as fast as the external ones, a window one and a half to four chip modes
    wide that the piezo moves part of the way with the modes, hysteresis, pull and lag as the lab's two lasers' and
    between; its power as theirs (falling as a mode sits higher on its chip resonance)."""
    from . import lasermodel as lm

    rng = random.Random(9000 + seed)
    de = rng.uniform(3.0, 9.0)
    blue = rng.random() < 0.5
    dc = rng.uniform(60.0, 110.0) if blue else rng.uniform(15.0, 45.0)
    dc += 0.37 * de if abs(dc / de - round(dc / de)) < 0.12 else 0.0  # (a whole number of its modes: no vernier)
    e_i = -rng.uniform(0.2, 2.0)
    c_i = e_i * rng.uniform(2.0, 5.0)
    e_v = rng.choice([-1, 1]) * rng.uniform(0.3, 1.5)
    width = dc * rng.uniform(1.5, 4.0)
    lo = -width * rng.uniform(0.3, 0.7)
    h = math.exp(rng.uniform(math.log(0.0005), math.log(0.04)))
    i0, v0 = round(rng.uniform(50.0, 160.0), 2), round(rng.uniform(40.0, 100.0), 2)
    i_th = i0 - rng.uniform(20.0, 40.0)
    slope = rng.uniform(0.3, 2.0)
    model = lm.LaserModel(
        f_ref=rng.uniform(350.0, 750.0), i0=i0, v0=v0, t0=0.0,
        ext=lm.Comb(de, p0=rng.uniform(-de, de), p_i=e_i, p_ii=rng.uniform(-0.004, 0.001), p_v=e_v,
                    p_vv=0.006 * e_v * rng.uniform(0.0, 1.0), p_leg=-rng.uniform(0.0, 0.04) * de),
        chip=lm.Comb(dc, p0=rng.uniform(-dc / 2, dc / 2), p_i=c_i, p_ii=rng.uniform(-0.008, 0.0),
                     p_v=rng.uniform(-0.1, 0.1) * abs(e_v), p_vv=0.0),
        lo=lo, hi=lo + width, h_up=h * rng.uniform(0.1, 1.0), h_down=h, offset=rng.uniform(-0.5, 0.5) * dc,
        pull=rng.uniform(-0.05, 0.45), lag=rng.uniform(0.03, 0.3), w_v=e_v * rng.uniform(0.2, 1.0),
        w_i=e_i * rng.uniform(0.05, 0.5), soft=rng.uniform(5.0, 30.0), i_th=i_th, slope=slope,
        power_dp=-slope * (i0 - i_th) * rng.uniform(0.01, 0.05), right=0.85, step_s=0.2, step_x=0.2,
        i_span=[i0 - 10.0, i0 + 10.0], v_span=[v0 - 30.0, v0 + 30.0])
    if rng.random() < 0.5:  # (the rule held the other way, as a fit keeps it where it does nearly as well)
        model.alt = {"lo": model.lo, "hi": model.hi, "h_up": model.h_up * 0.3, "h_down": model.h_down * 0.6,
                     "offset": model.offset - 0.1, "pull": model.pull + 0.1, "lag": model.lag * 1.5, "dc": dc * 0.999,
                     "ci": c_i, "w_v": model.w_v, "w_i": model.w_i, "right": 0.83}
    return model


def two_modes(model, part: float):
    """The model with two modes at once over about `part` of each mode's range before it hops (LaserModel.mm: the
    margin, as a loss, within which the mode that would take over lases too): that part of the margin a mode has
    when it has just begun to lase, as the model itself has it sweeping its current slowly up and down about where
    it runs (the median over its modes)."""
    tops, state, top, order = [], None, 0.0, None
    span = [model.i0 + 3.0 * k / 150 - 3.0 for k in range(301)]
    for n, i in enumerate(span + span[::-1]):
        _, _, state = model.step(i, model.v0, model.t0, state, clock=50.0 * n)
        u = model.look(i, model.v0, model.t0, state)["u"]
        if state[0] != order:
            if order is not None and top > 0:
                tops.append(top)
            order, top = state[0], 0.0
        top = max(top, u)
    model.mm = part * statistics.median(tops) if tops else 0.0
    return model


def unlike(model, rng, spread: float = 1.0):
    """A laser like the model, not it (as another fit of its map would have it): its rule's terms a little off, its
    chip's modes anywhere, its cavity drifted. rng: numpy's. spread: how unlike (1: as fits of one map differ: its
    hysteresis by a factor e^0.4 either way, its pull by 0.05, its lag by e^0.3, its window's edges by 3 % of its
    width, how it moves by 15 %; 2.5: a model that is a poor likeness)."""
    import numpy as np

    k = spread
    true = copy.deepcopy(model)
    true.h_up *= float(np.exp(rng.normal(0, 0.4 * k)))
    true.h_down *= float(np.exp(rng.normal(0, 0.4 * k)))
    true.pull = float(np.clip(true.pull + rng.normal(0, 0.05 * k), -0.4, 0.8))
    true.lag *= float(np.exp(rng.normal(0, 0.3 * k)))
    w = true.hi - true.lo
    true.lo += float(rng.normal(0, 0.03 * k * w))
    true.hi += float(rng.normal(0, 0.03 * k * w))
    true.w_v *= float(1 + rng.normal(0, min(0.15 * k, 0.4)))
    true.w_i *= float(1 + rng.normal(0, min(0.15 * k, 0.4)))
    true.offset += float(rng.uniform(-0.5, 0.5) * true.dc)
    true.ext.p0 += float(rng.uniform(-5, 5))
    return true


# ------------------------------------------------------------------ recoveries
POLICIES = os.environ.get("WMLOCK_POLICIES") or os.path.join(tempfile.gettempdir(), "wmlock-policies")  # where the
#                         policies learned for the trials' lasers' models are kept (learned.policy_for)


@dataclass
class Case:
    """A laser that needs recovering: its rig, the settings and the arguments its Recenter is made with, and what
    'recovered' means for it."""
    kind: str
    seed: int
    rig: Rig
    p: HopParams
    args: tuple          # Recenter's, after p
    kwargs: dict
    hop: float = 0.0     # GHz it hopped by
    feasible: bool = True  # a mode that holds can be had on target within its windows, as the laser really is
    right: object = None   # () -> bool: on the mode it was on before it hopped (None: no telling)
    tol: float = 20.0    # MHz: on target within this


@dataclass
class Outcome:
    kind: str
    seed: int
    ok: bool             # the recovery said done
    on_target: bool      # ...and it is (within the case's tol; a cavity lock: locked on its peak)
    right: bool | None   # on the mode it was on before the hop
    margin: float        # mA to where its mode ends, as it really is (NaN: not done)
    second: float        # the part of its power in a second mode
    seconds: float       # s it took, simulated
    readings: int
    ops: int             # operations on the controller
    feasible: bool
    hop: float
    error: str = ""
    summary: str = ""
    method: str = ""
    notes: list = field(default_factory=list)


def _windows(p: HopParams, i_nom: float):
    return i_nom - p.i_below, i_nom + p.i_window, i_nom + max(p.jump_max, p.i_window)


def case_far(seed: int, **hop) -> Case:
    laser, nudge, noise = far_ecdl(seed)
    target = laser.freq(V0, I0)
    laser.nudge(nudge)
    laser.freq(V0, I0)
    p = HopParams(enabled=True, i_nom=I0, i_below=2.0, **hop)
    rig = Rig(laser, target, noise, seed=seed)
    lo, hi, top = _windows(p, I0)
    return Case("far", seed, rig, p, (I0, V0, lo, hi, V0 - p.v_window, V0 + p.v_window), {"i_top": top},
                hop=(laser.freq(V0, I0) - target) * 1e3, feasible=_best_margin(laser, p) >= 0.3,
                right=lambda: laser.m == 0, tol=5 + 3 * noise)


def case_strong(seed: int, **hop) -> Case:
    laser, target, noise = strong_ecdl(seed)
    p = HopParams(enabled=True, i_nom=I0, i_below=2.0, **hop)
    rig = Rig(laser, target, noise, seed=seed)
    lo, hi, top = _windows(p, I0)
    return Case("strong", seed, rig, p, (I0, V0, lo, hi, V0 - p.v_window, V0 + p.v_window), {"i_top": top},
                hop=(laser.freq(V0, I0) - target) * 1e3, feasible=_best_margin(laser, p) >= 0.3,
                right=lambda: laser.m == 0, tol=5 + 3 * noise)


def _best_margin(laser, p: HopParams) -> float:
    """The largest margin any operating point on target within the windows has (mA), as the ECDL really is."""
    best = -math.inf
    for m in range(-4, 5):
        for k in range(-100, 101):
            di = (p.i_window if k > 0 else p.i_below) * k / 100
            dv = -(m * laser.fsr + laser.ki * di) / laser.kv
            if abs(dv) <= p.v_window:
                a = m * laser.fsr + (laser.kv - laser.ksv) * dv + (laser.ki - laser.ksi) * di - laser.shift
                best = max(best, (laser.fsr / 2 - laser.hysteresis - abs(a)) / abs(laser.ki - laser.ksi))
    return best


def _locked(laser, seed: int, noise: float = 0.5, **kwargs):
    """A rig whose DLC pro has the laser locked to the cavity resonance at its target (as by hand)."""
    target = laser.freq(V0, I0)
    rig = Rig(laser, target, noise, seed=seed, scan=True)
    lock = rig.lock_it(seed=seed, **kwargs)
    lock.settle()
    if lock.has_shg:
        rig.shg_usual = lock._intra_now()
    return rig, lock


def case_shg(seed: int, method: str = "walk", **hop) -> Case:
    laser, nudge, noise = far_ecdl(1000 + seed)
    laser.multimode, laser.mm_noise = max(laser.multimode, 0.15 * laser.fsr), 0.0
    rig, lock = _locked(laser, seed, noise, shg=True, lockpoints=True, pdh_dc=random.Random(seed).uniform(-0.03, 0.03))
    laser.nudge(nudge)
    lock.frequency()
    extra = {"i_below": 2.0, "v_window": 10.0} if method == "map" else {}
    p = HopParams(enabled=True, i_nom=I0, cavity=True, shg=True, method=method, shg_lock=rig.shg_usual, **extra, **hop)
    lo, hi, top = _windows(p, I0)
    return Case("shg", seed, rig, p, (I0, V0, lo, hi, V0 - p.v_window, V0 + p.v_window),
                {"i_top": top, "cavity": True}, hop=rig.off_mhz() / 1e3, feasible=_best_margin(laser, p) >= 0.5,
                right=lambda: laser.m == 0, tol=1.0)


def case_near(seed: int, **hop) -> Case:
    laser, rng = near_ecdl(seed)
    rig, lock = _locked(laser, seed, shg=True, lockpoints=True, pdh_dc=rng.uniform(-0.03, 0.03))
    lo, hi = laser.limits()
    a, margin = laser.misalignment(laser.m, V0 + lock.dv, rig.act), rng.uniform(0.02, 0.3) * laser.fsr
    laser.nudge((a - lo + margin) if rng.random() < 0.5 else -(hi - a + margin))
    lock.frequency()
    mis = [laser.misalignment(0, V0, I0 + 0.01 * k) for k in range(-100, 201)]
    can = any(abs(x) < laser.zone(x)[0] for x in mis)  # the right mode single-mode on target in the window
    rig.noise = rng.uniform(0.3, 3.0)
    p = HopParams(enabled=True, i_nom=I0, cavity=True, shg=True, opt_i=I0, opt_v=V0, opt_shg=rig.shg_usual,
                  opt_t="2026-10-19 21:03", v_window=40.0, shg_lock=rig.shg_usual, **hop)
    lo, hi, top = _windows(p, I0)
    return Case("near", seed, rig, p, (I0, V0, lo, hi, V0 - p.v_window, V0 + p.v_window),
                {"i_top": top, "cavity": True}, hop=rig.off_mhz() / 1e3, feasible=can, right=lambda: laser.m == 0,
                tol=1.0)


def case_model(kind: str, seed: int, drift: float = 0.0004, spread: float = 1.0, sided: bool = False, **hop) -> Case:
    """A laser a model describes (kind: blue, red, or model: a random one), unlike it by seed (spread: how unlike:
    unlike()), on target where it lases coming down onto its nominal current; its chip's modes then move a little at
    a time until it hops. sided: the laser's hysteresis is by the edge of its mode it is at, not by the way its
    current last moved as the model has it (sim.ModelLaser): a laser that is not of the model's kind there."""
    import numpy as np

    model = {"blue": lab493, "red": lab650}[kind]() if kind in ("blue", "red") else random_model(seed)
    rng = np.random.default_rng(seed)
    true = unlike(model, rng, spread) if spread != 1.0 else unlike(model, rng)
    i_nom = round(float(model.i0 + rng.uniform(-1, 1)), 2)
    v_nom = round(float(model.v0 - model.v_set_to_act + rng.uniform(-3, 3)), 2)
    rig = None
    laser = (ModelLaser(true, clock=lambda: rig.t if rig is not None else 0.0, sided=True) if sided else
             ModelLaser(true, clock=lambda: rig.t if rig is not None else 0.0))
    rig = Rig(laser, 0.0, current=i_nom + 2.0, offset=v_nom, dt=0.15, seed=seed)
    for k in range(21):  # (onto it from 2 mA above, as by hand)
        rig.client.set("laser1:dl:cc:current-set", i_nom + 2.0 - 0.1 * k)
        rig.t += 0.15
    rig.target = target = laser.freq(v_nom, i_nom)
    right = laser._state[0]
    for _ in range(20000):
        if abs(laser.freq(v_nom, i_nom) - target) * 1e3 >= 0.4 * true.ext.spacing:
            break
        true.offset += drift * true.dc
        rig.t += 0.15
        rig._settle()
    rig.due = rig.t + rig.dt
    hop = dict(hop)
    p = HopParams(enabled=True, method=hop.pop("method", "model"), i_nom=i_nom, i_below=1.0, i_window=2.0,
                  v_window=25.0, **hop)
    more = {}
    if p.method == "learned":  # (a policy learned for the model the program has of it: made now, if it is not kept)
        from . import learned

        more["policy"] = learned.policy_for(model, POLICIES)
    return Case(kind, seed, rig, p, (i_nom, v_nom, i_nom - 1.0, i_nom + 2.0, v_nom - 25.0, v_nom + 25.0),
                {"ref": (v_nom, i_nom), "model": model, "target": target, **more},
                hop=(laser.freq(v_nom, i_nom) - target) * 1e3, right=lambda: laser._state[0] == right, tol=20.0)


def make_case(kind: str, seed: int, spread: float = 1.0, sided: bool = False, **hop) -> Case:
    """spread, sided: a model's laser less like its model (case_model); the others' are as they come."""
    if kind == "far":
        return case_far(seed, **hop)
    if kind == "strong":
        return case_strong(seed, **hop)
    if kind == "shg":
        return case_shg(seed, **hop)
    if kind == "near":
        return case_near(seed, **hop)
    if kind in ("blue", "red", "model"):
        return case_model(kind, seed, spread=spread, sided=sided, **hop)
    raise ValueError(f"no such laser: {kind!r} (one of {', '.join(KINDS)})")


def recover(case: Case) -> Outcome:
    """Run the case's recovery to its end: what came of it."""
    rig = case.rig
    t0, r0, o0 = rig.t, rig.reads, rig.ops
    rec = Recenter(case.p, *case.args, **case.kwargs)
    runner = rig.run(rec)
    ok = runner.state == "done"
    off = rig.off_mhz()
    on = ok and math.isfinite(off) and abs(off) <= case.tol and (rig.lock is None or rig.lock.locked())
    return Outcome(case.kind, case.seed, ok, on, (bool(case.right()) if case.right is not None else None) if ok else None,
                   rig.margin() if ok else math.nan, rig.second_mode() if ok else math.nan, rig.t - t0,
                   rig.reads - r0, rig.ops - o0, case.feasible, case.hop, runner.error or "",
                   runner.result.summary if ok else "", rec.method, list(rec.notes))


def recoveries(kind: str, n: int, first: int = 0, say=None, rig: dict | None = None, **hop) -> list:
    """n cases of that kind (seeds first ..), each recovered: their outcomes. rig: its rigs' own (Rig's attributes:
    latency, op_s, dt; noise_x: the wavemeter's noise that many times what it is), if not as they come."""
    out = []
    for seed in range(first, first + n):
        case = make_case(kind, seed, **hop)
        for name, value in (rig or {}).items():
            if name == "noise_x":
                case.tol += 3 * case.rig.noise * max(value - 1, 0)  # (on target: within what its readings tell)
                case.rig.noise *= value
            elif value is not None:
                setattr(case.rig, name, value)
        case.rig.due = case.rig.t + case.rig.dt
        o = recover(case)
        out.append(o)
        if say is not None:
            say(o)
    return out


def _q(xs, q: float) -> float:
    xs = sorted(xs)
    return xs[min(int(q * len(xs)), len(xs) - 1)] if xs else math.nan


def summary(outs: list) -> dict:
    """What a set of outcomes comes to: how many done and on target, of those that could be; on the right mode; the
    time they took (median, nine in ten, the longest), their readings, their margins."""
    done = [o for o in outs if o.on_target]
    can = [o for o in outs if o.feasible]
    times = [o.seconds for o in done]
    right = [o for o in done if o.right is not None]
    return {
        "n": len(outs), "feasible": len(can), "done": len(done), "done_feasible": sum(o.on_target for o in can),
        "said_done_off_target": sum(o.ok and not o.on_target for o in outs),
        "right_mode": sum(bool(o.right) for o in right), "right_known": len(right),
        "s_median": round(statistics.median(times), 1) if times else None,
        "s_p90": round(_q(times, 0.9), 1) if times else None, "s_max": round(max(times), 1) if times else None,
        "s_mean": round(statistics.fmean(times), 1) if times else None,
        "readings_median": statistics.median([o.readings for o in done]) if done else None,
        "margin_median": round(statistics.median([o.margin for o in done]), 2) if done else None,
        "margin_p10": round(_q([o.margin for o in done], 0.1), 2) if done else None,
        "second_mode_over_10pc": sum(o.second > 0.1 for o in done),
    }


def said(kind: str, s: dict) -> str:
    """A summary, as a line."""
    if not s["done"]:
        return f"{kind:7s} {s['n']:4d} lasers: none recovered"
    right = f", {s['right_mode']} of {s['right_known']} on their own mode" if s["right_known"] else ""
    return (f"{kind:7s} {s['n']:4d} lasers: {s['done_feasible']} of the {s['feasible']} that could be recovered"
            f" ({s['done']} in all{right}); {s['s_median']:g} s median, {s['s_p90']:g} s for nine in ten, "
            f"{s['s_max']:g} s at most; {s['readings_median']:g} readings; margin {s['margin_median']:g} mA median, "
            f"{s['margin_p10']:g} mA for the worst tenth"
            + (f"; {s['said_done_off_target']} SAID DONE BUT OFF TARGET" if s["said_done_off_target"] else ""))


# --------------------------------------------------------------------- keeping
KEEPS = ("shg", "dl", "blue", "red")


@dataclass
class Kept:
    """A laser locked and drifting towards a hop for so many hours, kept off the edge of its mode or not: when it
    hopped (h; None: it did not), how often it was kept (its current trimmed, its signal back up), how often a
    keeping did not mend it, where its current ended (mA from where it began), and whether that was the edge of its
    window (no further to go: its temperature, or its nominal current, wants adjusting)."""
    kind: str
    seed: int
    hours: float
    hop: float | None
    kept: int
    unmended: int
    failed: int
    moved: float
    at_edge: bool
    drift: float           # GHz its selection (its chip's modes) swings each way (GHz/h, drifting steadily)
    off_rms: float         # MHz rms from its target all the while, as it really was (not as two modes read)...
    off_max: float = 0.0   # ...and the furthest
    notes: list = field(default_factory=list)


def keep_trial(kind: str, seed: int, hours: float = 6.0, keep: bool = True, rate: float | None = None,
               zone: float | None = None, dt: float = 0.5, noise: float = 1.0) -> Kept:
    """One laser, locked, its selection drifting to and fro over hours, further each way than where it hops (rate:
    steadily instead, GHz/h), and wandering a little: left alone it hops within an hour or two. Watched
    (keep.ModeWatch) and kept (Recenter.keep) as the engine would, or left alone:
      shg   an SHG master whose next mode is close by (near), its DLC pro locking it to its cavity and its SHG cavity
            to it: its SHG signal is what is watched
      dl    an ECDL with nothing but the wavemeter, locked to it by this program's lock (which goes on during a
            keeping): the wavemeter's interference patterns are what is watched (their fringes fading)
      blue, red  the lab's two DL pros as their models have them (a model's laser: its chip's modes drifting; two
            modes over a fifth of each mode's range before a hop; its hysteresis by the edge of its mode it is at:
            ModelLaser's sided), locked to the wavemeter likewise.
    What the keeping rests on can be taken away, to see what is left of it: zone: the part of a mode's range that
    runs on two modes before it hops (None: as above; 0: none: it hops out of the blue); dt: s between the
    wavemeter's readings (and the polls of its SHG signal); noise: its signal's noise, that many times what it is."""
    from .keep import ModeWatch
    from .lock import Lock, LockParams
    from .sim import SimCCD

    rng = random.Random(31000 + seed)
    watch = ModeWatch()
    locked, ccd, wm_lock, model = None, None, None, kind in ("blue", "red")
    if kind == "shg":
        laser, _ = near_ecdl(seed)
        if zone is not None:
            laser.multimode = zone * laser.fsr
        rig, locked = _locked(laser, seed, shg=True, lockpoints=True, pdh_dc=rng.uniform(-0.03, 0.03))
        locked.shg_slope = rng.uniform(-0.015, 0.015)  # (its signal goes a little with its master's current, too)
        locked.shg_noise *= noise
        p = HopParams(enabled=True, i_nom=I0, cavity=True, shg=True, opt_i=I0, opt_v=V0, opt_shg=rig.shg_usual,
                      shg_lock=rig.shg_usual, keep=keep, i_below=2.0)
        watch.signals["shg"].prior = rig.shg_usual
        rig.dt = dt  # (its SHG signal polled twice a second, as the engine does; the wavemeter as often)
        i_nom, v_nom = I0, V0
    elif kind == "dl":
        laser, _, noise = far_ecdl(2000 + seed)
        laser.shift, laser.m = 0.0, 0
        laser.multimode = rng.uniform(0.1, 0.25) * laser.fsr if zone is None else zone * laser.fsr
        rig = Rig(laser, laser.freq(V0, I0), noise, seed=seed, dt=dt)
        p = HopParams(enabled=True, i_nom=I0, keep=keep, i_below=2.0)
        i_nom, v_nom = I0, V0
    elif model:
        true = two_modes(unlike({"blue": lab493, "red": lab650}[kind](), __import__("numpy").random.default_rng(seed)),
                         0.2 if zone is None else zone)
        i_nom = round(true.i0 + rng.uniform(-1, 1), 2)
        v_nom = round(true.v0 - true.v_set_to_act + rng.uniform(-3, 3), 2)
        rig = None
        laser = ModelLaser(true, clock=lambda: rig.t if rig is not None else 0.0, seed=seed, sided=True)
        rig = Rig(laser, 0.0, current=i_nom + 2.0, offset=v_nom, dt=dt, seed=seed)
        for k in range(21):  # (onto its mode from 2 mA above, as by hand; then into the middle of where it holds)
            rig.client.set("laser1:dl:cc:current-set", i_nom + 2.0 - 0.1 * k)
            rig.t += 0.15
        for _ in range(3):
            down, up = holds(laser, v_nom, rig.act, reach=3.0)
            to = rig.act + (0.5 * (min(up, 3.0) - min(down, 3.0)) if math.isfinite(down + up) or up != down else 0.0)
            while abs(rig.act - to) > 1e-6:  # (slowly: its chip as warm as its current)
                rig.client.set("laser1:dl:cc:current-set", rig.act + max(-0.02, min(0.02, to - rig.act)))
                rig.t += 0.5
        for _ in range(20):  # (at rest there)
            rig.t += 0.5
            rig._settle()
        i_nom = round(rig.act, 3)
        rig.target = laser.f0 + laser.lase(v_nom, rig.act) * 1e-3
        p = HopParams(enabled=True, i_nom=i_nom, keep=keep, i_below=2.0)
    else:
        raise ValueError(f"no such laser to keep: {kind!r} (one of {', '.join(KEEPS)})")
    if zone == 0 and isinstance(laser, ECDL):  # (never two modes at once: it hops out of the blue)
        laser.components = lambda v, i: [(1.0, laser.mode_ghz(v, i))] if laser.lasing else []
        laser.mm_noise = 0.0
    lo, hi, top = _windows(p, i_nom)
    if locked is None:  # this program's wavemeter lock, on its piezo; the wavemeter's patterns of it
        ccd = SimCCD(seed=seed, amp_noise=3.0 * noise)
        if noise != 1.0:  # (its CCD's noise, a pixel's)
            ccd._noise = [ccd.rng.gauss(0.0, 2.0 * noise) for _ in range(ccd.PIXELS)]
        kv = laser.kv if isinstance(laser, ECDL) else laser.model.ext.p_v
        wm_lock = Lock(LockParams(ki=0.3 / kv, excursion=0.0))
        wm_lock.engage(rig.V)
    # how its selection drifts (an ECDL's filter, a model's chip modes): to and fro (the room's temperature, the air's
    # pressure: a few hours a cycle), further each way than where it hops (a mode's half and its hysteresis away), and
    # wandering a little besides; or steadily, `rate` GHz/h
    reach = laser.limits()[1] if isinstance(laser, ECDL) else 0.5 * laser.model.ext.spacing
    swing, period, phase = rng.uniform(1.2, 1.8) * reach, rng.uniform(3.0, 6.0) * 3600, rng.uniform(-0.3, 0.3)
    sign = rng.choice([-1, 1])

    def drifted(t):  # GHz, by t s
        if rate is not None:
            return rate * t / 3600
        return sign * swing * (math.sin(2 * math.pi * t / period + phase) - math.sin(phase))

    state = {"sq": 0.0, "n": 0, "max": 0.0}

    def really():  # (where it is, as it really is: MHz from its target; kept while it is on its mode: not hopped)
        off = rig.off_mhz()
        on = (laser.m if isinstance(laser, ECDL) else laser._state[0]) == right
        if math.isfinite(off) and on and (locked is None or locked.locked()):
            state["sq"], state["n"], state["max"] = state["sq"] + off * off, state["n"] + 1, max(state["max"],
                                                                                              abs(off))
        return off

    def fringes(t):
        v = rig.V
        comps = laser.components(v, rig.act)
        _, extra, _ = ccd.measure(comps, laser.power(rig.act), rig.act, rig.target, laser.f0, True)
        return watch.fringes.seen(extra["patterns"][0][1])

    def also(t, e):  # (what the engine goes on doing with each reading while a keeping runs)
        really()
        if wm_lock is None:
            return None
        if math.isfinite(e) and abs(e) <= p.tolerance:
            u = wm_lock.update(e)
            if u is not None:
                rig.set(offset=u)
        return fringes(t)

    rig.also = also
    t0, t_end = rig.t, rig.t + hours * 3600
    out = Kept(kind, seed, hours, None, 0, 0, 0, 0.0, False, rate if rate is not None else sign * swing, 0.0)
    right = laser.m if isinstance(laser, ECDL) else laser._state[0]
    was, thin = 0.0, max(1, round(2.0 / rig.dt))
    while rig.t < t_end:
        now = drifted(rig.t - t0 + rig.dt)
        step, was = now - was + rng.gauss(0.0, 0.002), now
        if isinstance(laser, ECDL):
            laser.nudge(step)
        else:
            laser.model.offset += step
        if locked is not None:
            rig.t += rig.dt
            locked.frequency(0.0)
            off = really()
            hopped = not locked.locked() or laser.m != right or abs(off) > p.peak_tol
            watch.feed("shg", rig.t, locked._intra_now())
        else:
            rig._settle()
            t, e = rig.reading()
            state["k"] = state.get("k", 0) + 1
            if state["k"] % thin == 0:  # (its patterns looked at every 2 s here: the simulated CCD's take a while)
                q = also(t, e)
                if q is not None:
                    watch.feed("fringes", t, q)
            else:
                really()
                if math.isfinite(e) and abs(e) <= p.tolerance:
                    u = wm_lock.update(e)
                    if u is not None:
                        rig.set(offset=u)
            now = laser.m if isinstance(laser, ECDL) else laser._state[0]
            hopped = now != right
        if hopped:
            out.hop = (rig.t - t0) / 3600
            break
        alarm = watch.alarm(rig.t, p.keep_dip) if keep else None
        if alarm:
            rec = Recenter(p, rig.I, rig.V, lo, hi, v_nom - p.v_window, v_nom + p.v_window, cavity=locked is not None,
                           i_top=top, keep=alarm)
            rig.due = rig.t + rig.dt
            r = rig.run(rec)
            k = rec.kept or {}
            done = r.state == "done"
            mended = done and bool(k.get("mended"))
            out.kept += mended and bool(k.get("way"))
            out.unmended += done and not mended
            out.failed += not done
            out.notes.append(f"{(rig.t - t0) / 3600:.2f} h: " + (r.result.summary if done else r.error))
            watch.kept(rig.t, mended, k.get("way", 0), rebase=done and not mended and not k.get("edge"),
                       level=k.get("level"))
    out.moved = rig.act - i_nom
    out.at_edge = rig.I <= lo + 0.1 or rig.I >= hi - 0.1
    out.off_rms = math.sqrt(state["sq"] / state["n"]) if state["n"] else 0.0
    out.off_max = state["max"]
    return out


def keepings(kind: str, n: int, hours: float = 6.0, first: int = 0, keep: bool = True, say=None, **how) -> list:
    out = []
    for seed in range(first, first + n):
        k = keep_trial(kind, seed, hours, keep, **how)
        out.append(k)
        if say is not None:
            say(k)
    return out


def kept_said(kind: str, outs: list, keep: bool) -> str:
    n, hops = len(outs), [o for o in outs if o.hop is not None]
    edge = [o for o in hops if o.at_edge]
    text = (f"{kind:5s} {'kept' if keep else 'left alone':10s} {n} lasers, {outs[0].hours:g} h each: "
            f"{len(hops)} hopped")
    if hops:
        text += f" (the first after {min(o.hop for o in hops):.2f} h, half of them by {_q([o.hop for o in hops], 0.5):.2f} h)"
    if keep:
        text += (f"; {len(edge)} of those with the current at the edge of its window (nowhere to go); kept "
                 f"{sum(o.kept for o in outs) / n:.1f} times a laser, {sum(o.unmended for o in outs)} keepings that "
                 f"did not mend it, {sum(o.failed for o in outs)} that failed; the current moved "
                 f"{statistics.median([abs(o.moved) for o in outs]):.2f} mA (median); before any hop, "
                 f"{statistics.median([o.off_rms for o in outs]):.1f} MHz rms from their targets (median), "
                 f"{max(o.off_max for o in outs):.0f} MHz at the worst")
    return text


# ------------------------------------------------------------------------ main
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m wmlock.trials", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="what", required=True)
    rc = sub.add_parser("recover", help="lasers that hopped, recovered")
    rc.add_argument("kind", choices=KINDS + ("all",))
    rc.add_argument("--n", type=int, default=40)
    rc.add_argument("--first", type=int, default=0, help="the first seed")
    rc.add_argument("--json", help="write every outcome and the summaries there")
    rc.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="a mode-hop recovery setting (HopParams), as method=map")
    rc.add_argument("--latency", type=float, default=0.0, help="s a reading is old when it comes (0)")
    rc.add_argument("--op", type=float, default=0.002, help="s an operation on its controller takes (0.002)")
    rc.add_argument("--dt", type=float, help="s between the wavemeter's readings (0.1; the model's lasers 0.15)")
    rc.add_argument("--noise", type=float, default=1.0, help="the wavemeter's noise, times what it is")
    rc.add_argument("--unlike", type=float, default=1.0, help="a model's laser this much less like its model (1)")
    rc.add_argument("--sided", action="store_true", help="a model's laser whose hysteresis is by the edge it is at")
    rc.add_argument("-v", "--verbose", action="store_true", help="each laser as it goes")
    kp = sub.add_parser("keep", help="lasers locked and drifting towards a hop: kept off the edge of their mode, or not")
    kp.add_argument("kind", choices=KEEPS + ("all",))
    kp.add_argument("--n", type=int, default=20)
    kp.add_argument("--first", type=int, default=0)
    kp.add_argument("--hours", type=float, default=6.0)
    kp.add_argument("--zone", type=float, help="the part of a mode's range on two modes before a hop (0: none)")
    kp.add_argument("--dt", type=float, default=0.5, help="s between readings (0.5)")
    kp.add_argument("--noise", type=float, default=1.0, help="its signal's noise, times what it is")
    kp.add_argument("--json")
    kp.add_argument("-v", "--verbose", action="store_true")
    ls = sub.add_parser("lasers", help="what the random lasers are")
    ls.add_argument("kind", choices=KINDS)
    ls.add_argument("--n", type=int, default=5)
    ls.add_argument("--first", type=int, default=0)
    args = ap.parse_args(argv)
    if args.what == "lasers":
        for seed in range(args.first, args.first + args.n):
            c = make_case(args.kind, seed)
            las = c.rig.laser
            if isinstance(las, ECDL):
                print(f"{args.kind} {seed}: modes {las.fsr:.2f} GHz apart, piezo {las.kv:+.2f} GHz/V (its selection "
                      f"{las.ksv:+.2f}), current {las.ki:+.2f} GHz/mA (its selection {las.ksi:+.2f}), hysteresis "
                      f"{las.hysteresis:.2f} GHz, two modes over {las.multimode:.2f} GHz; hopped {c.hop:+.2f} GHz"
                      + ("" if c.feasible else " (no mode that holds within its windows)"))
            else:
                print(f"{args.kind} {seed}: hopped {c.hop:+.2f} GHz; " + "; ".join(c.kwargs["model"].facts()[:3]))
        return 0
    if args.what == "keep":
        every = {}
        for kind in (KEEPS if args.kind == "all" else (args.kind,)):
            for keep in (False, True):
                def say(k):
                    if args.verbose and keep:
                        print(f"  {k.kind} {k.seed}: swing {k.drift:+.2f} GHz, "
                              + (f"hopped after {k.hop:.2f} h" if k.hop is not None else "no hop")
                              + f", kept {k.kept} times, current {k.moved:+.2f} mA" + (" (the window's edge)"
                                                                                       if k.at_edge else ""), flush=True)
                outs = keepings(kind, args.n, args.hours, args.first, keep, say, zone=args.zone, dt=args.dt,
                                noise=args.noise)
                every[f"{kind} {'kept' if keep else 'alone'}"] = outs
                print(kept_said(kind, outs, keep), flush=True)
        if args.json:
            with open(args.json, "w", encoding="utf-8") as f:
                json.dump({k: [asdict(o) for o in v] for k, v in every.items()}, f, indent=1)
        return 0
    hop = {}
    for item in args.set:
        name, _, value = item.partition("=")
        was = getattr(HopParams(), name, None)
        if was is None:
            ap.error(f"no such setting: {name}")
        hop[name] = (value.strip().lower() in ("1", "true", "yes", "on") if isinstance(was, bool)
                     else type(was)(value))
    kinds = KINDS if args.kind == "all" else (args.kind,)
    every, t0 = {}, time.monotonic()
    for kind in kinds:
        def say(o):
            if args.verbose:
                print(f"  {o.kind} {o.seed}: {'on target' if o.on_target else 'NOT: ' + (o.error or 'off target')}, "
                      f"{o.seconds:.1f} s, {o.readings} readings" + (f", margin {o.margin:.2f} mA" if o.ok else ""),
                      flush=True)
        how = {"spread": args.unlike, "sided": args.sided} if kind in ("blue", "red", "model") else {}
        outs = recoveries(kind, args.n, args.first, say, {"latency": args.latency, "op_s": args.op, "dt": args.dt,
                                                         "noise_x": args.noise}, **how, **hop)
        every[kind] = outs
        print(said(kind, summary(outs)), flush=True)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({"took_s": round(time.monotonic() - t0, 1), "set": hop, "latency": args.latency, "op_s": args.op,
                       "dt": args.dt, "noise": args.noise, "unlike": args.unlike, "sided": args.sided,
                       "summary": {k: summary(v) for k, v in every.items()},
                       "outcomes": {k: [asdict(o) for o in v] for k, v in every.items()}}, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
