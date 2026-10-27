"""Engine: glue between the wavemeter source, the locks and the DLC pros (Qt-free).

Threads
  source       polls the wavemeter and calls on_readings(); the lock maths runs
               right there (microseconds) and the new offset is handed to the
               controller of that laser
  controllers  one per DLC pro: owns the TCP connection, coalesces offset writes
               (latest value wins), runs one-off jobs, polls offsets for display
  discovery    UDP broadcast + parallel probes, on demand
The GUI reads plain attributes on a timer and calls the public methods, which
only take a lock briefly and enqueue work: it never waits on the network.

With a reference laser (reference.py), every reading is corrected for the
wavemeter's drift before anything else sees it: channel.freq is corrected,
channel.raw is what the wavemeter said.

Mode-hop recovery (modehop.py, relock.py), per channel: a Guard watches the
readings of a laser that is locked, by this program to the wavemeter or by its
DLC pro to a cavity, and after a hop a Recenter procedure takes the laser over
(current, piezo, its locks and scans, through the controller thread) and hands
it back to its lock. What each recovery saw and did is kept (Channel.last) and
appended to a log file.

A laser its DLC pro locks to a cavity (HopParams.cavity) is never locked to the
wavemeter: engaging it ('Relock') only arms the guard, which watches it and
relocks it after a hop; and if it is not locked on target then, relocks it now.

Whether it lases: with a TA, its seed power is polled too (HopParams.seed), and
an armed laser that stops lasing is recovered at once, the hourly limit
notwithstanding: the recovery jumps its current up first (modehop.Recenter.panic).
The seed power while it lases as usual is learned (HopParams.seed_ok).

A laser its DLC pro locks to a cavity is relocked, besides after a hop, when it
is locked more than HopParams.peak_tol from its target (the wrong peak: the
wavemeter is never that far off), and when the lock keeps losing it and
relocking: its transmission (the input its lock detection watches, polled while
its lock is on) averages, over LEVEL_S, below lock_min of the way from its zero
(off resonance, measured while the master scans in a relock) to its level locked
(HopParams.trans_zero, trans_locked). After a relock, its target (the setpoint)
is moved to where the wavemeter puts it, locked. With an SHG stage, its
intra-cavity signal averaging below lock_min of usual over LEVEL_S (the SHG
cavity lost, or the laser not lasing: a broadband glow fools a seed photodiode,
not a cavity) starts a recovery too. Either counts towards the hourly limit only
if the wavemeter still reads the laser (else it most likely stopped lasing). The
levels locked, of the transmission and the intra-cavity signal, are the medians
of what they were over the last few minutes while it ran as it should
(HopParams.trans_locked, shg_lock), and what a relock measured until then.

Laser maps (lasermap.py): a channel's laser, unlocked, can be mapped (start_map): its current and piezo swept all
over, both ways, everything recorded in a file. Its state is 'mapping' meanwhile: its readings go to the map
(MapRunner, through the laser's controller as a recovery's do), in detail (Source.detail), and nothing else
touches it; stopped (stop_map, or the program closing), it goes back where it began. Once its file is written, a
thread of its own fits the laser's model to it (lasermodel.py, a minute or two; the laser is free meanwhile): kept
in the file, its facts said in the map's notes (Mapper.model).

Lock dropouts (dropouts.py): for each laser its DLC pro locks to a cavity, a monitor of its own watches the
transmission for the lock losing the laser even for a couple of milliseconds (HopParams.drop_ms between
samples), while its lock is on and no recovery runs (and DROP_SETTLE_S after); each dropout is logged (the
console, and a line of JSON in drop_path), and how many there were, once an hour.

Currents are shown, and the nominal current given, as measured (current-act);
the set current (current-set) is what a recovery writes, and is watched for
changes by hand.

A laser an SLS unit locks (sls.py; Channel.sls): its unit polled by a watch of its own (SlsWatch), its lock's
dropouts logged as the dropout monitor's are; armed (Relock), a slave an SLS PLL's manual lock beat-locks to a master
is guarded (sls.Guard): its PLL released the moment its master's lock or its beat is lost, the slave recovered if it
stopped lasing (its mode-hop recovery), brought to its beat point by the wavemeter lock (steering, Channel.steering)
once its master holds, and its PLL engaged again. Meanwhile its state is 'recovering'. Its setpoint is its beat point:
its master's reading plus the beat (as learned, its PLL holding it).

A wavemeter channel switched off ('Use' unticked in the WLM software, by someone
who wants a faster rate on theirs) gives no readings; the source can switch it
on again. A laser its DLC pro locks to a cavity stays watched meanwhile, by what
its DLC pro says: its lock's state, its transmission, its SHG cavity's lock and
signal, its seed power. When any of that says something happened, its channel
is switched on for a look (Engine.LOOK_S at most, LOOK_GAP apart at least):
locked on target, it is switched off again; else the laser is relocked. Any
recovery switches the channel on first, waits for readings (WAIT_S at most; not
for a laser that stopped lasing: its current jump does not wait), and switches
it off again when it is over; switched off during one, it is switched on again
at once. A laser this program locks to the wavemeter is held where it is (its
lock goes on when readings do). A channel the switch stops measuring for
another reason (not cycling) unlocks as before.
"""
from __future__ import annotations

import dataclasses
import json
import logging
import math
import sqlite3
import statistics
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from . import dac
from .dlcpro import SCAN_OUT, DLCPro, DecopError, Device, discover, drives, output_name, probe, scan_output
from .dropouts import Monitor as DropoutMonitor
from .dropouts import log as drop_log
from .lasermap import FOLDER, LOOK, MODEL_MIN, MapFile, MapPlan, Mapper, MapRunner, file_name, fmt_time
from .lock import Lock, LockParams
from .modehop import SEED_FLOOR, Guard, HopParams, Recenter, Runner, perform
from .observe import Observation
from .relock import TRANS, says_locked, setup_report, transmission_input, transmission_signal, usable_zero, volts
from .reference import Reference, RefParams
from .sls import Guard as BeatGuard
from .sls import Look, SlsParams, SlsWatch
from .wavemeter import C_NM_THZ, NAN

log = logging.getLogger(__name__)
meter_log = logging.getLogger("wmlock.meters")  # (which wavemeter each one's settings are for: said at INFO)

ACTUATOR = "scan:offset"  # the DLC pro parameter the lock writes, below laserN: (where its scan drives the piezo, as
CURRENT = "dl:cc:current-set"  # it does on most; else Channel.piezo, and the current Channel.cc: dlcpro.drives)
CURRENT_ACT = "dl:cc:current-act"
CAVITY_LOCK = "dl:lock:lock-enabled"
SHG_STATE = "nlo:shg:lock:state-txt"
SHG_INTRA = "nlo:pd:shg-int:photodiode"
WATCHED = (CURRENT_ACT, CAVITY_LOCK, "dl:lock:state-txt", SHG_STATE, SHG_INTRA, SCAN_OUT)  # (and its piezo, current)
FINE_N = 600   # polls: a level locked as usual is the median of the last this many running as it should...
FINE_MIN = 10  # ...once there are this many (until then, the one learned before)
OBSERVING = ("observe", "observe_end", "observed")  # remote(): a channel's lock observed, nothing changed
OBSERVED_PART = 20  # [began, s] at most, of an observation's times asked from another PC at a time (one datagram)
BEAT_N = 200          # readings its beat is learned from (their median)
STEADY_MHZ = 3.0      # MHz: a reading this far from the one before, its laser was not steady
DAC_LOCK = dict(kp=0.0, ki=0.01, max_step=0.02, v_min=-1.2, v_max=1.2, excursion=0.8)  # a laser on an analog output:
#                Epimetheus's limits (±1.2 V; its PID's output ±0.8 V), a gentle start for the gain


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def round_sig(x: float, digits: int = 9) -> float:
    if not x or not math.isfinite(x):
        return x
    return round(x, digits - 1 - math.floor(math.log10(abs(x))))


def step_fit(before, after) -> tuple:
    """Two windows of readings ((t, y) pairs) either side of a step: (how far a straight line through `after` sits
    from one through `before`, the two sharing their slope (a drift, taken off: what the step did by itself), its
    uncertainty, that slope)."""
    sides = []
    for pts in (before, after):
        t = statistics.fmean(t for t, _ in pts)
        y = statistics.fmean(y for _, y in pts)
        sides.append((len(pts), t, y, [(tk - t, yk - y) for tk, yk in pts]))
    sxx = sum(dt * dt for *_, d in sides for dt, _ in d)
    sxy = sum(dt * dy for *_, d in sides for dt, dy in d)
    syy = sum(dy * dy for *_, d in sides for _, dy in d)
    (n0, t0, y0, _), (n1, t1, y1, _) = sides
    b = sxy / sxx if sxx > 0 else 0.0
    dof = n0 + n1 - 3
    s2 = max(syy - b * sxy, 0.0) / dof if dof > 0 else math.inf
    var = s2 * (1 / n0 + 1 / n1 + ((t1 - t0) ** 2 / sxx if sxx > 0 else 0.0))
    return (y1 - y0) - b * (t1 - t0), math.sqrt(var), b


class Track:
    """Another laser's readings as followed (a beat lock's master: its reading, on any PC): the last few, and since when
    they have been steady (no jump of STEADY_MHZ)."""

    def __init__(self, n: int = 3):
        self.f = deque(maxlen=n)  # THz
        self.t = None             # monotonic: when the last was read
        self.steady = None        # monotonic: steady since

    def feed(self, f, t):
        """Its reading f (THz; None: none) as read at t (monotonic); one fed again is left out."""
        if t is None or (self.t is not None and t <= self.t + 1e-4):
            return
        if f is None:
            self.f.clear()
            self.steady = None
        else:
            if not self.f or abs(f - self.f[-1]) * 1e6 > STEADY_MHZ:
                self.steady = t
            self.f.append(f)
        self.t = t

    def point(self, now: float, fresh: float):
        """THz: the median of its last few readings, if the last is at most fresh s old (else None)."""
        if not self.f or self.t is None or now - self.t > fresh:
            return None
        return statistics.median(self.f)

    def steady_s(self, now: float, fresh: float):
        """s its readings have been steady, None if its last is older than fresh s (or there is none)."""
        if self.steady is None or self.t is None or now - self.t > fresh:
            return None
        return now - self.steady


class Channel:
    """Settings and live state of one wavemeter channel."""

    def __init__(self, n: int, cfg: dict | None = None):
        cfg = cfg or {}
        self.n = n
        self.name = str(cfg.get("name", ""))
        self.hidden = bool(cfg.get("hidden", False))
        self.setpoint_nm = None
        self.f_set = NAN
        self.set_setpoint(cfg.get("setpoint_nm"))
        link = cfg.get("link") or {}
        self.device = link.get("device") or None  # controller key: 'ip' or 'ip:port'; an analog output's: 'dac:mcc:0'
        self.laser = int(link.get("ao", link.get("laser", 1)))  # laserN of its DLC pro; on a DAC, the output's number
        ff = link.get("ff_ao")
        self.ff_ao = ff if isinstance(ff, int) and not isinstance(ff, bool) else None  # on a DAC: a second output, fed
        self.ff_ratio = float(link.get("ff_ratio") or 0.0) if self.ff_ao is not None else 0.0  # the moves of the first
        #                                                                               divided by this (Epimetheus's)
        self.ff0 = None       # V: that output where the lock engaged
        self.lock = Lock(LockParams.from_dict(cfg.get("lock")))
        self.hop = HopParams.from_dict(cfg.get("hop"))
        # live state, written by engine threads and read by the GUI
        self.freq = NAN       # THz, corrected by the reference if there is one; NaN if invalid
        self.raw = NAN        # THz, as the wavemeter read it
        self.status = "no data"
        self.t = 0.0          # monotonic time of the last new reading
        self._state = "off"   # off | engaging | on (locked, or for a cavity-locked laser: watched) | recovering
        self.state_t = time.time()  # ...since then
        self.observers = []   # its lock observed (observe.py): each told of every change of its state
        self.message = ""     # why the lock is off, or what to do
        self.offset = None    # last known scan offset (its piezo's set value)
        self.unit = "V"
        self.piezo = ACTUATOR  # the parameters, below laserN:, that set its piezo voltage and its current, by what
        self.cc = CURRENT      # its scan generator drives (dlcpro.drives: read when it is locked or recovered, watched)
        self.scan_out = None   # that: scan:output-channel (None: not known)
        self.token = 0        # bumped on every engage/disengage; late callbacks compare it
        # mode-hop recovery
        self.guard = Guard()
        self.runner = None    # the recovery in progress
        self.current = None   # mA, last known current as measured (polled while the recovery is enabled)
        self.current_set = None  # mA, its set current (polled too: a change by hand)
        self.cavity_on = None  # the DLC pro's own lock enabled (polled for a cavity-locked laser)
        self.cavity_txt = ""
        self.note = ""        # what the last recovery did
        self.note_t = 0.0
        self.hop_why = ""     # why the recovery in progress started
        self.hop_t = 0.0      # when
        self.resume = False   # re-engage the wavemeter lock when it is done
        self.shg_txt = ""     # the SHG cavity's lock state (polled for a laser with an SHG stage)
        self.last = None      # what the last recovery saw and did (a dict, as logged)
        self.setup = None     # the last setup check: [(level, text)], None while it runs
        self.seed = None      # mW, its TA's seed power (polled while the recovery is enabled, if it has one)
        self.seed_name = ""   # the parameter it is read from ('': none)
        self.seed_floor = 0.0  # mW: the TA's own power-min
        self.lasing = None    # by the seed power: True, False, None (no telling)
        self.seed_low = 0     # polls in a row with too little seed
        self.trans_name = ""  # its cavity transmission: the lock input's present value (absolute name)
        self.trans = None     # V, as last polled
        self.trans_hist = deque()  # (t, V) over the last Engine.LEVEL_S, while its lock is on
        self.trans_fine = deque(maxlen=FINE_N)  # V, polled running as it should
        self.shg_level = None  # V: its SHG cavity's intra-cavity signal, as last polled
        self.shg_hist = deque()  # (t, V) of that over the last Engine.LEVEL_S, armed
        self.shg_fine = deque(maxlen=FINE_N)  # V, polled running as it should
        # its wavemeter channel switched off ('Use' unticked in the WLM software)
        self.off = False       # switched off: no readings until it is switched on again
        self.borrowed = False  # switched on by this program (for a look or a recovery): to be switched off again
        self.look_why = ""     # what its DLC pro said that has it looked at now, its channel on ('': no look)
        self.look_until = 0.0  # the look ends then at the latest
        self.look_next = 0.0   # no look before then...
        self.look_due = ""     # ...one asked for sooner is made then: why
        self.look_n = (0, 0)   # readings in a row in the look: on target, off target
        self.looked = ""       # what the last look found
        self.looked_t = 0.0
        self.pending = None    # a recovery waiting for readings, its channel switched on for it: what it begins with
        self.wait_since = 0.0  # when the recovery began waiting for them
        self.wait_fail = ""    # why its channel could not be switched on for it
        # lock dropouts (a laser its DLC pro locks)
        self.trans_sig = None  # the signal id of its transmission input
        self.drops = deque()   # (time.time(), s) of its dropouts over the last hour
        self.drops_total = 0   # ...since the program started
        self.drop_last = ""    # the last one, said
        self.drop_said = 0.0   # time.time(): when the last one the console had happened...
        self.drop_unsaid = 0   # ...and how many since were not said there (at most one a second)
        self.hold_since = None  # monotonic: since when its lock should hold (on, no recovery)
        # where it was on target as it should be (the catch: its right mode is there)
        self.fine_at = None    # (monotonic, scan offset, set current) when last polled so
        # an SLS unit locks it (sls.py)
        self.sls = SlsParams.from_dict(cfg.get("sls"))
        self.sls_view = None   # sls.View: its unit's last poll
        self.sls_error = ""    # its unit does not answer: why ('': it does)
        self.sls_since = None  # time.time(): holding since then, by its unit's polls (None: not holding)
        self.sls_held_t = None  # time.time(): the last poll it held at (a dropout begins there)
        self.sls_count = None  # its unit's relock count at the last poll
        self.sls_drop = None   # (time.time(), why): a dropout of its lock going on, since then
        self.sls_bad_t = None  # time.time(): the last poll it did not hold at
        self.sls_guard = None  # sls.Guard: armed, the guard of a slave its PLL beat-locks
        self.sls_note = ""     # what the guard did last, said (and when, monotonic)
        self.sls_note_t = 0.0
        self.steering = None   # MHz: the wavemeter lock brings it to its beat point plus this (None: not steering)
        self.steer_fail = ""   # ...and could not: why
        self.beat_e = None     # MHz: its last reading from its beat point (None: no telling)
        self.beat_off_n = 0    # readings in a row further than SlsParams.off_mhz from its beat point
        self.near_n = 0        # readings in a row within capture of where it is steered
        self.beat_seen = deque(maxlen=BEAT_N)  # MHz: its readings less its master's, its PLL holding it
        self.amp_seen = deque(maxlen=BEAT_N)  # V: its beat amplitude (a one-channel PLL's), holding it
        self.recovered = None  # its mode-hop recovery, for the guard, just ended: '' it did, else why not
        self.steady_t = 0.0    # monotonic: its readings steady (no jump of STEADY_MHZ) since then...
        self.steady_f = NAN    # ...the last of them (THz)
        self.mtrack = Track()  # a PLL's slave: its master's readings, followed (this PC's or another's)
        self.beat_ok = False   # ...its beat learned since the program started (one from before, or typed, steers it
        #                        back, but the wavemeter reading it off its beat point does not count until then)
        self.master_row = None  # ...its master as last looked at (Elsewhere.row's), None: not found
        self.master_lost = False  # ...its lock lost, as its row says (its guard told at once, not at its next poll)
        self.mview = None      # ...its master's SLS servo, polled from here (sls.View), its error, holding since
        self.m_error = ""
        self.m_since = None
        self.fast_seen = deque(maxlen=BEAT_N)   # V: its PLL's fast output, holding it...
        self.phase_seen = deque(maxlen=BEAT_N)  # ...and its phase error
        self.band_t = None     # time.time(): they read far from there since (the wavemeter reading it where it should)
        self.off_seen = deque(maxlen=BEAT_N)  # MHz: its readings from its beat point since they went off it, its PLL
        self.off_t = None      # holding it by all it shows, and since when (monotonic): a step of its beat, or lost?
        self.beat_step = False  # ...being told: its readings off its beat point do not count meanwhile
        self.steps = deque(maxlen=8)  # monotonic: its beat learned again so (a step of it)
        # its piezo trimmed to keep its PLL's fast output near 0 (SlsParams.trim: its slow servo unused)
        self.trim_seen = []    # (time.time(), V): its fast output since its last step settled...
        self.trim_from = 0.0   # ...looked at from then on (monotonic)
        self.trim_last = None  # (the readings before, 'piezo' or 'current', V or mA moved): the last step, its effect
        self.trim_level = math.inf  # V: its fast output over the last look (inf: not trimmed of late)
        self.trim_size = 0.0   # V: its next step to learn by (0: Engine.TRIM_PROBE)...
        self.cur_size = 0.0    # mA: ...and its current's (0: Engine.CUR_PROBE)
        self.trim_probes = []  # (kind, V or mA moved, V its fast output moved by it, give or take): what they did
        self.trim_sign = 1     # (their direction, every other one the other way)
        self.trim_cur_want = False  # how its fast output follows its current is wanted (an upkeep due: trim_cur)
        self.trim_odd = 0      # steps in a row that moved its fast output the other way from what was learned
        self.trim_busy = False  # a step on its way (its piezo read and set, in its controller's thread)
        self.trim_moved = 0.0  # V its piezo was moved so, since Relock
        self.trim_said = ""    # what it did last
        # the last adjustment made from another PC (share.py)
        self.remote_note = ""  # what, and from where
        self.remote_t = 0.0    # monotonic...
        self.remote_wall = 0.0  # ...and time.time(): when
        # the upkeep (HopParams.upkeep): the top of its mode found now and then, and backed off from
        self.upkeeping = False  # the recovery in progress is that
        self.opt_why = ""      # its optimum taken last: why its SHG level was not (as its DLC pro said)
        self.upkeep_t = 0.0    # monotonic: when it was armed, recovered or upkept last
        # what its wavemeter lock last did by itself (it ran away: its gains reversed)
        self.lock_note = ""
        self.lock_t = 0.0      # monotonic...
        self.lock_wall = 0.0   # ...and time.time(): when
        # a laser map (lasermap.py)
        self.mapper = None     # the map going on (a MapRunner)...
        self.map_last = None   # ...or the last one, over
        self.map_plan = MapPlan.from_dict(cfg["map"]) if isinstance(cfg.get("map"), dict) else None  # the last plan
        self.map_note = ""     # how the last map ended
        self.map_pending = None  # a map about to begin (its Mapper), its laser's settings being read
        self.map_info = None   # what a map would use of its laser, as last read for the dialog (map_look)
        self.gaps = deque(maxlen=30)  # s between its readings, of late (how fast the wavemeter reads it)
        # its laser's model (lasermodel.py), fitted to its last map: what recoveries go by (HopParams.method 'model')
        m = cfg.get("model")
        self.model_dict = m if isinstance(m, dict) and isinstance(m.get("model"), dict) else None  # {model, from,
        self._model = None     # (the LaserModel, made of model_dict once asked for)          laser, t, right}
        self.model_note = ""   # how taking its laser's model from a map's file went, or is going

    @property
    def state(self) -> str:
        """off | engaging | on (locked, or for a cavity-locked laser: watched) | recovering | mapping"""
        return self._state

    @state.setter
    def state(self, value: str):
        if value != self._state:
            self._state, self.state_t = value, time.time()
            for o in self.observers:
                o.state(self.state_t, value)

    @property
    def param(self) -> str:
        return f"laser{self.laser}:{self.piezo}"

    @property
    def error_mhz(self) -> float:
        return (self.freq - self.f_set) * 1e6  # NaN if no reading or no setpoint

    def set_setpoint(self, nm):
        if nm and float(nm) > 0:
            self.setpoint_nm = float(nm)
            self.f_set = C_NM_THZ / self.setpoint_nm
        else:
            self.setpoint_nm, self.f_set = None, NAN

    def to_config(self) -> dict:
        d = {"name": self.name, "hidden": self.hidden, "lock": self.lock.p.to_dict()}
        if self.setpoint_nm:
            d["setpoint_nm"] = self.setpoint_nm
        if self.device and dac.is_dac(self.device):
            d["link"] = {"device": self.device, "ao": self.laser}
            if self.ff_ao is not None:
                d["link"].update(ff_ao=self.ff_ao, ff_ratio=self.ff_ratio)
        elif self.device:
            d["link"] = {"device": self.device, "laser": self.laser}
        if self.hop != HopParams():
            d["hop"] = self.hop.to_dict()
        if self.map_plan is not None:
            d["map"] = self.map_plan.to_dict()
        if self.model_dict is not None:
            d["model"] = self.model_dict
        if self.sls.kind:
            d["sls"] = self.sls.to_dict()
        return d

    def laser_model(self):
        """Its laser's model (lasermodel.LaserModel), if it has one, fitted to a map of the laser it is linked to now
        (None: none; or numpy, scipy missing)."""
        d = self.model_dict
        if d is None or d.get("laser") != [self.device, self.laser]:
            return None
        if self._model is None:
            try:
                from .lasermodel import LaserModel

                self._model = LaserModel.from_dict(d["model"])
            except (ImportError, TypeError, ValueError, KeyError) as exc:
                log.warning("channel %d: its laser's model cannot be used: %s", self.n, exc)
                self.model_dict = None
                return None
        return self._model

    def adopt_model(self, model, source: str):
        """Its laser's model from now on (lasermodel.LaserModel fitted to a map of its laser now: source, that map)."""
        self.model_dict = {"model": model.to_dict(), "from": source, "laser": [self.device, self.laser],
                           "t": time.strftime("%Y-%m-%d %H:%M"), "right": round(model.right, 3)}
        self._model = model

    @property
    def dac(self) -> bool:
        """It is driven by an analog output (dac.py), not by a DLC pro."""
        return dac.is_dac(self.device)

    @property
    def relock(self) -> bool:
        """Its DLC pro locks it to a cavity: engaging it arms the relock, the wavemeter lock never runs."""
        return self.hop.cavity

    @property
    def beat_slave(self) -> bool:
        """An SLS PLL beat-locks it to a master."""
        return self.sls.on and self.sls.kind == "pll"

    @property
    def hop_stage(self) -> str:
        if self.state != "recovering":
            return ""
        if self.runner is not None:
            return self.runner.stage
        if self.sls_guard is not None and self.sls_guard.relocking:
            return self.sls_guard.said(time.time())
        if self.borrowed and (self.off or self.pending is not None):
            return "its wavemeter channel switched on: waiting for readings"
        return "reading the laser's settings"

    @property
    def hop_prompt(self) -> str:
        """Step by step: what the recovery waits for you to allow ('' if nothing)."""
        return self.runner.asking if self.state == "recovering" and self.runner is not None else ""


class Controller(threading.Thread):
    """Owns the connection to one DLC pro; every network call happens here."""

    POLL = 0.5   # s between read-backs of watched parameters
    RETRY = 2.0  # s before reconnecting after a failure

    def __init__(self, key: str, host: str = "", port: int = 0, client=None):
        """client: in place of a DLC pro's (an analog output board's: dac.DacClient)."""
        super().__init__(daemon=True, name=f"dlcpro {key}")
        self.key = key
        self.client = client if client is not None else DLCPro(host, port, timeout=1.0)
        self.online = None  # None until the first exchange
        self.error = ""
        self._cv = threading.Condition()
        self._sets = {}     # param -> (value, on_error, on_done); the latest value wins
        self._jobs = deque()
        self._watch = {}    # param -> (callback(value), kind)
        self._next_poll = 0.0
        self._retry_at = 0.0
        self._running = True

    def set(self, param, value, on_error=None, on_done=None):
        with self._cv:
            self._sets[param] = (value, on_error, on_done)
            self._cv.notify()

    def cancel(self, param):
        with self._cv:
            self._sets.pop(param, None)

    def submit(self, fn, on_error=None):
        """Run fn(client) in this thread; on_error(exc) if it raises."""
        with self._cv:
            self._jobs.append((fn, on_error))
            self._cv.notify()

    def watch(self, param, callback, kind="float"):
        """Read param every POLL s and call callback(value); kind: 'float', 'bool' or 'str'."""
        with self._cv:
            self._watch[param] = (callback, kind)
            self._next_poll = 0.0
            self._cv.notify()

    def unwatch(self, param):
        with self._cv:
            self._watch.pop(param, None)

    def stop(self):
        with self._cv:
            self._running = False
            self._cv.notify()

    def run(self):
        while True:
            with self._cv:
                while self._running and not self._sets and not self._jobs:
                    wait = None
                    if self._watch:
                        wait = self._next_poll - time.monotonic()
                        if wait <= 0:
                            break
                    self._cv.wait(wait)
                if not self._running:
                    break
                sets, self._sets = self._sets, {}
                jobs = list(self._jobs)
                self._jobs.clear()
                watch = {}
                if self._watch and time.monotonic() >= self._next_poll:
                    watch = dict(self._watch)
                    self._next_poll = time.monotonic() + self.POLL
            for param, (value, on_error, on_done) in sets.items():  # lock writes first
                self._call(lambda c, p=param, v=value: c.set(p, v), on_error, on_done)
            for fn, on_error in jobs:
                self._call(fn, on_error)
            for param, (callback, kind) in watch.items():
                get = {"bool": DLCPro.get_bool, "str": DLCPro.get_str}.get(kind, DLCPro.get_float)
                self._call(lambda c, p=param, cb=callback, g=get: cb(g(c, p)), None)
        self.client.close()

    def _call(self, fn, on_error, on_done=None):
        if not self.client.is_open and time.monotonic() < self._retry_at:
            if on_error:
                on_error(ConnectionError(self.error or "offline"))
            return
        try:
            fn(self.client)
        except DecopError as e:  # the device answered, the connection is fine
            self.online = True
            if on_error:
                on_error(e)
        except OSError as e:  # timeouts, resets, refused connections
            self.client.close()
            self.online, self.error = False, str(e) or type(e).__name__
            self._retry_at = time.monotonic() + self.RETRY
            if on_error:
                on_error(e)
        except Exception:
            log.exception("controller %s", self.key)
        else:
            self.online, self.error = True, ""
            if on_done:
                on_done()


class _HopIO:
    """A recovery's operations, through the laser's controller thread; the callbacks come back
    under the engine's lock. Current and offset go through the controller's coalescing writes,
    everything else runs as a job (modehop.perform). The procedure's current and scan offset are the
    laser's current and piezo, whichever parameters set them (Channel.cc, Channel.piezo)."""

    def __init__(self, engine: "Engine", ch: Channel, controller: Controller):
        self.engine, self.ch, self.ctrl = engine, ch, controller
        self.laser = ch.laser
        self.names = {CURRENT: ch.cc, ACTUATOR: ch.piezo}

    def do(self, op, done, failed):
        e, ch = self.engine, self.ch

        def ok(result=None):
            with e.lock:
                if op[0] == "set" and op[1] == CURRENT:
                    ch.current_set = op[2]
                elif op[0] == "set" and op[1] == ACTUATOR:
                    ch.offset = op[2]
                done(result)

        def err(exc):
            with e.lock:
                failed(exc)

        r = e._piezo_per_ma(ch) if op[0] == "set" and op[1] == CURRENT and ch.upkeeping and ch.beat_slave else 0.0
        if r and ch.current_set is not None:  # an upkeep of a laser an SLS PLL holds: its piezo moves with its
            self.ctrl.submit(self._with_piezo(op[2], op[2] - ch.current_set, r, ok), err)  # current, its frequency
            return                                                                          # kept
        if op[0] == "set" and op[1] in (CURRENT, ACTUATOR):
            self.ctrl.set(f"laser{self.laser}:{self.names[op[1]]}", op[2], on_error=err, on_done=ok)
            return
        laser, wire = self.laser, op
        if op[0] == "get":
            wire = ("get", tuple(self.names.get(name, name) for name in op[1]))
        self.ctrl.submit(lambda client: ok(perform(client, laser, wire)), err)

    def _with_piezo(self, current: float, di: float, r: float, ok):
        """A job: its current set to `current` (di mA from where it was set) and its piezo moved r*di V from where it
        is (read afresh), within its lock's limits: its PLL holds it on the same frequency all the while, its fast
        output where it was, and where on its mode it runs moves (as a cavity lock would have it)."""
        e, ch = self.engine, self.ch
        cc, piezo = f"laser{self.laser}:{self.names[CURRENT]}", f"laser{self.laser}:{self.names[ACTUATOR]}"
        lo, hi = ch.lock.p.v_min, ch.lock.p.v_max

        def job(client):
            client.set(cc, current)
            v = client.get_float(piezo)
            new = round(min(max(v + r * di, lo), hi), 5)
            if abs(new - v) > 1e-9:
                client.set(piezo, new)
            with e.lock:
                ch.offset = new
            ok()

        return job

    def think(self, fn, done, failed):
        """Work fn out in a thread of its own, the engine's lock not held (seconds of numpy: the laser's model);
        the answer under it."""
        e = self.engine

        def run():
            try:
                result = fn()
            except Exception as exc:  # (a bug: the recovery stops, and puts things back)
                log.exception("channel %d: recovery", self.ch.n)
                with e.lock:
                    failed(exc)
                return
            with e.lock:
                done(result)

        threading.Thread(target=run, daemon=True, name=f"recovery {self.ch.n}: thinking").start()


class Engine:
    LEVEL_S = 10.0  # s: a cavity lock's transmission (an SHG signal) averaged over this long, to tell one that keeps
    #                 losing it
    LOOK_S = 15.0    # s: a look at a laser, its wavemeter channel switched on for it, lasts this long at most...
    LOOK_GAP = 60.0  # s: ...and comes this long after the last at the soonest
    LOOK_N = 5       # readings in a row on target (its locks saying locked) end a look; off target, it is relocked
    WAIT_S = 10.0    # s a recovery waits for readings once its wavemeter channel is switched on for it
    FINE_S = 900.0   # s: where it was last on target as it should be, if that recent, is where its right mode is
    OBSERVE_N = 32          # lasers' locks observed at once at most (observe.py; the oldest dropped)
    OBSERVE_S = 7 * 86400.0  # s an observation not ended is kept at most
    OBSERVED_S = 600.0      # s an ended one is kept, to be fetched
    END_S = 6.0             # s an ended one waits for its dropouts up to its end to be read, at most...
    PAST_S = 0.25           # ...read until this long past its end (its recorder starts a moment after it is told)

    def __init__(self, config: dict, make_source, discovery_targets=None, extra_devices=(), log_path=None,
                 drop_path=None):
        """log_path: a file every recovery is appended to, as a line of JSON; drop_path: the same for every lock
        dropout."""
        self.config = config
        self.log_path, self.drop_path = log_path, drop_path
        self.monitors = {}  # channel -> its DropoutMonitor
        self.sls_watches = {}  # channel -> its SlsWatch (an SLS unit locks its laser)
        self.master_watches = {}  # channel -> the SlsWatch of its master's SLS servo (a PLL's slave, polled here)
        self.elsewhere = None  # meters.Elsewhere: lasers on this PC's other wavemeters and on other PCs (a master)
        self.observations = {}  # tag -> an Observation of a channel's lock (observe.py), oldest first
        self.started = time.time()
        self.lock = threading.RLock()
        self.channels = {int(k): Channel(int(k), c) for k, c in (config.get("channels") or {}).items()}
        self.active = []         # channels measured by the wavemeter, in order
        self.off = []            # channels switched off on the wavemeter, which the source can switch on
        self._timers_t = 0.0
        self.layout_version = 0  # bumped when the set of rows to show changes
        self.source_ok, self.source_msg = False, "starting"
        self.rate = 0.0          # new readings per second, all channels
        self._n_readings, self._rate_t = 0, time.monotonic()
        self.known = list(dict.fromkeys(config.get("known_devices") or []))
        self.dacs = dac.Memory(config.get("dac"))  # what each analog output was last set to
        self.devices = {}
        for key in [*self.known, *extra_devices]:
            d = Device.from_key(key)
            self.devices[d.key] = d
        self.devices_version = 0
        self.scanning = False
        self.discovery_targets = discovery_targets
        self.controllers = {}
        ref = config.get("reference")
        self.reference = Reference.from_config(ref) if ref else None
        self.dirty = False       # settings changed since the last save
        self.source = make_source(self)
        if self.source is not None:  # (the WLM software stuck or gone: restarted, where the source can)
            self.source.restart = config.get("wlm_restart", True) is not False

    # ------------------------------------------------------------- lifecycle
    def start(self):
        with self.lock:
            for ch in self.channels.values():
                if ch.device:
                    self._watch(ch)
                if ch.sls.on:
                    self._watch_sls(ch)
        self.source.start()
        self.start_discovery()

    def stop(self):
        with self.lock:
            monitors, self.monitors = list(self.monitors.values()), {}
            watches, self.sls_watches = list(self.sls_watches.values()), {}
            watches += list(self.master_watches.values())
            self.master_watches = {}
        for w in watches:
            w.stop()  # (no more releases or engagements: its PLL is left as it is)
        for m in monitors:
            m.stop()  # (each puts its recorder's settings back)
        with self.lock:
            for ch in self.channels.values():
                if ch.state != "off":
                    self._disengage(ch, "")
        end = time.monotonic() + 20.0  # a recovery (a map) that was cut short puts current and offset back
        while time.monotonic() < end:
            with self.lock:
                runners = [r for ch in self.channels.values() for r in (ch.runner, ch.mapper) if r is not None]
                for r in runners:
                    r.tick(time.monotonic())
                if not runners:
                    break
            time.sleep(0.02)
        with self.lock:
            for ch in self.channels.values():
                self._release(ch)  # a channel switched on for a look or a recovery: off again
        end = time.monotonic() + 2.0
        while getattr(self.source, "switching", False) and self.source.is_alive() and time.monotonic() < end:
            time.sleep(0.02)
        self.source.stop()
        for m in monitors + watches:
            m.join(max(0.0, end + 5.0 - time.monotonic()))
        with self.lock:
            controllers = list(self.controllers.values())
        for c in controllers:
            c.stop()

    # ------------------------------------------- source callbacks (its thread)
    def on_state(self, ok: bool, msg: str):
        self.source_ok, self.source_msg = ok, msg

    @property
    def meter(self) -> dict:
        """The wavemeter these settings are for, as kept in them: {'serial': 1234, 'model': 'WS7'} ({}: none yet)."""
        m = self.config.get("wavemeter")
        return m if isinstance(m, dict) else {}

    def on_wavemeter(self, model: str, serial: int):
        """The source reads that wavemeter from now on: these settings are kept as its (the source keeps to it,
        also on a PC with several). Another one than before (swapped): nothing read from the one before counts,
        nothing stays locked."""
        with self.lock:
            old = self.meter.get("serial")
            if not serial or (old == serial and self.meter.get("model") == model):
                return
            self.config["wavemeter"] = {"serial": int(serial), "model": model}
            self.dirty = True
            if old and old != serial:
                for ch in self.channels.values():
                    if ch.state != "off":
                        self._disengage(ch, "unlocked: another wavemeter reads this channel now")
                    ch.freq = ch.raw = NAN
                    ch.status, ch.t = "no data", 0.0
                ref = self.reference
                if ref is not None:  # (the cavity is where it was; the wavemeter's error is another's)
                    self.reference = Reference(ref.channel, ref.p, ref.anchor)
        if old and old != serial:
            meter_log.warning("settings for wavemeter %s %s now (they were for %s)", model, serial, old)
        else:
            meter_log.info("reading wavemeter %s %s", model, serial)

    def choose_wavemeter(self, cands):
        """Of several wavemeters, the one these settings are for (a source given none asks): cands [(Found,
        {channel: THz})]. The one whose channels read near (10 GHz) the setpoints here on the most channels; None if
        none does, or two as many."""
        with self.lock:
            sets = {n: ch.f_set for n, ch in self.channels.items() if ch.f_set > 0}
        scores = sorted(((sum(1 for n, thz in freqs.items() if n in sets and abs(thz - sets[n]) < 0.01), k)
                         for k, (_, freqs) in enumerate(cands)), reverse=True)
        if not scores or scores[0][0] == 0 or (len(scores) > 1 and scores[1][0] == scores[0][0]):
            return None
        return cands[scores[0][1]][0]

    def on_active(self, chans, off=()):
        """chans: the channels the wavemeter measures; off: those switched off ('Use' unticked), which the source
        can switch on again. One that is neither any more is no longer measured: unlocked (watched no more)."""
        with self.lock:
            chans = list(chans)
            off = [n for n in off if n not in chans]
            for n in chans + off:
                if n not in self.channels:
                    self.channels[n] = Channel(n)
            for n in chans:
                if self.channels[n].off:
                    self._channel_on(self.channels[n])
            for n in off:
                if not self.channels[n].off:
                    self._channel_off(self.channels[n])
            for n in (set(self.active) | set(self.off)) - set(chans) - set(off):
                ch = self.channels[n]
                ch.off = ch.borrowed = False
                if ch.state != "off":
                    self._disengage(ch, "unlocked: channel no longer measured")
            self.active, self.off = chans, off
            self.layout_version += 1

    def on_switch_failed(self, n: int, on: bool, why: str):
        """The source could not switch channel n on (or off)."""
        with self.lock:
            ch = self.channels.get(n)
            if ch is None:
                return
            text = f"cannot switch channel {n} {'on' if on else 'off'} on the wavemeter: {why}"
            log.warning(text)
            if not on:
                ch.looked, ch.looked_t = text, time.monotonic()
                return
            ch.borrowed = False  # (it is not on: nothing to switch off again)
            if ch.mapper is not None:
                if ch.off:
                    ch.mapper.cancel(text)
            elif ch.runner is not None:
                if ch.off:
                    ch.runner.cancel(text)
            elif ch.state == "recovering":  # (waiting for readings to begin, or for its settings first)
                ch.wait_fail = text
                if ch.pending is not None:
                    self._wait_over(ch, text)
            elif ch.look_why:
                self._look_over(ch, text)

    def rows(self) -> list:
        """The channels to show: those measured, and those switched off that are locked, watched or being
        recovered, or could be watched (Relock); not those hidden."""
        with self.lock:
            chs = self.channels
            shown = set(self.active) | {n for n in self.off if chs[n].state != "off" or (
                chs[n].relock and chs[n].hop.enabled and chs[n].device) or chs[n].sls.on}
            return [n for n in sorted(shown) if not chs[n].hidden]

    # ------------------------------------------------ its channel switched off
    def _channel_off(self, ch: Channel):
        """Its wavemeter channel was switched off: no readings until it is on again. A recovery or a look needs
        them: switched on again at once. Else a laser its DLC pro locks stays watched by what its DLC pro says,
        and one this program locks to the wavemeter is held (its lock goes on when readings do)."""
        ch.off = True
        ch.freq = ch.raw = NAN
        ch.status = "channel off"
        ch.guard.pause()
        if ch.runner is not None:
            ch.runner.blind(True)
        if ch.state in ("recovering", "mapping") or ch.look_why:
            if not ch.borrowed:
                log.info("channel %d: switched off on the wavemeter during a %s: on again", ch.n,
                         "look" if ch.look_why else "laser map" if ch.state == "mapping" else "recovery")
            ch.borrowed = True
            self._switch(ch, True)
        else:
            ch.borrowed = False

    def _channel_on(self, ch: Channel):
        """Its wavemeter channel is measured again (switched on by this program, or by someone). A laser watched
        meanwhile that this program did not switch it on for is looked at (and relocked if it is off target)."""
        ch.off = False
        ch.status = "waiting"  # (for its first reading)
        if ch.runner is not None:
            ch.runner.blind(False)
        if not ch.borrowed and ch.state == "on" and ch.relock and not self._is_reference(ch):
            self._look(ch, "its wavemeter channel was switched on again", switch=False)

    def _switch(self, ch: Channel, on: bool):
        if getattr(self.source, "switchable", False):
            self.source.switch(ch.n, on)

    def _release(self, ch: Channel):
        """Its wavemeter channel switched off again, if this program switched it on."""
        if ch.borrowed:
            ch.borrowed = False
            self._switch(ch, False)
            log.info("channel %d: switched off again on the wavemeter", ch.n)

    def _says_locked(self, ch: Channel) -> bool:
        return bool(ch.cavity_on) and says_locked(ch.cavity_txt) and (not ch.hop.shg or says_locked(ch.shg_txt))

    def _look(self, ch: Channel, why: str, switch: bool = True):
        """What its DLC pro says (why) may mean it is not locked on target any more, and its wavemeter channel is
        off: switched on for a look, LOOK_S at most; one asked for within LOOK_GAP of the last is made then.
        switch False: its channel is on already."""
        now = time.monotonic()
        if ch.look_why or ch.state != "on" or not ch.relock:
            return
        if switch:
            if not ch.off:
                return  # (readings come: the guard sees it)
            if now < ch.look_next:
                ch.look_due = ch.look_due or why
                return
            ch.borrowed = True
            self._switch(ch, True)
        ch.look_why, ch.look_until, ch.look_n, ch.look_due = why, now + self.LOOK_S, (0, 0), ""
        log.info("channel %d: %s: looked at with its wavemeter channel on", ch.n, why)

    def _looking(self, ch: Channel):
        """A reading in a look, its cavity lock saying locked (while its DLC pro relocks it, the laser is anywhere:
        it is given the time to): LOOK_N in a row on target with its locks saying locked, it is fine (its channel
        switched off again, if this program switched it on for the look); LOOK_N in a row off target, relocked."""
        e = self._hop_error(ch) - ch.guard.track
        if not math.isfinite(e):
            return
        if not (ch.cavity_on and says_locked(ch.cavity_txt)):
            ch.look_n = (0, 0)
            return
        on, off = ch.look_n
        on, off = (on + 1, 0) if abs(e) <= ch.hop.off_target() else (0, off + 1)
        ch.look_n = (on, off)
        if on >= self.LOOK_N and self._says_locked(ch):
            self._look_over(ch, f"locked on target ({e:+.0f} MHz)")
        elif off >= self.LOOK_N:
            self._look_relock(ch, e)

    def _look_relock(self, ch: Channel, e: float):
        why, ch.look_why = ch.look_why, ""
        ch.looked, ch.looked_t = f"{why}: {self._off_text(e)}", time.monotonic()
        self._start_recovery(ch, f"{why}, then {self._off_text(e)}", relock=True)

    @staticmethod
    def _off_text(e: float) -> str:
        if not math.isfinite(e):
            return "no valid reading"
        return f"jumped {e / 1e3:+.3f} GHz" if abs(e) >= 1e3 else f"locked {e:+.0f} MHz off: the wrong peak"

    def _look_over(self, ch: Channel, found: str):
        """The look is over: what it found; its channel switched off again if this program switched it on."""
        now = time.monotonic()
        why, ch.look_why, ch.look_next = ch.look_why, "", now + self.LOOK_GAP
        ch.looked, ch.looked_t = f"{why}: {found}", now
        log.info("channel %d: looked (%s): %s", ch.n, why, found)
        self._release(ch)

    def _look_timeout(self, ch: Channel):
        """LOOK_S without LOOK_N readings in a row either way, its cavity lock saying locked: as the last of them
        had it; with none, by its DLC pro (its lock still not saying locked: relocked)."""
        on, off = ch.look_n
        if off:
            self._look_relock(ch, self._hop_error(ch) - ch.guard.track)
        elif on:
            self._look_over(ch, "on target, " + ("its locks saying locked" if self._says_locked(ch) else
                                                 f"its SHG cavity's lock saying “{ch.shg_txt or '?'}”"))
        elif ch.cavity_on and not says_locked(ch.cavity_txt):
            why, ch.look_why = ch.look_why, ""
            ch.looked, ch.looked_t = f"{why}: still “{ch.cavity_txt}” after {self.LOOK_S:g} s", time.monotonic()
            self._start_recovery(ch, f"{why}, still after {self.LOOK_S:g} s", relock=True)
        else:
            self._look_over(ch, f"no valid reading in {self.LOOK_S:g} s; its DLC pro says it is locked: left as it is")

    def _timers(self, now: float):
        """Looks, and recoveries waiting for readings, that have run out of time; looks that are due."""
        if now - self._timers_t < 0.1:
            return
        self._timers_t = now
        for ch in list(self.channels.values()):
            if ch.pending is not None and now - ch.wait_since >= self.WAIT_S:
                self._wait_over(ch)
            if ch.look_why and now >= ch.look_until:
                self._look_timeout(ch)
            elif ch.look_due and now >= ch.look_next:
                why, ch.look_due = ch.look_due, ""
                if ch.off and ch.state == "on":
                    self._look(ch, why)
            if self._upkeep_due(ch, now):
                ch.upkeeping = True
                self._start_recovery(ch, "upkeep", budget=False, relock=True)

    def _upkeep_due(self, ch: Channel, now: float) -> bool:
        """A laser its DLC pro locks, with an SHG stage, watched, locked and steady (on target a while, its locks
        saying locked, its transmission up), or one an SLS PLL holds, its piezo trimmed (_beat_upkeep_due): is its
        upkeep due (HopParams.upkeep minutes since it was armed, recovered, or upkept)?"""
        hop = ch.hop
        if not (hop.enabled and hop.shg and hop.upkeep > 0) or self._is_reference(ch):
            return False
        if ch.state != "on" or ch.runner is not None or ch.pending is not None or ch.look_why or ch.off:
            return False
        if ch.beat_slave:
            return self._beat_upkeep_due(ch, now)
        if not (hop.cavity and ch.guard.armed and ch.cavity_on and says_locked(ch.cavity_txt)
                and says_locked(ch.shg_txt)):
            return False
        return now - ch.upkeep_t >= 60.0 * hop.upkeep and not self._slipping(ch)

    def _beat_upkeep_due(self, ch: Channel, now: float) -> bool:
        """The same of a laser an SLS PLL holds on its beat point (its guard holding it, its SHG cavity locked, its beat
        learned this session): its piezo trimmed (SlsParams.trim), its fast output near 0, and how that follows its
        piezo and its current learned (its current then moves with its piezo, its frequency kept: _HopIO); else the
        trim learns that first."""
        p, g, v = ch.sls, ch.sls_guard, ch.sls_view
        if not (p.trim and g is not None and g.stage == "hold") or now - ch.upkeep_t < 60.0 * ch.hop.upkeep:
            return False
        if not (v is not None and v.holds and ch.steering is None and not self._beat_lost(ch)
                and says_locked(ch.shg_txt)):
            return False
        if self._beat_point(ch, look=False) is None or (p.master and not ch.beat_ok):
            return False  # (its beat point not known for sure: released meanwhile, it could not be steered back)
        if p.trim_gain and not p.trim_cur:
            ch.trim_cur_want = True  # (its trim's next step: its current's)
            return False
        return bool(self._piezo_per_ma(ch)) and abs(ch.trim_level) <= 2.0 * self.TRIM_DEAD

    def on_readings(self, readings):
        with self.lock:
            self._n_readings += len(readings)
            ref = self.reference
            if ref is not None:  # the reference first, so this batch gets the newest correction
                for r in readings:
                    if r.ch == ref.channel:
                        ref.update(r.t, r.freq)
                ref.tick(readings[-1].t)
            for r in readings:
                ch = self.channels.get(r.ch)
                if ch is None:
                    continue
                if 0.0 < ch.t < r.t:
                    ch.gaps.append(r.t - ch.t)
                ch.raw, ch.status, ch.t = r.freq, r.status, r.t
                ch.freq = r.freq if ref is None or r.ch == ref.channel else ref.correct(r.freq)
                if ch.observers:  # (its lock observed: how far it was)
                    wall = time.time() - (time.monotonic() - r.t)
                    for o in ch.observers:
                        o.reading(wall, ch.error_mhz if ch.freq > 0 else None)
                if ch.mapper is not None:  # (a map has it: nothing else)
                    ch.mapper.on_reading(r)
                    continue
                if ch.freq > 0:  # (steady: no jump of STEADY_MHZ from the reading before)
                    if not abs(ch.freq - ch.steady_f) * 1e6 <= STEADY_MHZ:
                        ch.steady_t = r.t
                    ch.steady_f = ch.freq
                else:
                    ch.steady_t, ch.steady_f = r.t, NAN
                withheld = self._hop_reading(ch, r.t) if ch.hop.enabled or ch.runner else False
                if ch.beat_slave:
                    self._beat_reading(ch)
                elif ch.state == "on" and ch.freq > 0 and not withheld and not ch.relock and not ch.sls.on:
                    self._step(ch)
                if ch.pending is not None and ch.freq > 0:  # the readings it waited for
                    args, ch.pending = ch.pending, None
                    self._begin_recovery(ch, *args)
                elif ch.look_why and ch.state == "on":
                    self._looking(ch)
            for ch in self.channels.values():  # a recovery (a map) waiting for time to pass
                if ch.runner is not None:
                    ch.runner.tick(readings[-1].t)
                if ch.mapper is not None:
                    ch.mapper.tick(readings[-1].t)
            self._timers(readings[-1].t)

    def _step(self, ch: Channel):
        gains = (ch.lock.p.kp, ch.lock.p.ki)
        u = ch.lock.update(ch.error_mhz)
        if (ch.lock.p.kp, ch.lock.p.ki) != gains:  # it ran away: its gains reversed by the lock (or put back)
            self.dirty = True
            note, ch.lock.note = ch.lock.note, ""
            if note:
                ch.lock_note, ch.lock_t, ch.lock_wall = note, time.monotonic(), time.time()
                log.warning("channel %d: %s", ch.n, note)
        if not ch.lock.active:  # (first: unlocking drops the lock's writes on their way, not the one it puts back)
            self._disengage(ch, ch.lock.message)
        if u is not None:
            ch.offset = u
            tok = ch.token
            c = self._controller(ch.device)
            c.set(ch.param, u, lambda e: self._comm_failed(ch, tok, e))
            if ch.ff_ao is not None and ch.ff_ratio and ch.ff0 is not None:  # (an analog output's feed-forward)
                p = ch.lock.p
                ff = min(max(ch.ff0 + (u - ch.lock.u0) / ch.ff_ratio, p.v_min), p.v_max)
                c.set(f"laser{ch.ff_ao}:{ACTUATOR}", ff, lambda e: self._comm_failed(ch, tok, e))

    def _comm_failed(self, ch: Channel, tok: int, exc: Exception):
        with self.lock:
            if ch.token == tok and ch.state == "on":
                self._disengage(ch, f"unlocked: {exc}")

    # ---------------------------------------------------- GUI-facing actions
    def toggle(self, n: int):
        with self.lock:
            if self.channels[n].state == "off":
                self.engage(n)
            elif self.channels[n].state == "recovering":
                self.cancel_recovery(n)
            else:
                self.disengage(n)

    def engage(self, n: int):
        with self.lock:
            ch = self.channels[n]
            if ch.state != "off":
                return
            if self.reference is not None and self.reference.channel == n:
                return  # the reference laser is locked to its cavity, not to the wavemeter
            if ch.sls.on:  # an SLS unit locks it: watched (a PLL's slave, guarded)
                self._arm_sls(ch)
                return
            if not ch.device:
                ch.message = "right-click to link a laser first"
                return
            if ch.setpoint_nm is None:
                if not ch.freq > 0:
                    ch.message = "no setpoint and no valid reading"
                    return
                ch.set_setpoint(round_sig(C_NM_THZ / ch.freq))
                self.dirty = True
            if ch.relock:
                self._arm(ch)
                return
            ch.state, ch.message = "engaging", ""
            ch.token += 1
            ch.guard.reset()
            tok, key, laser, params, on_dac = ch.token, ch.device, ch.laser, ch.lock.p, ch.dac
            ff_ao = ch.ff_ao if ch.dac and ch.ff_ratio else None

            def job(client):
                piezo = ACTUATOR if on_dac else self._drives(ch, client, key, laser)[0]  # (what its scan drives now)
                v = client.get_float(f"laser{laser}:{piezo}")  # (an analog output never set: not known, an error)
                try:
                    unit = (client.get_str(f"laser{laser}:scan:unit") or "V") if piezo == ACTUATOR else "V"
                except DecopError:
                    unit = "V"
                ff0 = client.get_float(f"laser{ff_ao}:{ACTUATOR}") if ff_ao is not None else None
                with self.lock:
                    if ch.token != tok:
                        return
                    if ch.piezo != piezo and not on_dac:  # (its scan switched to another output meanwhile)
                        ch.state, ch.message = "off", f"cannot lock: its scan now drives {output_name(ch.scan_out)}"
                        ch.token += 1
                        return
                    if unit.strip().lower() not in ("v", ""):  # (its scan drives something else, which it does not say)
                        ch.state, ch.message = "off", (f"cannot lock: its scan offset is {v:.3f} {unit}, not the piezo "
                                                       f"(see Scan › Output channel at the DLC pro)")
                        ch.token += 1
                        return
                    ch.offset, ch.unit, ch.ff0 = v, unit, ff0
                    if not params.v_min <= v <= params.v_max:
                        what = "offset" if piezo == ACTUATOR else f"piezo voltage ({piezo})"
                        ch.state, ch.message = "off", f"{what} {v:.3f} outside {params.v_min:g} … {params.v_max:g}"
                        ch.token += 1
                        return
                    ch.lock.engage(v)
                    ch.state = "on"

            def failed(exc):
                with self.lock:
                    if ch.token == tok:
                        ch.state, ch.message = "off", f"cannot lock: {exc}"
                        ch.token += 1

            self._controller(ch.device).submit(job, failed)

    def _arm(self, ch: Channel, check: bool = True, take: bool = True):
        """A laser its DLC pro locks to a cavity: watch it, and relock it after a hop. check: if it is
        not locked on target now, relock it now. take: Relock clicked (its optimum taken, the first time)."""
        hop = ch.hop
        if not hop.enabled:
            ch.message = "mode-hop recovery is off: right-click › Mode-hop recovery…"
            return
        e = self._hop_error(ch) - ch.guard.track
        slip = self._slipping(ch)
        if check and (ch.cavity_on is False or (math.isfinite(e) and abs(e) > hop.off_target()) or slip):
            if not math.isfinite(e) and not ch.off:  # (switched off, its channel is switched on for the relock)
                ch.message = "relock: needs a valid reading"
                return
            why = "cavity lock off" if ch.cavity_on is False else (
                f"off by {e / 1e3:+.3f} GHz" if abs(e) >= 1e3 else f"off by {e:+.0f} MHz" if abs(e) > hop.off_target()
                else self._slip_text(*slip))
            self._start_recovery(ch, why, budget=False, relock=True)
            return
        ch.state, ch.message = "on", ""
        ch.token += 1
        ch.guard.reset()
        ch.upkeep_t = time.monotonic()
        if check and take and not hop.optimum:  # clicked on a laser set up by hand: its optimum, the first time
            self._take_optimum(ch)
        if check and ch.off:  # is it locked on target? A look, its channel switched on for it
            ch.look_next = 0.0
            self._look(ch, "Relock clicked")

    def disengage(self, n: int, message: str = ""):
        with self.lock:
            ch = self.channels[n]
            if ch.state != "off":
                self._disengage(ch, message)

    def _disengage(self, ch: Channel, message: str):
        if ch.sls_guard is not None:  # (a PLL's slave: guarded no more, its PLL left as it is)
            if ch.sls_guard.stage in ("released", "recover", "steer", "engage", "failed"):
                ch.sls_note, ch.sls_note_t = "its PLL left released: stopped while it was being relocked", \
                    time.monotonic()
            ch.sls_guard = None
        if ch.steering is not None:
            self._unsteer(ch)
        if ch.runner is not None:  # a recovery: it puts current and offset back, then reports
            ch.resume = False
            ch.runner.cancel(message or "stopped")
            return
        if ch.mapper is not None:  # a map: likewise
            ch.mapper.cancel(message or "stopped")
            return
        if ch.state == "mapping":  # (one about to begin)
            m, ch.map_pending = ch.map_pending, None
            self._map_over(ch, m, f"map {message or 'stopped'} before it began")
            return
        ch.lock.disengage(message)
        ch.state, ch.message = "off", message
        ch.token += 1
        ch.resume = False
        ch.guard.reset()
        ch.pending, ch.look_why, ch.look_due = None, "", ""
        self._release(ch)
        c = self.controllers.get(ch.device)
        if c is not None:
            c.cancel(ch.param)

    def set_setpoint(self, n: int, nm):
        with self.lock:
            ch = self.channels[n]
            if self._follows(ch):
                ch.sls_note, ch.sls_note_t = (f"its setpoint is its beat point ({ch.sls.master_where}'s reading "
                                              f"{ch.sls.beat:+.1f} MHz): not set by hand"), time.monotonic()
                return
            ch.set_setpoint(nm)
            ch.guard.reset()  # not a hop: the guard waits for the laser to settle there
            ch.guard.track = 0.0
            if ch.state == "recovering":
                self._disengage(ch, "setpoint changed")
            elif ch.setpoint_nm is None and ch.state != "off":
                self._disengage(ch, "unlocked: setpoint cleared")
            self.dirty = True

    def setpoint_to_current(self, n: int):
        with self.lock:
            ch = self.channels[n]
            if ch.freq > 0 and ch.state != "recovering":
                ch.set_setpoint(round_sig(C_NM_THZ / ch.freq))
                ch.guard.reset()
                ch.guard.track = 0.0
                self.dirty = True

    def set_params(self, n: int, params: LockParams):
        with self.lock:
            self.channels[n].lock.p = params
            self.dirty = True

    def rename(self, n: int, name: str):
        with self.lock:
            self.channels[n].name = name
            self.dirty = True

    def set_wlm_restart(self, on: bool):
        """Restart the WLM software when it is stuck or gone (wavemeter.py), or not."""
        with self.lock:
            self.config["wlm_restart"] = bool(on)
            if self.source is not None:
                self.source.restart = bool(on)
            self.dirty = True

    def set_hidden(self, n: int, hidden: bool):
        with self.lock:
            ch = self.channels[n]
            if hidden and ch.state != "off":
                self._disengage(ch, "")
            ch.hidden = hidden
            self.layout_version += 1
            self.dirty = True

    def link(self, n: int, key: str, laser: int, ff_ao=None, ff_ratio: float = 0.0):
        """Channel n's laser: laserN of the DLC pro at key; or, key an analog output board's ('dac:mcc:0', dac.py),
        its output number laser, with ff_ao an output fed forward (its moves: the first's divided by ff_ratio)."""
        on_dac = dac.is_dac(key)
        key = key if on_dac else Device.from_key(key).key
        with self.lock:
            for other in self.channels.values():
                if other.n != n and other.device == key and other.laser == laser:
                    self._unlink(other)
                    other.message = (f"output {laser} moved to channel {n}" if on_dac else
                                     f"laser{laser} moved to channel {n}")
            ch = self.channels[n]
            self._unlink(ch)
            ch.device, ch.laser = key, int(laser)
            if on_dac:
                if ff_ao is not None and ff_ratio:
                    ch.ff_ao, ch.ff_ratio = int(ff_ao), float(ff_ratio)
                if ch.lock.p == LockParams():  # (a DLC pro's offset limits: not an output's)
                    ch.lock.p = LockParams(**{**ch.lock.p.to_dict(), **DAC_LOCK})
                ch.hop.enabled = False  # (mode-hop recovery needs a DLC pro)
                self._watch(ch)
                self.dirty = True
                return
            self._watch(ch)
            if key not in self.known:
                self.known.append(key)
            if key not in self.devices:
                self.devices[key] = Device.from_key(key)
                self.devices_version += 1
                threading.Thread(target=self._probe_one, args=(key,), daemon=True).start()
            self.dirty = True

    def unlink(self, n: int):
        with self.lock:
            self._unlink(self.channels[n])
            self.dirty = True

    def _unlink(self, ch: Channel):
        if ch.state != "off":
            self._disengage(ch, "")
        c = self.controllers.get(ch.device)
        if c is not None:
            for param in WATCHED + (ch.piezo, ch.cc) + ((ch.seed_name,) if ch.seed_name else ()):
                c.unwatch(f"laser{ch.laser}:{param}")
        self._unwatch_trans(ch)
        m = self.monitors.pop(ch.n, None)
        if m is not None:
            m.stop()
        ch.device, ch.offset, ch.message = None, None, ""
        ch.piezo, ch.cc, ch.scan_out, ch.unit = ACTUATOR, CURRENT, None, "V"
        ch.ff_ao, ch.ff_ratio, ch.ff0 = None, 0.0, None
        ch.current = ch.current_set = ch.cavity_on = ch.seed = ch.lasing = ch.shg_level = None
        ch.cavity_txt = ch.shg_txt = ch.seed_name = ""
        ch.shg_hist.clear()
        ch.shg_fine.clear()

    def _watch(self, ch: Channel):
        """Poll the offset of the linked laser (its piezo's set value) for display, and what its scan generator
        drives (which parameters set its piezo and its current, _take_drives); read its unit once."""
        key, laser = ch.device, ch.laser
        c = self._controller(key)

        def mine():
            return ch.device == key and ch.laser == laser

        def read_unit(client):
            out = None if ch.dac else scan_output(client, laser)
            unit = client.get_str(f"laser{laser}:scan:unit") or "V"
            with self.lock:
                if mine():
                    self._take_drives(ch, out)
                    ch.unit = unit if ch.piezo == ACTUATOR else "V"

        def on_scan_out(v):
            with self.lock:
                if mine():
                    self._take_drives(ch, int(v) if _num(v) else None)

        self._watch_offset(ch)
        c.submit(read_unit)
        if ch.dac:  # (its outputs set back to where this program left them whenever the board is opened: dac.py)
            for ao in (laser, ch.ff_ao):
                if ao is not None:
                    c.client.use(ao)
            return  # (nothing else to watch on an output)
        c.watch(f"laser{laser}:{SCAN_OUT}", on_scan_out)
        self._watch_hop(ch)

    def _watch_offset(self, ch: Channel):
        """Poll its piezo's set value (Channel.piezo: the scan offset, as a rule), for display and for where it was on
        target as it should be."""
        key, laser, name = ch.device, ch.laser, ch.piezo

        def on_offset(v):
            with self.lock:
                if ch.device == key and ch.laser == laser and ch.piezo == name:
                    if ch.state == "off":
                        ch.offset = v
                    if ch.current_set is not None and self._running_fine(ch):
                        ch.fine_at = (time.monotonic(), v, ch.current_set)  # (a cavity lock moves the offset)

        self._controller(key).watch(f"laser{laser}:{name}", on_offset)

    def _drives(self, ch: Channel, client, key: str, laser: int) -> tuple:
        """In a job (the controller's thread): what its scan generator drives now, taken on (_take_drives); returns the
        parameters that set its piezo and its current."""
        out = scan_output(client, laser)
        with self.lock:
            if ch.device == key and ch.laser == laser:
                self._take_drives(ch, out)
                return ch.piezo, ch.cc
        return drives(out)

    def _take_drives(self, ch: Channel, out):
        """Its scan generator drives `out` (scan:output-channel; None: it does not say, nothing changes): the parameters
        that set its piezo and its current follow (dlcpro.drives: the scan's offset sets that one), and so does what is
        watched. Changed while it is locked (at the DLC pro, by hand), it is unlocked: what the lock writes would set
        something else now; a recovery stops, leaving current and piezo where they are."""
        if out is None:
            return
        ch.scan_out = out
        piezo, cc = drives(out)
        if (piezo, cc) == (ch.piezo, ch.cc):
            return
        log.info("channel %d: its scan drives %s: its piezo is set by %s, its current by %s", ch.n, output_name(out),
                 piezo, cc)
        why = f"its scan now drives {output_name(out)}: the piezo is {piezo}"
        if ch.steering is not None:
            self._unsteer(ch, why)
        if ch.runner is not None:
            ch.resume = False
            ch.runner.cancel(why, keep=True)
        elif ch.state == "on" and not ch.relock and not ch.sls.on:
            self._disengage(ch, f"unlocked: {why}")
        c = self.controllers.get(ch.device)
        if c is not None:
            for name in (ch.piezo, ch.cc):
                c.unwatch(f"laser{ch.laser}:{name}")
        ch.piezo, ch.cc, ch.unit, ch.offset = piezo, cc, "V", None
        ch.current_set = ch.fine_at = None
        self._watch_offset(ch)
        self._watch_hop(ch)

    def _watch_hop(self, ch: Channel):
        """While mode-hop recovery is enabled: poll the current (set and measured), and the state of
        the DLC pro's own lock for a laser it locks to a cavity."""
        key, laser = ch.device, ch.laser
        c = self._controller(key)
        p = f"laser{laser}:"

        def mine():
            return ch.device == key and ch.laser == laser

        def on_current_set(v):
            with self.lock:
                if mine() and ch.state != "recovering":
                    if ch.current_set is not None and abs(v - ch.current_set) > 0.005:
                        ch.guard.reset()  # changed by hand: stand by until the laser is on target again
                    ch.current_set = v

        def on_current(v):
            with self.lock:
                if mine():
                    ch.current = v

        def on_cavity(v):
            with self.lock:
                if mine():
                    ch.cavity_on = v

        def on_txt(v):
            with self.lock:
                if mine():
                    ch.cavity_txt = v
                    if ch.off and self._armed(ch) and isinstance(v, str) and not says_locked(v):
                        self._look(ch, f"its cavity lock said “{v}”")  # (its wavemeter channel switched off)

        def on_shg(v):
            with self.lock:
                if mine():
                    ch.shg_txt = v
                    if ch.off and ch.hop.shg and self._armed(ch) and isinstance(v, str) and not says_locked(v):
                        self._look(ch, f"its SHG cavity's lock said “{v}”")

        def on_shg_level(v):
            with self.lock:
                if mine():
                    self._shg(ch, v)

        hop = ch.hop
        for param, cb, kind, on in ((ch.cc, on_current_set, "float", hop.enabled),
                                    (CURRENT_ACT, on_current, "float", hop.enabled),
                                    (CAVITY_LOCK, on_cavity, "bool", hop.enabled and hop.cavity),
                                    ("dl:lock:state-txt", on_txt, "str", hop.enabled and hop.cavity),
                                    (SHG_STATE, on_shg, "str", hop.enabled and hop.shg),
                                    (SHG_INTRA, on_shg_level, "float", hop.enabled and hop.shg)):
            if on:
                c.watch(p + param, cb, kind)
            else:
                c.unwatch(p + param)
        if hop.enabled and hop.cavity:
            if not ch.trans_name:
                self._find_trans(ch)
        else:
            self._unwatch_trans(ch)
        names = hop.seed_names if hop.enabled else ()
        if ch.seed_name and ch.seed_name not in names:
            c.unwatch(p + ch.seed_name)
            ch.seed_name, ch.seed, ch.lasing = "", None, None
        if names and not ch.seed_name:
            self._find_seed(ch)
        self._watch_drops(ch)

    def _find_seed(self, ch: Channel):
        """Which parameter shows its seed power (the first of HopParams.seed_names it has), and its TA's
        own minimum; then poll it."""
        key, laser, names = ch.device, ch.laser, ch.hop.seed_names
        c = self._controller(key)

        def job(client):
            values = perform(client, laser, ("get", names + (SEED_FLOOR,)))
            name = next((n for n, v in zip(names, values) if _num(v)), "")
            with self.lock:
                if ch.device != key or ch.laser != laser or not ch.hop.enabled or name not in ch.hop.seed_names:
                    return
                floor = values[-1]
                ch.seed_floor = float(floor) if _num(floor) and floor > 0 else 0.0
                ch.seed_name = name
                c.watch(f"laser{laser}:{name}", lambda v: self._on_seed(ch, key, laser, name, v))

        c.submit(job)

    def _find_trans(self, ch: Channel):
        """Which input shows its cavity transmission (the one its lock detection watches, if on; else its lock
        input); then poll that input's present value."""
        key, laser = ch.device, ch.laser
        c = self._controller(key)

        def job(client):
            got = perform(client, laser, ("get", TRANS))
            name = transmission_input(*got) or ""
            with self.lock:
                if ch.device != key or ch.laser != laser or not (ch.hop.enabled and ch.hop.cavity):
                    return
                ch.trans_sig = transmission_signal(*got)
                if not name:
                    return
                ch.trans_name = name
                ch.trans_hist.clear()
                c.watch(name, lambda v: self._on_trans(key, name, v))  # (absolute: shared by its lasers)

        c.submit(job)

    def _unwatch_trans(self, ch: Channel):
        name, key = ch.trans_name, ch.device
        ch.trans_name, ch.trans, ch.trans_sig = "", None, None
        ch.trans_hist.clear()
        ch.trans_fine.clear()
        c = self.controllers.get(key)
        if name and c is not None and not any(o.device == key and o.trans_name == name for o in self.channels.values()):
            c.unwatch(name)

    def _on_trans(self, key: str, name: str, v: float):
        """A poll of a cavity lock's transmission, for every laser of that controller locking on it."""
        with self.lock:
            for ch in list(self.channels.values()):
                if ch.device == key and ch.trans_name == name:
                    self._trans(ch, v)

    def _trans(self, ch: Channel, v: float):
        """Its transmission, while its DLC pro's lock is on (a lock turned off, by hand say, is not one that keeps
        losing it): kept over LEVEL_S; armed, its average below its limit means the lock keeps losing it: relock it
        (the hourly limit only if the wavemeter still reads it: without, it most likely stopped lasing). Running as
        it should: its level locked learned."""
        hop = ch.hop
        ch.trans = v
        if ch.state == "recovering" or not hop.enabled or not hop.cavity or not ch.cavity_on:
            ch.trans_hist.clear()
            return
        if self._running_fine(ch):
            self._usual(ch, ch.trans_fine, "trans_locked", v)
        now = time.monotonic()
        ch.trans_hist.append((now, v))
        while ch.trans_hist and now - ch.trans_hist[0][0] > self.LEVEL_S:
            ch.trans_hist.popleft()
        slip = self._slipping(ch) if self._armed(ch) else None
        if slip:
            ch.trans_hist.clear()
            why = self._slip_text(*slip)
            log.warning("channel %d: %s", ch.n, why)
            self._start_recovery(ch, why, budget=math.isfinite(self._hop_error(ch)) or ch.off, relock=True)
        elif ch.off and self._armed(ch):  # its wavemeter channel switched off: one poll that low is worth a look
            _, usual, _, limit = self.trans_levels(ch)
            if limit is not None and v < limit:
                self._look(ch, f"its transmission fell to {volts(v, usual)} V (locked {volts(usual, usual)} V)")

    def _shg(self, ch: Channel, v: float):
        """Its SHG intra-cavity signal, while armed: kept over LEVEL_S; running as it should, its level locked
        learned; far below that on average: recover it (the hourly limit only if the wavemeter still reads it:
        without, it most likely stopped lasing)."""
        hop = ch.hop
        ch.shg_level = v
        if ch.state == "recovering" or not hop.enabled or not hop.shg:
            ch.shg_hist.clear()
            return
        if self._running_fine(ch):
            self._usual(ch, ch.shg_fine, "shg_lock", v)
        if not self._armed(ch):
            ch.shg_hist.clear()
            return
        now = time.monotonic()
        ch.shg_hist.append((now, v))
        while ch.shg_hist and now - ch.shg_hist[0][0] > self.LEVEL_S:
            ch.shg_hist.popleft()
        gone = self._fallen(ch.shg_hist, hop.shg_lock, 0.0, hop.lock_min)
        if gone:
            ch.shg_hist.clear()
            mean, usual, _ = gone
            why = (f"the SHG signal is gone: {volts(mean, usual)} V on average over {self.LEVEL_S:g} s (locked "
                   f"{volts(usual, usual)} V)")
            log.warning("channel %d: %s", ch.n, why)
            self._start_recovery(ch, why, budget=math.isfinite(self._hop_error(ch)) or ch.off, relock=hop.cavity)
        elif ch.off and hop.shg_lock > 0 and v < hop.lock_min * hop.shg_lock:  # (as the transmission)
            self._look(ch, f"its SHG signal fell to {volts(v, hop.shg_lock)} V (locked {volts(hop.shg_lock)} V)")

    def _fallen(self, hist, usual: float, zero: float, frac: float):
        """(average, usual, zero) if the levels in hist (over most of LEVEL_S at least) average below frac of the way
        from zero to usual, else None (and if there is no telling)."""
        if usual <= zero or len(hist) < 5 or hist[-1][0] - hist[0][0] < 0.8 * self.LEVEL_S:
            return None
        mean = sum(v for _, v in hist) / len(hist)
        return (mean, usual, zero) if mean < zero + frac * (usual - zero) else None

    def _slipping(self, ch: Channel):
        """(average, level locked, zero) if its transmission has averaged below lock_min of the way from its zero to
        its level locked over the last LEVEL_S, else None (and if there is no telling)."""
        hop = ch.hop
        if not hop.cavity:
            return None
        return self._fallen(ch.trans_hist, hop.trans_locked, usable_zero(hop.trans_zero, hop.trans_locked),
                            hop.lock_min)

    def _slip_text(self, mean: float, usual: float, zero: float) -> str:
        return (f"the cavity lock keeps losing it: transmission {volts(mean, usual)} V on average over "
                f"{self.LEVEL_S:g} s (locked {volts(usual, usual)} V, zero {volts(zero, usual)} V)")

    @staticmethod
    def trans_levels(ch: Channel):
        """Its cavity transmission, V: (its average over the last LEVEL_S while its lock was on, else its last
        poll; its level locked, 0 while not known; its zero; the limit its average must stay above, armed, None if
        there is no telling); None if it is not watched."""
        hop = ch.hop
        hist = list(ch.trans_hist)
        if not hist and ch.trans is None:
            return None
        now = sum(v for _, v in hist) / len(hist) if hist else ch.trans
        usual = hop.trans_locked
        zero = usable_zero(hop.trans_zero, usual)
        return now, usual, zero, zero + hop.lock_min * (usual - zero) if usual > zero else None

    @staticmethod
    def shg_levels(ch: Channel):
        """Its SHG intra-cavity signal, V: (its average over the last LEVEL_S while armed, else its last poll; its
        level locked, 0 while not known; the limit its average must stay above, armed, None if there is no
        telling); None if it is not watched."""
        hist = list(ch.shg_hist)
        if not hist and ch.shg_level is None:
            return None
        usual = ch.hop.shg_lock
        now = sum(v for _, v in hist) / len(hist) if hist else ch.shg_level
        return now, usual, ch.hop.lock_min * usual if usual > 0 else None

    def _running_fine(self, ch: Channel) -> bool:
        """As it should be, as far as can be told: on target, and its locks saying locked: its DLC pro's lock to a
        cavity (watched or not: what it transmits locked is known when Relock is clicked), or this program's to the
        wavemeter (on target for a while); and its SHG cavity's."""
        hop = ch.hop
        if ch.state == "recovering" or not hop.enabled or not ch.device:
            return False
        if hop.cavity:
            if not (ch.cavity_on and says_locked(ch.cavity_txt)):
                return False
        elif ch.beat_slave:  # (an SLS PLL holds it: holding it on its beat point, its SHG cavity locked)
            v = ch.sls_view
            return (v is not None and bool(v.holds) and ch.sls_since is not None and ch.steering is None
                    and not self._beat_lost(ch) and (not hop.shg or says_locked(ch.shg_txt)))
        elif not (self._armed(ch) and ch.guard.armed):
            return False
        e = self._hop_error(ch) - ch.guard.track
        return math.isfinite(e) and abs(e) <= hop.off_target() and (not hop.shg or says_locked(ch.shg_txt))

    def _usual(self, ch: Channel, fine: deque, name: str, v: float):
        """A level locked as usual (HopParams.trans_locked, shg_lock), polled running as it should: the median of
        the last FINE_N such polls, once there are FINE_MIN (until then, as learned before: by a relock, say)."""
        fine.append(v)
        if len(fine) >= FINE_MIN:
            if not getattr(ch.hop, name):
                self.dirty = True
            setattr(ch.hop, name, round(statistics.median(fine), 5))

    def _armed(self, ch: Channel) -> bool:
        """Watched for hops (and for losing lasing): locked to the wavemeter by this program, or Relock
        armed while its DLC pro has its cavity lock on (the reference laser: always, it has no button). Not a laser
        an SLS unit locks: its guard calls its recoveries."""
        hop = ch.hop
        if not hop.enabled or not ch.device or ch.sls.on:
            return False
        if hop.cavity:
            return ch.state == ("off" if self._is_reference(ch) else "on") and bool(ch.cavity_on)
        return ch.state == "on"

    def _on_seed(self, ch: Channel, key: str, laser: int, name: str, v: float):
        """A poll of its seed power. Too little twice in a row, armed: recover it at once (it begins by
        jumping the current up), whatever the hourly limit. Lasing as usual and on target: learn how
        much seed that is."""
        with self.lock:
            if ch.device != key or ch.laser != laser or ch.seed_name != name:
                return
            hop = ch.hop
            ch.seed = v
            least = hop.seed_least(hop.seed_ok, ch.seed_floor)
            ch.lasing = None if least <= 0 else v >= least
            if ch.state == "recovering" or not hop.enabled:
                ch.seed_low = 0
                return
            if ch.lasing is False:
                ch.seed_low += 1
                if ch.seed_low >= 2 and self._armed(ch):
                    ch.seed_low = 0
                    log.warning("channel %d: not lasing, seed power %.2f mW", ch.n, v)
                    self._start_recovery(ch, f"stopped lasing (seed {v:.2f} mW)", budget=False, urgent=True)
                return
            ch.seed_low = 0
            e = self._hop_error(ch)
            if self._armed(ch) and ch.guard.armed and math.isfinite(e) and abs(e) <= hop.off_target() and v > 0:
                if not hop.seed_ok:
                    hop.seed_ok = round(v, 3)  # lasing as usual: this much
                    self.dirty = True
                else:
                    hop.seed_ok = round(0.98 * hop.seed_ok + 0.02 * v, 3)

    # ---------------------------------------------------------- lock dropouts
    DROP_SETTLE_S = 5.0  # s after its lock went on (or a recovery ended) before dropouts count

    def _watch_drops(self, ch: Channel):
        """A dropout monitor for a laser its DLC pro locks, while HopParams.dropouts says so."""
        hop, key = ch.hop, (ch.device, ch.laser)
        want = bool(ch.device) and hop.enabled and hop.cavity and hop.dropouts
        dt = min(max(hop.drop_ms, 0.05), 100.0) * 1e-3
        m = self.monitors.get(ch.n)
        if m is not None and (not want or m.key != key or abs(m.dt - dt) > 1e-9):
            m.stop()
            del self.monitors[ch.n]
            m = None
        if want and m is None:
            d = Device.from_key(ch.device)
            m = DropoutMonitor(d.ip, d.port, ch.laser, status=lambda: self._drop_status(ch, key),
                               report=lambda drop: self._dropped(ch, key, drop),
                               summary=lambda text: self._drop_summary(ch, key, text),
                               name=f"channel {ch.n}" + (f" {ch.name}" if ch.name else ""), dt=dt)
            m.key = key
            self.monitors[ch.n] = m
            m.start()

    def _drop_status(self, ch: Channel, key) -> tuple:
        """For its dropout monitor: (should its lock hold, the limit its transmission counts below (None: not
        known yet), its transmission's signal id). Not while it is off, not reporting locked, or being recovered:
        the monitor stops its recorder then (the DLC pro refuses a recovery's scope traces and its lock going on
        as busy now and then while it records)."""
        with self.lock:
            if (ch.device, ch.laser) != key:
                return False, None, None
            if not ch.cavity_on or not says_locked(ch.cavity_txt) or ch.state == "recovering":
                ch.hold_since = None
                return False, None, ch.trans_sig
            now = time.monotonic()
            if ch.hold_since is None:
                ch.hold_since = now
            lv = self.trans_levels(ch)
            return now - ch.hold_since >= self.DROP_SETTLE_S, lv[3] if lv else None, ch.trans_sig

    def _dropped(self, ch: Channel, key, d):
        """A dropout, from its monitor: said in the console (at most one a second, and how many in between), and
        appended to drop_path."""
        with self.lock:
            if (ch.device, ch.laser) != key:
                return
            ch.drops.append((d.t, d.duration))
            while ch.drops and d.t - ch.drops[0][0] > 3600.0:
                ch.drops.popleft()
            ch.drops_total += 1
            for o in ch.observers:
                o.drop(d.t, d.duration)
            usual = ch.hop.trans_locked
            how = f"{d.duration * 1e3:.1f} ms" if d.duration < 1.0 else f"{d.duration:.2f} s"
            if d.method == "polling":
                how += f" (polled every {d.resolution * 1e3:.0f} ms)"
            ch.drop_last = (f"{time.strftime('%H:%M:%S', time.localtime(d.t))} {how}, transmission down to "
                            f"{volts(d.low, usual)} V")
            at = time.strftime("%H:%M:%S", time.localtime(d.t)) + f".{int(d.t % 1 * 1000):03d}"
            text = (f"lock dropout {how}: transmission down to {volts(d.low, usual)} V (counts below "
                    f"{volts(d.limit, usual)} V; locked {volts(usual, usual)} V), at {at}; {len(ch.drops)} in the last "
                    f"hour")
            say = d.t - ch.drop_said >= 1.0  # (by when they happened: they are read a few seconds' worth at a time)
            if say:
                if ch.drop_unsaid:
                    text += f" ({ch.drop_unsaid} more since the last one said here)"
                ch.drop_said, ch.drop_unsaid = d.t, 0
            else:
                ch.drop_unsaid += 1
            who = f"channel {ch.n}" + (f" {ch.name}" if ch.name else "")
            rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(d.t)) + f".{int(d.t % 1 * 1000):03d}",
                   "channel": ch.n, "name": ch.name, "duration_ms": round(d.duration * 1e3, 3),
                   "low_v": round(d.low, 5), "limit_v": round(d.limit, 5), "locked_v": usual, "method": d.method,
                   "resolution_ms": round(d.resolution * 1e3, 4)}
        if say:
            drop_log.warning("%s: %s", who, text)
        if self.drop_path:
            try:
                with open(self.drop_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, separators=(",", ":"), ensure_ascii=False) + "\n")
            except OSError as exc:
                drop_log.warning("cannot log the dropout: %s", exc)

    def _drop_summary(self, ch: Channel, key, figures: str):
        """Once an hour, from its monitor: how many dropouts, and how it watches."""
        with self.lock:
            if (ch.device, ch.laser) != key:
                return
            now = time.time()
            while ch.drops and now - ch.drops[0][0] > 3600.0:
                ch.drops.popleft()
            n, total = len(ch.drops), ch.drops_total
            longest = max((s for _, s in ch.drops), default=0.0)
            who = f"channel {ch.n}" + (f" {ch.name}" if ch.name else "")
        since = time.strftime("%Y-%m-%d %H:%M", time.localtime(self.started))
        most = f"{longest * 1e3:.1f} ms" if longest < 1.0 else f"{longest:.2f} s"
        drop_log.info("%s: %d lock dropout%s in the last hour%s (%d since %s); watched by %s", who, n,
                      "" if n == 1 else "s", f", the longest {most}" if n else "", total, since, figures)

    # ---------------------------------------------------------- an SLS unit locks it (sls.py)
    SLS_STALE_S = 2.0   # s: a unit's poll older than this (it does not answer) tells nothing
    BEAT_FRESH_S = 2.0  # s: its master's reading goes with its own if at most this old
    BEAT_LEARN_S = 10.0  # s its PLL must have held before its beat is learned from the readings
    TRIM_HOLD_S = 3.0    # s its PLL must have held before its piezo is trimmed...
    TRIM_S = 2.5         # ...its fast output looked at this long before each step (TRIM_N readings at least; a
    TRIM_N = 5           # straight line through them, it drifting)...
    TRIM_SETTLE_S = 0.5  # ...from this long after the step before
    TRIM_DEAD = 0.02     # V: its fast output this near 0 on average (or within 3 times its uncertainty), left as it is
    TRIM_FRAC = 0.5      # of its fast output's average taken off at a step, its piezo moved at most TRIM_STEP V and
    TRIM_STEP = 0.05     # its fast output at most TRIM_FAST V by it (as learned: its guard's band moves along, well
    TRIM_FAST = 0.02     # within its width even if what was learned is twice what it is); twice that with its fast
    TRIM_FAR = 0.1       # output this far off
    TRIM_PROBE = 0.002   # V: a step to learn by how its fast output follows its piezo (every other one the other way,
    TRIM_PROBE_MAX = 0.032  # doubled while it hardly shows, up to this)...
    CUR_PROBE = 0.02     # mA: ...likewise its current (for an upkeep: how its piezo follows it, its frequency kept)...
    CUR_PROBE_MAX = 0.16  # ...up to this
    TRIM_SURE = 8.0      # ...learned once what they did together is this many times its uncertainty
    TRIM_SHOWS = 0.002   # V: a step moved its fast output at least this much (and 4 times its uncertainty), or it did
    #                      not (what is learned kept up to date by those that did)
    BEAT_STEP_S = 6.0    # s a slave read off its beat point, its PLL holding it by all it shows, has to read steady
    STEP_N = 10          # there (this many readings at least, STEP_MIN_S at least): its beat moved (a wavemeter
    STEP_MIN_S = 2.0     # recalibrated), learned again; not steady by then, it is lost. STEPS of them in STEPS_S at
    STEPS, STEPS_S = 3, 600.0  # most (more: no recalibration, something is wrong)
    BAND_STALE_S = 3.0   # s its PLL's outputs may read far from where they sat holding it, the wavemeter reading it
    #                      on its beat point all along, before where they sit is learned afresh
    BEAT_MIN = 20       # readings it is learned from, at least

    def set_sls(self, n: int, params: SlsParams):
        """What locks channel n's laser: an SLS unit (params.kind 'pdh', 'pll'), or none (kind '')."""
        with self.lock:
            ch = self.channels[n]
            old = ch.sls
            if ch.state != "off" and (params.kind != old.kind or not params.same_unit(old) or not params.on):
                self._disengage(ch, "")
            if (params.kind != "pll" or (params.master, params.master_on) != (old.master, old.master_on)
                    or not params.same_unit(old)):
                ch.beat_seen.clear()
                ch.beat_ok = False
            if params.kind != "pll" or (params.master, params.master_on) != (old.master, old.master_on):
                ch.mtrack = Track()
            if params.kind != "pll" or not params.same_unit(old):
                ch.fast_seen.clear()
                ch.phase_seen.clear()
            ch.sls = params
            if ch.sls_guard is not None:
                ch.sls_guard.p = params
            self._watch_sls(ch)
            self.layout_version += 1
            self.dirty = True

    def _watch_sls(self, ch: Channel):
        """Its unit polled by a watch of its own while it has one (a new watch for a new unit)."""
        w = self.sls_watches.get(ch.n)
        if w is not None and (not ch.sls.on or not w.p.same_unit(ch.sls)):
            w.stop()
            del self.sls_watches[ch.n]
            w = None
            ch.sls_view, ch.sls_error, ch.sls_since, ch.sls_held_t, ch.sls_count = None, "", None, None, None
            if ch.sls_drop is not None:
                self._sls_drop_over(ch, time.time())
        if w is not None:
            w.p = ch.sls  # (its settings, as they are now)
        elif ch.sls.on:
            name = f"channel {ch.n}" + (f" {ch.name}" if ch.name else "")
            w = SlsWatch(ch.sls, step=lambda watch, view, fresh: self._sls_step(ch, watch, view, fresh),
                         wrote=lambda watch, action, err: self._sls_wrote(ch, watch, action, err), name=name)
            w.seen = 0.0  # time.time(): the last poll it went by
            self.sls_watches[ch.n] = w
            w.start()
        self._watch_master(ch)

    def _watch_master(self, ch: Channel):
        """A PLL's slave: its master's SLS servo polled from here, by a watch of its own, while it has one."""
        p = ch.sls
        want = SlsParams(kind="pdh", host=p.master_host.strip(), port=p.master_port) if (
            ch.beat_slave and p.master_host.strip()) else None
        w = self.master_watches.get(ch.n)
        if w is not None and (want is None or not w.p.same_unit(want)):
            w.stop()
            del self.master_watches[ch.n]
            w = None
            ch.mview, ch.m_error, ch.m_since = None, "", None
        if want is not None and w is None:
            name = f"channel {ch.n}'s master" + (f" ({ch.name}'s)" if ch.name else "")
            w = SlsWatch(want, step=lambda watch, view, fresh: self._master_step(ch, watch, view, fresh), name=name)
            self.master_watches[ch.n] = w
            w.start()

    def _master_step(self, ch: Channel, watch, view, fresh: bool) -> list:
        """From the watch of a slave's master's SLS servo: its lock as it says (lost: the slave's watch looks at
        once)."""
        wake = None
        with self.lock:
            if self.master_watches.get(ch.n) is not watch:
                return []
            ch.m_error = watch.error
            if view is not None and fresh:
                was = ch.mview.holds if ch.mview is not None else None
                ch.mview = view
                if view.holds:
                    if ch.m_since is None:
                        ch.m_since = view.t - view.held if view.held is not None and view.held >= 0 else view.t
                else:
                    ch.m_since = None
                if was and view.holds is False:
                    wake = self.sls_watches.get(ch.n)
        if wake is not None:
            wake.wake()
        return []

    def _arm_sls(self, ch: Channel):
        """Relock clicked on a laser an SLS unit locks: watched (a PLL's slave, guarded: if its PLL does not hold it on
        its beat point now, it is relocked now)."""
        ch.state, ch.message = "on", ""
        ch.token += 1
        ch.upkeep_t = time.monotonic()
        ch.trim_moved, ch.trim_said, ch.trim_last = 0.0, "", None
        hop = ch.hop
        if hop.enabled and ch.device and not ch.dac and not hop.optimum:  # (as on a laser its DLC pro locks: its
            self._take_optimum(ch)                                         # optimum, the first time)
        if ch.beat_slave and ch.sls.guard:
            ch.sls_guard = BeatGuard(ch.sls)
            w = self.sls_watches.get(ch.n)
            if w is not None:
                w.wake()

    def _follows(self, ch: Channel) -> bool:
        """Its setpoint is its beat point (its master's reading plus the beat), not its own."""
        return ch.beat_slave and bool(ch.sls.master) and ch.sls.beat_known

    @staticmethod
    def holds_of(ch: Channel) -> tuple:
        """Whether a laser's lock holds, as far as this program knows (its SLS unit's last poll, else its state): (True,
        False, None: no telling), and why not. Reads its channel as it is (no lock needed: another engine's, too)."""
        v = ch.sls_view
        if ch.sls.on and v is not None and not ch.sls_error and time.time() - v.t <= Engine.SLS_STALE_S:
            return v.holds, v.why
        if ch.state == "recovering":
            return False, "being recovered"
        if ch.lasing is False and ch.hop.enabled:
            return False, "not lasing"
        return (True, "") if ch.state == "on" else (None, "")

    def _master_look(self, ch: Channel):
        """A PLL's slave's master as it is now, on this wavemeter or elsewhere (Elsewhere.row's dict; None: none, or not
        found); its reading followed (Channel.mtrack)."""
        p = ch.sls
        if not p.master:
            row = None
        elif not p.master_on:
            m = self.channels.get(p.master)
            if m is None or m is ch:
                row = None
            else:
                holds, why = self.holds_of(m)
                row = {"f": m.freq if m.freq > 0 else None, "t": m.t if m.t > 0 else None, "holds": holds, "why": why,
                       "name": m.name, "label": ""}
        else:
            row = self.elsewhere.row(p.master_on, p.master) if self.elsewhere is not None else None
        ch.master_row = row
        if row is not None:
            ch.mtrack.feed(row["f"], row["t"])
        lost = row is not None and row["holds"] is False
        if lost and not ch.master_lost and ch.sls_guard is not None:
            w = self.sls_watches.get(ch.n)
            if w is not None:
                w.wake()  # (a poll of a slow unit cut short: its PLL released now)
        ch.master_lost = lost
        return row

    def _beat_point(self, ch: Channel, look: bool = True):
        """THz: where its PLL holds it, its master's reading (the median of its last few, the last of the last
        BEAT_FRESH_S) plus the beat; with no master, its setpoint. None: not known now. look: its master looked at
        anew."""
        p = ch.sls
        if not p.master:
            return ch.f_set if ch.f_set > 0 else None
        if look:
            self._master_look(ch)
        f = ch.mtrack.point(time.monotonic(), self.BEAT_FRESH_S)
        return f + p.beat * 1e-6 if f is not None and p.beat_known else None

    def _beat_reading(self, ch: Channel):
        """A reading of a laser a PLL beat-locks: its beat learned (its PLL holding it), how far it is from its beat
        point (two in a row too far: its guard releases it), and, steered there, the wavemeter lock's step."""
        p, v = ch.sls, ch.sls_view
        row = self._master_look(ch)
        mono = time.monotonic()
        if (row is not None and row["f"] and row["t"] is not None and ch.freq > 0
                and mono - row["t"] <= self.BEAT_FRESH_S and v is not None and v.holds and ch.sls_since is not None
                and ch.steering is None and time.time() - ch.sls_since >= self.BEAT_LEARN_S and not ch.off_seen
                and (ch.sls_guard is None or ch.sls_guard.stage in ("hold", "auto"))):
            ch.beat_seen.append((ch.freq - row["f"]) * 1e6)  # (kept learned: two wavemeters' disagreement drifts)
            if len(ch.beat_seen) >= self.BEAT_MIN:
                beat = round(statistics.median(ch.beat_seen), 2)
                moved = not p.beat_known or abs(beat - p.beat) > 0.3
                if not ch.beat_ok:
                    was = f" (it was {p.beat:+.2f} MHz)" if p.beat_known and moved else ""
                    log.info("channel %d: its beat with its master learned: %+.2f MHz%s", ch.n, beat, was)
                    ch.beat_ok = True
                if moved:
                    p.beat, p.beat_known = beat, True
                    self.dirty = True
        point = self._beat_point(ch, look=False)
        if point is None or not ch.freq > 0:
            ch.beat_e, ch.beat_off_n, ch.near_n = None, 0, 0
            self._beat_step(ch, None)
        else:
            e = (ch.freq - point) * 1e6
            ch.beat_e = e
            ch.beat_off_n = ch.beat_off_n + 1 if abs(e) > p.off_mhz and ch.beat_ok else 0
            self._beat_step(ch, e)
            if not ch.beat_off_n:
                point = self._beat_point(ch, look=False)  # (its beat learned again, just now)
            near = ch.steering is not None and abs(e - ch.steering) <= p.capture
            ch.near_n = ch.near_n + 1 if near else 0
            if p.master:  # (its setpoint: its beat point, or where it is steered)
                ch.f_set = point + (ch.steering or 0.0) * 1e-6
                ch.setpoint_nm = C_NM_THZ / ch.f_set
        if ch.steering is not None and ch.lock.active and ch.freq > 0 and point is not None:
            self._steer_step(ch, point)

    def _beat_step(self, ch: Channel, e):
        """A reading e (MHz from its beat point; None: none) of a slave its PLL holds by all it shows (its own
        signals, holding, not just engaged): off its beat point and steady there (STEP_N readings, spread no more than
        three times as its readings less its master's do), its beat moved (one of the wavemeters recalibrated):
        learned again; still moving after BEAT_STEP_S, it counts (lost)."""
        p, v, g = ch.sls, ch.sls_view, ch.sls_guard
        holding = (v is not None and v.holds and not v.band and ch.sls_since is not None
                   and (g is None or g.stage in ("hold", "auto")))
        if e is None or ch.beat_off_n < 2 or not holding:
            ch.off_seen.clear()
            ch.off_t, ch.beat_step = None, False
            return
        mono = time.monotonic()
        if ch.off_t is None:
            ch.off_t = mono
        ch.off_seen.append(e)
        recent = sum(1 for t in ch.steps if mono - t <= self.STEPS_S)
        if (len(ch.off_seen) >= self.STEP_N and mono - ch.off_t >= self.STEP_MIN_S and recent < self.STEPS
                and self._steady(ch.off_seen, max(3 * self._beat_spread(ch), 0.5))):
            old = p.beat
            p.beat = round(old + statistics.median(ch.off_seen), 2)
            ch.beat_seen.clear()
            ch.off_seen.clear()
            ch.off_t, ch.beat_step, ch.beat_off_n = None, False, 0
            ch.steps.append(mono)
            self.dirty = True
            ch.sls_note = (f"its beat moved from {old:+.2f} to {p.beat:+.2f} MHz, its PLL holding it all along (one of "
                           f"the wavemeters recalibrated?): learned again")
            ch.sls_note_t = mono
            log.warning("channel %d: %s", ch.n, ch.sls_note)
            return
        ch.beat_step = mono - ch.off_t <= self.BEAT_STEP_S

    @staticmethod
    def _steady(values, tol: float) -> bool:
        """Readings that sit still: their spread (robust) within tol, and no drift (their two halves' medians within
        tol of each other)."""
        x = list(values)
        m = statistics.median(x)
        half = len(x) // 2
        return (1.4826 * statistics.median(abs(v - m) for v in x) <= tol
                and abs(statistics.median(x[half:]) - statistics.median(x[:half])) <= tol)

    @staticmethod
    def _beat_spread(ch: Channel) -> float:
        """MHz: how much its readings less its master's spread, its PLL holding it (1 MHz until known)."""
        seen = list(ch.beat_seen)
        if len(seen) < Engine.BEAT_MIN:
            return 1.0
        m = statistics.median(seen)
        return 1.4826 * statistics.median(abs(x - m) for x in seen)

    @staticmethod
    def _beat_lost(ch: Channel) -> bool:
        """The wavemeter reads it off its beat point (two readings in a row), and that counts (not while a step of
        its beat is told from a loss)."""
        return ch.beat_off_n >= 2 and not ch.beat_step

    def _steer(self, ch: Channel, mhz: float):
        """The wavemeter lock brings it to its beat point plus mhz (its PLL released), from where its piezo is."""
        ch.steering, ch.steer_fail, ch.near_n = mhz, "", 0
        ch.lock.disengage("")
        if not ch.device:
            ch.steer_fail = "no laser linked to steer it with"
            return
        ch.token += 1
        tok, key, laser, on_dac = ch.token, ch.device, ch.laser, ch.dac

        def job(client):
            piezo = ACTUATOR if on_dac else self._drives(ch, client, key, laser)[0]
            v = client.get_float(f"laser{laser}:{piezo}")
            with self.lock:
                if ch.token == tok and ch.steering is not None and ch.piezo == piezo:
                    ch.offset = v
                    ch.lock.engage(v)

        def failed(exc):
            with self.lock:
                if ch.token == tok and ch.steering is not None:
                    ch.steer_fail = f"cannot steer it: {exc}"

        self._controller(key).submit(job, failed)

    def _steer_step(self, ch: Channel, point: float):
        """The wavemeter lock's step, steering it to point (THz) plus Channel.steering."""
        u = ch.lock.update((ch.freq - point) * 1e6 - ch.steering)
        if not ch.lock.active:
            ch.steer_fail = ch.lock.message or "the wavemeter lock let go"
            return
        if u is not None:
            ch.offset = u
            tok = ch.token

            def failed(exc):
                with self.lock:
                    if ch.token == tok and ch.steering is not None:
                        ch.steer_fail = f"cannot steer it: {exc}"

            self._controller(ch.device).set(ch.param, u, failed)

    def _unsteer(self, ch: Channel, why: str = ""):
        """No more steering (its piezo left where the wavemeter lock had it)."""
        ch.steering, ch.near_n = None, 0
        ch.steer_fail = why
        ch.lock.disengage("")
        c = self.controllers.get(ch.device)
        if c is not None:
            c.cancel(ch.param)

    def _sls_look(self, ch: Channel, view, fresh: bool) -> Look:
        """What its guard is told now."""
        now, mono = time.time(), time.monotonic()
        master = master_s = None
        mv, row = ch.mview, self._master_look(ch)
        if mv is not None and not ch.m_error and now - mv.t <= self.SLS_STALE_S and mv.holds is not None:
            master = mv.holds  # (its SLS servo, polled from here)
            master_s = now - ch.m_since if mv.holds and ch.m_since is not None else 0.0
        elif row is not None and row["holds"] is not None:
            master = row["holds"]  # (as its row says: its own PC watching its lock)
            master_s = ch.mtrack.steady_s(mono, self.BEAT_FRESH_S) if row["holds"] else 0.0
            if master and master_s is None:
                master_s = 0.0
        else:
            master_s = ch.mtrack.steady_s(mono, self.BEAT_FRESH_S)  # (not watched: its readings steady that long)
        far = ""
        cap = ch.lock.p.capture if ch.lock.p.capture > 0 else 3000.0
        if ch.beat_e is not None and abs(ch.beat_e) > cap:
            far = f"{ch.beat_e / 1e3:+.3f} GHz from its beat point"
        elif not ch.off and ch.t > 0 and not ch.freq > 0 and mono - ch.t <= self.BEAT_FRESH_S and (
                row is None or row["f"]):
            far = f"no reading ({ch.status or 'none'}): it may not lase"
        hop = ch.hop
        k = Look(now=now, armed=ch.sls_guard is not None and ch.state in ("on", "recovering"),
                 pll=view if view is not None and now - view.t <= self.SLS_STALE_S and not ch.sls_error else None,
                 fresh=fresh, master=master, master_s=master_s,
                 beat_off=(f"{ch.beat_e:+.1f} MHz from its beat point" if self._beat_lost(ch) and ch.beat_e is not None
                           else ""),
                 near_n=ch.near_n, lasing=ch.lasing, far=far,
                 can_steer=bool(ch.device) and self._beat_point(ch, look=False) is not None and not ch.off,
                 can_recover=hop.enabled and bool(ch.device) and not ch.dac, recovering=ch.runner is not None,
                 recovered=ch.recovered)
        if ch.steer_fail and ch.sls_guard is not None and ch.sls_guard.stage == "steer":
            k.far = k.far or ch.steer_fail
        ch.recovered = None
        return k

    def _sls_step(self, ch: Channel, watch, view, fresh: bool) -> list:
        """From its watch (its thread): its unit's poll (fresh), or a look at once (woken), or none (no answer). Its
        lock's dropouts kept; a PLL's slave armed, its guard's step: what to do here done, the writes returned for
        the watch to make."""
        wake = []
        with self.lock:
            if self.sls_watches.get(ch.n) is not watch:
                return []
            ch.sls_error = watch.error
            if view is not None and fresh:
                was = ch.sls_view.holds if ch.sls_view is not None else None
                ch.sls_view = view
                if ch.beat_slave:
                    self._outputs_band(ch, view)
                self._sls_track(ch, view)
                if ch.beat_slave:
                    self._trim(ch, view)
                watch.seen = view.t
                if (ch.beat_slave and view.holds and ch.sls_since is not None and ch.steering is None
                        and ch.beat_off_n == 0 and view.t - ch.sls_since >= self.BEAT_LEARN_S
                        and (ch.sls_guard is None or ch.sls_guard.stage in ("hold", "auto"))):
                    self._learn_outputs(ch, view)
                if ch.sls.kind == "pdh" and was and view.holds is False:  # its slaves here told at once
                    wake = [w for k, w in self.sls_watches.items()
                            if self.channels[k].beat_slave and self.channels[k].sls.master == ch.n
                            and not self.channels[k].sls.master_on]
            elif watch.error:
                watch.seen = time.time()  # (nothing more will be known of it for a while)
            out = []
            if ch.beat_slave and ch.sls_guard is not None:
                g = ch.sls_guard
                k = self._sls_look(ch, view, fresh)
                for a in g.step(k):
                    out += self._sls_act(ch, g, a)
                if ch.sls_guard is g and ch.state in ("on", "recovering") and ch.runner is None:
                    state = "recovering" if g.relocking else "on"
                    if state == "recovering" and ch.state == "on":  # (a relock begins)
                        ch.hop_why, ch.hop_t = g.why, time.monotonic()
                    ch.state = state
        for w in wake:
            w.wake()
        return out

    def _trim(self, ch: Channel, view):
        """A PLL with its slow servo unused (SlsParams.trim), holding its slave, its guard armed and holding it (or
        an upkeep under way): its fast output looked at over TRIM_S, and nulled bit by bit by the slave's piezo (a
        step every few s, as if its slow servo, very slow). How its fast output follows the piezo is learned by small
        steps first (what each did by itself, a drift taken off: step_fit), and kept learned by every step after (the
        other way twice in a row: learned again); how it follows its current likewise, wanted for an upkeep."""
        p, g, mono = ch.sls, ch.sls_guard, time.monotonic()
        on = (p.trim and ch.device and not ch.dac and view.holds and view.mode and view.fast is not None
              and ch.sls_since is not None and time.time() - ch.sls_since >= self.TRIM_HOLD_S and g is not None
              and g.stage in ("hold", "auto") and ch.steering is None and not ch.lock.active
              and (ch.runner is None or ch.upkeeping))
        if not on:
            ch.trim_seen = []
            ch.trim_from, ch.trim_last, ch.trim_level = mono, None, math.inf
            return
        if ch.trim_busy or mono < ch.trim_from:
            return
        ch.trim_seen.append((view.t, view.fast))
        if mono - ch.trim_from < self.TRIM_S or len(ch.trim_seen) < self.TRIM_N:
            return
        seen, ch.trim_seen, ch.trim_from = ch.trim_seen, [], mono  # (the next look begins now, unless a step comes)
        ys = [y for _, y in seen]
        mean, sem = statistics.fmean(ys), statistics.stdev(ys) / math.sqrt(len(ys))
        ch.trim_level = mean
        last, ch.trim_last = ch.trim_last, None
        if last is not None and not ch.upkeeping:  # (an upkeep moves its current and piezo meanwhile: nothing learned)
            self._trim_learn(ch, last[1], last[2], *step_fit(last[0], seen)[:2])
        if ch.trim_cur_want and p.trim_gain and not ch.upkeeping:
            ch.trim_sign = -ch.trim_sign
            self._trim_step(ch, "current", ch.trim_sign * (ch.cur_size or self.CUR_PROBE), seen, mean)
        elif not p.trim_gain:
            ch.trim_sign = -ch.trim_sign
            self._trim_step(ch, "piezo", ch.trim_sign * (ch.trim_size or self.TRIM_PROBE), seen, mean)
        elif abs(mean) > max(self.TRIM_DEAD, 3.0 * sem):
            cap = min(self.TRIM_STEP, self.TRIM_FAST / abs(p.trim_gain)) * (2.0 if abs(mean) >= self.TRIM_FAR else 1.0)
            self._trim_step(ch, "piezo", min(max(-self.TRIM_FRAC * mean / p.trim_gain, -cap), cap), seen, mean)

    def _trim_learn(self, ch: Channel, kind: str, moved: float, step: float, noise: float):
        """The step before (kind 'piezo': moved V of its piezo; 'current': mA of its current) moved its fast output
        step V by itself, give or take noise: learned (SlsParams.trim_gain, trim_cur). Not learned yet, the steps to
        learn by are taken together (each by how well it tells) until what they did is clear (TRIM_SURE: one that
        noise happened to make look big does not make it), bigger while it hardly shows; learned, every step that
        shows keeps it up to date."""
        p, piezo = ch.sls, kind == "piezo"
        name = "trim_gain" if piezo else "trim_cur"
        old = getattr(p, name)
        noise = max(noise, self.TRIM_SHOWS / 4)
        if not old:
            ch.trim_probes = [x for x in ch.trim_probes if x[0] == kind][-7:] + [(kind, moved, step, noise)]
            sxx = sum((m / s) ** 2 for _, m, _, s in ch.trim_probes)
            gain = sum(m * c / s ** 2 for _, m, c, s in ch.trim_probes) / sxx
            sure = abs(gain) * math.sqrt(sxx)  # (how many times its uncertainty)
            if sure < self.TRIM_SURE or abs(gain * moved) <= self.TRIM_SHOWS:
                if sure < 4.0 and piezo:
                    ch.trim_size = min(2 * abs(moved), self.TRIM_PROBE_MAX)
                    if abs(moved) >= self.TRIM_PROBE_MAX - 1e-9 and len(ch.trim_probes) >= 4:
                        ch.trim_said = (f"its fast output does not follow its piezo (steps of ±"
                                        f"{self.TRIM_PROBE_MAX * 1e3:.0f} mV): is the PLL's fast output on its "
                                        f"current, and that the slave's piezo?")
                elif sure < 4.0:
                    ch.cur_size = min(2 * abs(moved), self.CUR_PROBE_MAX)
                return
            setattr(p, name, round_sig(gain, 4))
            ch.trim_probes, ch.trim_odd = [], 0
            if not piezo:
                ch.trim_cur_want = False
            ch.trim_said = (f"learned: its fast output moves {gain:+.4g} V for a "
                            f"{'V of its piezo' if piezo else 'mA of its current'}")
            log.info("channel %d: its piezo trim: %s", ch.n, ch.trim_said)
            self.dirty = True
            return
        if abs(step) <= max(4.0 * noise, self.TRIM_SHOWS):  # (it did not show)
            return
        gain = step / moved
        if gain * old < 0:
            ch.trim_odd += 1
            if ch.trim_odd >= 2:
                ch.trim_said = (f"its fast output went the other way ({gain:+.4g} V for a "
                                f"{'V of its piezo' if piezo else 'mA of its current'}, learned {old:+.4g}): learned "
                                f"again")
                log.warning("channel %d: its piezo trim: %s", ch.n, ch.trim_said)
                setattr(p, name, 0.0)
                ch.trim_size = ch.cur_size = 0.0
                ch.trim_odd, ch.trim_probes = 0, []
                self.dirty = True
        else:
            ch.trim_odd = 0
            seen = min(max(gain / old, 1 / 3), 3.0) * old  # (one odd look: not too much)
            setattr(p, name, round_sig(0.7 * old + 0.3 * seen, 4))

    def _trim_step(self, ch: Channel, kind: str, d: float, seen: list, mean: float):
        """Its piezo (kind 'piezo': d V, within its lock's limits) or its current ('current': d mA) moved from where
        it is now (read afresh: no step from a stale value), in its controller's thread; where its PLL's fast output
        sits holding it moved along by what was learned (its guard's band)."""
        key, laser, tok, p, piezo = ch.device, ch.laser, ch.token, ch.sls, kind == "piezo"
        param = ch.param if piezo else f"laser{laser}:{ch.cc}"
        lo, hi = (ch.lock.p.v_min, ch.lock.p.v_max) if piezo else (0.0, math.inf)
        what = "its piezo" if piezo else "its current"
        ch.trim_busy = True

        def job(client):
            v = client.get_float(param)
            new = round(min(max(v + d, lo), hi), 5)
            if abs(new - v) > 1e-9:
                client.set(param, new)
            with self.lock:
                ch.trim_busy = False
                moved = new - v
                if ch.device != key or ch.laser != laser:
                    return
                if piezo:
                    ch.offset = new
                    ch.trim_moved += moved
                else:
                    ch.current_set = new
                self._shift_band(ch, (p.trim_gain if piezo else p.trim_cur) * moved)
                if ch.token != tok:
                    return
                if abs(moved) < 1e-9:
                    ch.trim_said = f"{what} at its limit, {new:.3f} {'V' if piezo else 'mA'}: no trim"
                    return
                ch.trim_last = (seen, kind, moved)
                ch.trim_from = time.monotonic() + self.TRIM_SETTLE_S
                ch.trim_seen = []
                if piezo and p.trim_gain:
                    ch.trim_said = f"{moved * 1e3:+.1f} mV, its fast output {mean:+.3f} V on average"

        def failed(exc):
            with self.lock:
                ch.trim_busy = False
                ch.trim_said = f"cannot move {what}: {exc}"

        self._controller(key).submit(job, failed)

    @staticmethod
    def _shift_band(ch: Channel, d: float):
        """Its PLL's fast output moved d V (expected, by a step of its trim's): where it sits holding it moves along
        (else its guard would see it far from there, and release it)."""
        if not d:
            return
        ch.fast_seen = deque((x + d for x in ch.fast_seen), maxlen=ch.fast_seen.maxlen)
        p = ch.sls
        if p.fast_dev:
            p.fast_mid = round(p.fast_mid + d, 6)

    @staticmethod
    def _piezo_per_ma(ch: Channel) -> float:
        """V its piezo moves for a mA of its current to keep its frequency (its PLL's fast output where it is), as
        its trim learned (0: not yet)."""
        p = ch.sls
        return -p.trim_cur / p.trim_gain if p.trim_gain and p.trim_cur else 0.0

    def _outputs_band(self, ch: Channel, view):
        """Where its PLL's outputs sit holding it, learned afresh when its PLL is off or relocked (engaged again,
        they may sit elsewhere), or when they read far from there for BAND_STALE_S while the wavemeter reads it on
        its beat point all along (they moved, and it holds it there)."""
        relocked = view.count is not None and ch.sls_count is not None and view.count > ch.sls_count
        if not view.mode or relocked:
            ch.band_t = None
            self._forget_outputs(ch)
        elif view.band and ch.beat_e is not None and ch.beat_off_n == 0:
            if ch.band_t is None:
                ch.band_t = view.t
            elif view.t - ch.band_t >= self.BAND_STALE_S:
                log.info("channel %d: %s, the wavemeter reading it on its beat point all along: where its PLL's "
                         "outputs sit learned afresh", ch.n, view.why)
                ch.band_t = None
                self._forget_outputs(ch)
        else:
            ch.band_t = None

    @staticmethod
    def _forget_outputs(ch: Channel):
        """Its PLL holding it anew: where its fast output and its phase error sit, learned afresh (engaged again, by
        hand or by its guard, they may sit elsewhere)."""
        ch.fast_seen.clear()
        ch.phase_seen.clear()
        p = ch.sls
        p.fast_mid = p.fast_dev = p.phase_mid = p.phase_dev = 0.0

    def _learn_outputs(self, ch: Channel, view):
        """A PLL holding its slave: what it shows learned (its beat amplitude; how far its fast output and its phase
        error go from where they sit: far beyond, it has lost the beat)."""
        p = ch.sls
        if view.amp is not None:
            ch.amp_seen.append(view.amp)
            if len(ch.amp_seen) >= self.BEAT_MIN:
                amp = round(statistics.median(ch.amp_seen), 3)
                if abs(amp - p.amp_usual) > 0.05 * amp:
                    p.amp_usual = amp
                    self.dirty = True
        for seen, value, mid, dev in ((ch.fast_seen, view.fast, "fast_mid", "fast_dev"),
                                      (ch.phase_seen, view.phase, "phase_mid", "phase_dev")):
            if value is None:
                continue
            seen.append(value)
            if len(seen) < self.BEAT_MIN:
                continue
            m = statistics.median(seen)
            spread = sorted(abs(x - m) for x in seen)
            d = max(spread[min(int(0.98 * len(spread)), len(spread) - 1)], 1e-6)
            old = getattr(p, dev)
            if not old or abs(d - old) > 0.25 * old or abs(m - getattr(p, mid)) > 0.5 * old:
                setattr(p, mid, round(m, 6))  # (not kept in its settings: SlsParams.FRESH)
                setattr(p, dev, round(d, 6))

    def _sls_act(self, ch: Channel, g: BeatGuard, a: tuple) -> list:
        """One of its guard's actions: done here, or (a write to its PLL) returned. An upkeep under way stops first."""
        what = a[0]
        mono = time.monotonic()
        if ch.upkeeping:
            self._end_upkeep(ch, a[1] if len(a) > 1 and isinstance(a[1], str) else what)
        if what == "release":
            if ch.steering is not None:
                self._unsteer(ch)
            ch.sls_note, ch.sls_note_t = f"its PLL released: {a[1]}", mono
            log.warning("channel %d: its PLL released: %s", ch.n, a[1])
            return [a]
        if what == "engage":
            self._forget_outputs(ch)  # (engaged again, its outputs may sit elsewhere)
            return [a]
        if what == "steer":
            self._steer(ch, a[1])
        elif what == "unsteer":
            self._unsteer(ch)
        elif what == "recover":
            log.warning("channel %d: %s, its PLL released: recovered", ch.n, a[1])
            self._start_recovery(ch, a[1], budget=False, urgent=bool(a[2]))
        elif what == "held":
            ch.message = ""
            took = mono - ch.hop_t if ch.hop_t else 0.0
            ch.sls_note = f"relocked: its PLL holds it again ({ch.hop_why or 'relock'}, {took:.0f} s)"
            ch.sls_note_t = mono
            log.info("channel %d: %s", ch.n, ch.sls_note)
        elif what == "gave_up":
            if ch.steering is not None:
                self._unsteer(ch)
            ch.sls_note, ch.sls_note_t = f"gave up for now: {a[1]}", mono
            log.warning("channel %d: beat lock: %s; tries again in %.0f s", ch.n, a[1], g.RETRY_S)
        elif what == "disarm":
            self._disengage(ch, f"watching stopped: {a[1]}")
        return []

    def _sls_wrote(self, ch: Channel, watch, action: tuple, err: str):
        """Its watch made a write its guard asked for (err: why it did not take, '' it did)."""
        with self.lock:
            if self.sls_watches.get(ch.n) is not watch or not err:
                return
            ch.sls_note, ch.sls_note_t = f"its PLL: {action[0]} not done: {err}", time.monotonic()

    def _sls_track(self, ch: Channel, v):
        """A poll of its unit: its lock's dropouts. One begins at the last poll that held, ends at the first that holds
        again; a relock counted between two polls that both held is one between them (as its unit says when)."""
        g = ch.sls_guard
        relocking = g is not None and g.relocking
        held = bool(v.holds) and not relocking and not (ch.beat_slave and self._beat_lost(ch))
        expect = bool(v.mode) or (g is not None and g.stage != "idle")  # (its lock on, or its guard has it: switched
        #                                                                  off by hand, it is no dropout)
        if held:
            anew = ch.sls_since is None
            if ch.sls_drop is not None:
                self._sls_drop_over(ch, v.t)
            elif (ch.sls_count is not None and v.count is not None and v.count > ch.sls_count
                  and ch.sls_held_t is not None):  # (relocked between two polls)
                end = v.t - v.held if v.held is not None and ch.sls_held_t < v.t - v.held < v.t else v.t
                self._sls_dropped(ch, ch.sls_held_t, end, f"relocked between two polls (its count {ch.sls_count} → "
                                                          f"{v.count})")
                ch.sls_since, anew = end, True
            if ch.sls_since is None:  # (by its unit's own count of how long it has held, if it has one)
                ch.sls_since = max(v.t - v.held, ch.sls_bad_t or 0.0) if v.held is not None and v.held >= 0 else v.t
            if anew and ch.beat_slave:
                self._forget_outputs(ch)
            ch.sls_held_t = v.t
        else:
            ch.sls_since, ch.sls_bad_t = None, v.t
            if expect and ch.sls_drop is None:
                why = (g.why if relocking and g.why else v.why or (
                    f"the wavemeter reads it {ch.beat_e:+.1f} MHz from its beat point" if ch.beat_e is not None
                    else "not holding"))
                ch.sls_drop = (ch.sls_held_t if ch.sls_held_t is not None else v.t, why)
            elif not expect and ch.sls_drop is not None:  # (its lock switched off: over there)
                self._sls_drop_over(ch, v.t)
        ch.sls_count = v.count

    def _sls_drop_over(self, ch: Channel, t1: float):
        t0, why = ch.sls_drop
        ch.sls_drop = None
        self._sls_dropped(ch, t0, t1, why)

    def _sls_dropped(self, ch: Channel, t0: float, t1: float, why: str):
        """A dropout of the lock an SLS unit has on it, t0 to t1 (time.time(), by its polls): kept, told its
        observations, said (at most one a second), logged to drop_path."""
        s = max(t1 - t0, 0.0)
        ch.drops.append((t0, s))
        while ch.drops and t1 - ch.drops[0][0] > 3600.0:
            ch.drops.popleft()
        ch.drops_total += 1
        for o in ch.observers:
            o.drop(t0, s)
        how = f"{s * 1e3:.0f} ms" if s < 1.0 else f"{s:.1f} s"
        unit = f"{'its PLL' if ch.sls.kind == 'pll' else 'its SLS servo'} {ch.sls.unit}"
        at = time.strftime("%H:%M:%S", time.localtime(t0)) + f".{int(t0 % 1 * 1000):03d}"
        ch.drop_last = f"{time.strftime('%H:%M:%S', time.localtime(t0))} {how}: {why}"
        text = f"lock dropout {how} ({unit}: {why}), at {at}; {len(ch.drops)} in the last hour"
        say = t0 - ch.drop_said >= 1.0
        if say:
            if ch.drop_unsaid:
                text += f" ({ch.drop_unsaid} more since the last one said here)"
            ch.drop_said, ch.drop_unsaid = t0, 0
        else:
            ch.drop_unsaid += 1
        who = f"channel {ch.n}" + (f" {ch.name}" if ch.name else "")
        if say:
            drop_log.warning("%s: %s", who, text)
        if self.drop_path:
            rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t0)) + f".{int(t0 % 1 * 1000):03d}",
                   "channel": ch.n, "name": ch.name, "duration_ms": round(s * 1e3, 1), "method": f"sls-{ch.sls.kind}",
                   "unit": ch.sls.unit, "why": why,
                   "resolution_ms": round(SlsWatch.POLL_S * 1e3, 1)}
            try:
                with open(self.drop_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, separators=(",", ":"), ensure_ascii=False) + "\n")
            except OSError as exc:
                drop_log.warning("cannot log the dropout: %s", exc)

    def _sls_recovered(self, ch: Channel, why: str) -> bool:
        """A mode-hop recovery its guard called is over (why: '' it recovered it, else how not): its guard goes on
        with it. True if so."""
        g = ch.sls_guard
        if g is None or g.stage != "recover":
            return False
        ch.recovered = why
        ch.state, ch.pending = "recovering", None
        ch.hop_why = g.why
        self._release(ch)
        w = self.sls_watches.get(ch.n)
        if w is not None:
            w.wake()
        return True

    def _controller(self, key: str) -> Controller:
        c = self.controllers.get(key)
        if c is None:
            if dac.is_dac(key):
                c = self.controllers[key] = Controller(key, client=dac.DacClient(key, self.dacs))
            else:
                d = Device.from_key(key)
                c = self.controllers[key] = Controller(key, d.ip, d.port)
            c.start()
        return c

    def where(self, ch: Channel) -> str:
        """Its laser, as its row says it: 'laser1 @ Ca+ blue', 'AO 3 @ USB-3106 (MCC board 0)'."""
        if ch.dac:
            c = self.controllers.get(ch.device)
            name = c.client.name if c is not None else dac.title(ch.device)
            ff = f" (+ AO {ch.ff_ao} fed forward)" if ch.ff_ao is not None else ""
            return f"AO {ch.laser}{ff} @ {name}"
        dev = self.devices.get(ch.device)
        return f"laser{ch.laser} @ {dev.name if dev else ch.device}"

    def set_output(self, n: int, volts: float, ff: bool = False):
        """An analog output set by hand (its feed-forward output, ff), within its lock's limits; not while its lock
        has it. Returns the voltage asked for (None: not done)."""
        with self.lock:
            ch = self.channels[n]
            ao = ch.ff_ao if ff else ch.laser
            if not ch.dac or ao is None:
                return None
            if ch.state != "off":
                ch.message = "set by hand: not while it is locked"
                return None
            p = ch.lock.p
            v = round(min(max(float(volts), p.v_min), p.v_max), 6)
            key = ch.device

            def done():
                with self.lock:
                    if ch.device == key and not ff:
                        ch.offset = v
                    if ch.message.startswith(("cannot lock", "set by hand", "output")):
                        ch.message = ""

            def failed(exc):
                with self.lock:
                    if ch.device == key:
                        ch.message = f"output {ao}: {exc}"

            self._controller(key).set(f"laser{ao}:{ACTUATOR}", v, on_error=failed, on_done=done)
            return v

    # -------------------------------------------------------------- reference
    def set_reference(self, n: int):
        """Use the laser on channel n, locked to a stable cavity, as the reference.

        Taking over from another reference keeps the correction continuous."""
        with self.lock:
            old = self.reference
            if old is not None and old.channel == n:
                return
            ch = self.channels[n]
            if ch.state != "off":
                self._disengage(ch, "")
            ch.message = ""
            carry = old.eps_offset() if old is not None and old.active else None
            self.reference = Reference(n, old.p if old else RefParams(), carry=carry)
            self.dirty = True

    def clear_reference(self):
        with self.lock:
            self.reference = None
            self.dirty = True

    def set_reference_params(self, params: RefParams):
        with self.lock:
            if self.reference is not None:
                self.reference.p = params
                self.dirty = True

    def set_reference_anchor(self, thz: float):
        """The reference laser's true frequency is known: calibrate everything against it."""
        with self.lock:
            if self.reference is not None:
                self.reference.set_anchor(thz, time.monotonic())
                self.dirty = True

    def reference_accept_drift(self) -> bool:
        with self.lock:
            ok = self.reference is not None and self.reference.accept_as_drift(time.monotonic())
            self.dirty |= ok
            return ok

    def reference_reanchor(self) -> bool:
        with self.lock:
            ok = self.reference is not None and self.reference.reanchor(time.monotonic())
            self.dirty |= ok
            return ok

    def locked_shift_mhz(self, correct) -> float:
        """How far the locked lasers would be moved if correct(raw THz) replaced the present correction."""
        with self.lock:
            return max((abs(correct(ch.raw) - ch.freq) * 1e6 for ch in self.channels.values()
                        if ch.state == "on" and ch.raw > 0 and not ch.relock and not ch.sls.on), default=0.0)

    def tick(self):
        """Call now and then from the GUI: keeps the reference's state current when readings stop,
        and times out recoveries that wait in vain."""
        with self.lock:
            now = time.monotonic()
            if self.dacs.changed and now - self.dacs.changed > 30.0:  # (analog outputs moved: saved now and then)
                self.dirty = True
            if self.reference is not None:
                self.reference.tick(now)
            for ch in list(self.channels.values()):
                if ch.runner is not None:
                    ch.runner.tick(now)
                if ch.mapper is not None:
                    ch.mapper.tick(now)
            self._timers(now)

    # ------------------------------------------------------------- laser maps
    def channel_rate(self, n: int) -> float:
        """Readings a second the wavemeter gives of channel n, of late (0: none)."""
        ch = self.channels[n]
        gaps = sorted(ch.gaps)
        return 1.0 / gaps[len(gaps) // 2] if gaps and gaps[len(gaps) // 2] > 0 else 0.0

    def map_estimate(self, n: int, plan: MapPlan) -> tuple:
        """(s, bytes) a map of channel n would take, roughly, at the rate the wavemeter reads it now."""
        info = getattr(self.source, "pattern_info", {}) or {}
        raw = sum(count * size for count, size in info.values()) or 8192
        return plan.estimate(self.channel_rate(n) or 20.0, 0.5 * raw)

    def map_look(self, n: int):
        """Read what a map would use of channel n's laser (its current, piezo, temperature, their limits, its
        feedforward, its TA, its lock), for the dialog: Channel.map_info, a dict by name ({} if it cannot be read),
        None until then."""
        with self.lock:
            ch = self.channels[n]
            ch.map_info = None
            if not ch.device or ch.dac:
                ch.map_info = {}
                return
            key, laser = ch.device, ch.laser

            def job(client):
                piezo, cc = self._drives(ch, client, key, laser)
                names = (CURRENT_ACT, cc, piezo, *LOOK)
                got = dict(zip(("current_act", "current_set", "piezo", *LOOK), perform(client, laser, ("get", names))))
                with self.lock:
                    if ch.device == key and ch.laser == laser:
                        ch.map_info = got

            def failed(exc):
                with self.lock:
                    ch.map_info = {"error": str(exc) or type(exc).__name__}

            self._controller(key).submit(job, failed)

    def start_map(self, n: int, plan: MapPlan) -> str:
        """Map channel n's laser (lasermap.py): '' if it begins, else why not."""
        with self.lock:
            ch = self.channels[n]
            if not ch.device:
                return "link a laser first"
            if ch.dac:
                return "not a laser on an analog output: a map needs a DLC pro"
            if self._is_reference(ch):
                return "not the reference laser"
            if ch.state != "off":
                return {"mapping": "it is being mapped already", "recovering": "it is recovering: stop that first"}.get(
                    ch.state, "unlock it first (or stop watching it)")
            bad = plan.problems()
            if bad:
                return "; ".join(bad)
            path = Path(plan.folder or FOLDER) / file_name(n, ch.name)
            try:
                out = MapFile(path)
            except (OSError, sqlite3.Error) as e:
                return f"cannot write {path}: {e}"
            dev = self.devices.get(ch.device)
            info = {"laser": {"channel": n, "name": ch.name, "device": ch.device, "laser": ch.laser,
                              "controller": dev.title() if dev else ch.device, "piezo": ch.piezo, "current": ch.cc},
                    "wavemeter": {"model": getattr(self.source, "model", ""),
                                  "serial": getattr(self.source, "serial", None)}}
            mapper = Mapper(plan, out, info)
            ch.map_plan, ch.map_note, ch.map_last, ch.map_pending = plan, "", None, mapper
            self.dirty = True
            ch.lock.disengage("")
            c = self._controller(ch.device)
            c.cancel(ch.param)
            ch.state, ch.message = "mapping", ""
            ch.token += 1
            ch.guard.reset()
            ch.look_why = ch.look_due = ""
            if ch.off:  # no readings: its wavemeter channel on for the map
                ch.borrowed = True
                self._switch(ch, True)
            if getattr(self.source, "details", False):
                self.source.detail(n, plan.patterns != "off")
            tok, key, laser = ch.token, ch.device, ch.laser
            log.info("channel %d: map begins, to %s", n, path)

            def job(client):
                self._drives(ch, client, key, laser)  # (what its scan drives now: its piezo, its current)
                with self.lock:
                    if ch.token == tok:
                        ch.map_pending = None
                        runner = MapRunner(mapper, _HopIO(self, ch, c), time.monotonic(),
                                           finished=lambda r: self._map_done(ch, r),
                                           exposure=lambda: self.source.exposure(n) if getattr(
                                               self.source, "details", False) else None)
                        if runner.busy:
                            ch.mapper = runner

            def failed(exc):
                with self.lock:
                    if ch.token == tok:
                        ch.map_pending = None
                        self._map_over(ch, mapper, f"no map, cannot read the laser's settings: {exc}")

            c.submit(job, failed)
            return ""

    def stop_map(self, n: int, why: str = "stopped"):
        """Stop channel n's map: it goes back where it began (one about to begin: it does not)."""
        with self.lock:
            ch = self.channels[n]
            if ch.state == "mapping":
                self._disengage(ch, why)

    def _map_done(self, ch: Channel, r: MapRunner):
        ch.mapper = None
        ch.map_last = r
        m = r.mapper
        took = fmt_time(r.now - r.started)
        if r.state == "done":
            text = f"map done in {took}: {r.result}"
        else:
            where = "everything put back" if r.restored else "left as it was when it stopped"
            text = f"map {'stopped' if r.error == 'stopped' else 'failed, ' + r.error} after {took} ({where})"
        self._map_over(ch, m, text, r)

    def _map_over(self, ch: Channel, m: Mapper | None, text: str, r: MapRunner | None = None):
        """The map over (r: its runner, None if it never began): the channel is as before, the file closed."""
        ok = r is not None and r.state == "done"
        ch.token += 1
        self._release(ch)
        if getattr(self.source, "details", False):
            self.source.detail(ch.n, None)
        out = m.out if m is not None else None
        if out is not None:
            out.meta("end", {"outcome": text, "ok": ok, "error": r.error if r else text,
                             "restored": r.restored if r else True, "points": m.points, "lasing": m.lasing,
                             "patterns_kept": m.kept, "f_end": m.f_end, "local": time.strftime("%Y-%m-%d %H:%M:%S"),
                             "took_s": round(r.now - r.started, 1) if r else 0.0, "notes": m.notes[-200:],
                             "fits": m.fits, "feedforward": m.ff_state(),
                             "pattern_info": {str(k): v for k, v in (getattr(self.source, "pattern_info", {}) or
                                                                     {}).items()}})
            closing = threading.Thread(target=out.close, daemon=True, name="map file close")
            closing.start()
            text += f"; saved to {out.path}"
            if m.plan.model and m.lasing >= MODEL_MIN:
                m.model_note = "fitting the laser's model…"
                threading.Thread(target=self._model_of, args=(ch, m, out.path, closing), daemon=True,
                                 name="laser model").start()
        ch.map_note, ch.message = text, "" if ok else text
        ch.state = "off"  # (last: off, its note is there to read)
        (log.info if ok else log.warning)("channel %d: %s", ch.n, text)

    MODEL_GOOD = 0.6  # a laser's model putting fewer of its map's points on the right mode is not gone by

    def _model_of(self, ch: Channel, m: Mapper, path: Path, closing: threading.Thread):
        """The laser's model from its map's file, once that is written (lasermodel.py): kept in the file, its facts
        in the map's notes (Mapper.model; model_note: how it went), and with the channel (Channel.adopt_model) for
        its recoveries to go by, if it describes the laser well enough (MODEL_GOOD), and it is still linked to the
        laser mapped."""
        closing.join()
        try:
            from . import lasermodel
            from .lasermap import load

            with load(path) as f:
                model = lasermodel.fit(f)
            lasermodel.save(path, model)
        except ImportError as exc:
            m.model_note = f"no model of the laser: it needs numpy and scipy ({exc})"
        except Exception as exc:  # (whatever the map: never in the way)
            log.exception("laser model of %s", path)
            m.model_note = f"no model of the laser: {exc or type(exc).__name__}"
        else:
            m.notes.append((None, "the laser, as its model has it:"))
            m.notes.extend((None, "  " + fact) for fact in model.facts())
            m.model, m.model_note = model, f"its model: right mode at {model.right:.0%} of the points"
            laser = m.info.get("laser") or {}
            doubts = model.doubts()
            with self.lock:
                if doubts:
                    m.notes.append((None, "its recoveries do not go by it: " + "; ".join(doubts)))
                elif model.right < self.MODEL_GOOD:
                    m.notes.append((None, f"its recoveries do not go by it ({model.right:.0%}: too little)"))
                elif [laser.get("device"), laser.get("laser")] == [ch.device, ch.laser]:
                    ch.adopt_model(model, path.name)
                    self.dirty = True
                    m.notes.append((None, "its recoveries go by it from now on (Mode-hop recovery's method: auto "
                                          "or model)"))
            log.info("map %s: %s", path, m.model_note)
            return
        m.notes.append((None, m.model_note))
        log.warning("map %s: %s", path, m.model_note)

    def model_from_file(self, n: int, path) -> str:
        """Its laser's model from a map's file (lasermap.py): the one kept in it, or fitted to it now, in a thread of
        its own (minutes; Channel.model_note says how it goes). Its recoveries go by it from then on, if the map is of
        the laser the channel is linked to (its controller and laser, as the map has them) and the model describes it
        well enough (MODEL_GOOD). Returns what it did, or is doing."""
        path = Path(path)
        try:
            from . import lasermodel
            from .lasermap import load

            with load(path) as f:
                laser, model = f.meta.get("laser") or {}, lasermodel.stored(f)
                old = lasermodel.outdated(f, model) if model is not None and model.plausible else None
        except ImportError as exc:
            return f"its laser's model needs numpy and scipy ({exc})"
        except Exception as exc:  # (not a map, say)
            return f"cannot read {path.name}: {exc or type(exc).__name__}"
        with self.lock:
            ch = self.channels[n]
            if [laser.get("device"), laser.get("laser")] != [ch.device, ch.laser]:
                return (f"{path.name} is a map of laser {laser.get('laser')} of {laser.get('device') or '?'}, not of "
                        f"the laser channel {n} is linked to: its model not taken")
        if model is not None and model.plausible and old is None:
            return self._take_model(ch, model, path.name)
        why = (f"the model it keeps is {old}: a model is being fitted to it again" if old else
               f"the model it keeps is not this laser's ({'; '.join(model.doubts())}): a model is being fitted to it "
               f"again" if model is not None else "it keeps no model of the laser: one is being fitted to it")

        def fit():
            try:
                with load(path) as f:
                    got = lasermodel.fit(f)
                lasermodel.save(path, got)
                note = self._take_model(ch, got, path.name)
            except Exception as exc:  # (whatever the map: never in the way)
                log.exception("laser model of %s", path)
                note = f"none from {path.name}: {exc or type(exc).__name__}"
                with self.lock:
                    ch.model_note = note
            log.info("channel %d: its laser's model: %s", ch.n, note)

        with self.lock:
            ch.model_note = f"being fitted to {path.name}…"
        threading.Thread(target=fit, daemon=True, name="laser model").start()
        return (f"{path.name}: {why}, in the background (a few minutes); Mode-hop recovery… shows it once it is taken, "
                f"and the row's tooltip says how it went")

    def _take_model(self, ch: Channel, model, source: str) -> str:
        """Its laser's model from now on, if it describes the laser well enough (MODEL_GOOD): what was done."""
        doubts = model.doubts()
        with self.lock:
            if doubts:
                ch.model_note = f"the one from {source} is not this laser's: " + "; ".join(doubts)
            elif model.right < self.MODEL_GOOD:
                ch.model_note = (f"the one from {source} puts only {model.right:.0%} of that map's points on the right "
                                 f"mode: not gone by")
            else:
                ch.adopt_model(model, source)
                self.dirty = True
                ch.model_note = (f"from {source}, {model.right:.0%} of that map's points on the right mode: its "
                                 f"recoveries go by it from now on (Mode-hop recovery's method: auto or model)")
            return ch.model_note

    # ------------------------------------------------------- mode-hop recovery
    def set_hop_params(self, n: int, params: HopParams):
        with self.lock:
            ch = self.channels[n]
            if ch.dac:
                params.enabled = False  # (mode-hop recovery needs a DLC pro)
            if ch.state == "on" and (params.cavity != ch.hop.cavity or (params.cavity and not params.enabled)):
                self._disengage(ch, "")  # locked to the wavemeter, or watched, as it no longer should be
            ch.hop = params
            ch.guard.reset()
            ch.guard.track = 0.0
            ch.trans_fine.clear()  # (learned afresh if forgotten)
            ch.shg_fine.clear()
            if ch.device:
                self._watch_hop(ch)
                if params.enabled and not params.i_nom:
                    self._read_nominal(ch)
            self.dirty = True

    def _read_nominal(self, ch: Channel):
        """The present current, as measured, becomes the nominal one, which the current stays near."""
        key, laser = ch.device, ch.laser

        def job(client):
            v = client.get_float(f"laser{laser}:{CURRENT_ACT}")
            with self.lock:
                if ch.device == key and ch.laser == laser and not ch.hop.i_nom:
                    ch.hop.i_nom, ch.current = round(v, 3), v
                    self.dirty = True

        self._controller(key).submit(job)

    def take_optimum(self, n: int, shg=None):
        """The laser as it is now is at its optimum (set by hand): its current, piezo voltage and SHG signal locked
        are what the catch goes back to. shg: it has an SHG stage (None: as its settings say; the dialog says as it
        is ticked there, not yet OK'd)."""
        with self.lock:
            ch = self.channels[n]
            if not ch.device:
                ch.message = "right-click to link a laser first"
            elif ch.dac:
                ch.message = "take the optimum: not on an analog output"
            elif ch.state in ("recovering", "mapping"):
                ch.message = f"take the optimum: not while it {'recovers' if ch.state == 'recovering' else 'is mapped'}"
            else:
                self._take_optimum(ch, shg)

    def _take_optimum(self, ch: Channel, shg=None):
        key, laser = ch.device, ch.laser
        shg = ch.hop.shg if shg is None else bool(shg)

        def job(client):
            piezo, _ = self._drives(ch, client, key, laser)
            got = perform(client, laser, ("get", (CURRENT_ACT, piezo) + ((SHG_STATE, SHG_INTRA) if shg else ())))
            with self.lock:
                if ch.device != key or ch.laser != laser:
                    return
                act, v = got[0], got[1]
                if not (_num(act) and _num(v)):
                    ch.message = "cannot read its current and offset: the optimum is not taken"
                    return
                hop = ch.hop
                hop.opt_i, hop.opt_v, hop.i_nom = round(act, 3), round(v, 4), round(act, 3)
                hop.opt_t = time.strftime("%Y-%m-%d %H:%M")
                level = got[3] if shg and says_locked(got[2]) and _num(got[3]) and got[3] > 0 else 0.0
                hop.opt_shg = round(level, 4)
                ch.opt_why = ("" if level else "no SHG stage ticked" if not shg else
                              f"its SHG cavity says “{got[2] or '?'}”, not locked" if not says_locked(got[2]) else
                              f"its SHG signal reads {got[3]!r}")
                self.dirty = True
                ch.note = (f"optimum taken: {act:.2f} mA, {v:.2f} {ch.unit}" + (
                    f", SHG {level:.3f} V locked" if level else f", SHG level not known: {ch.opt_why}" if shg else ""))
                ch.note_t = time.monotonic()
                log.info("channel %d: %s", ch.n, ch.note)

        self._controller(key).submit(job)

    def recenter(self, n: int):
        """Recenter now, even without a hop: always maps, so it finds the best mode."""
        with self.lock:
            ch = self.channels[n]
            if ch.state in ("recovering", "engaging", "mapping"):
                return
            if not ch.device:
                ch.message = "right-click to link a laser first"
            elif ch.dac:
                ch.message = "recenter: not on an analog output (it needs a DLC pro)"
            elif not (math.isfinite(self._hop_error(ch)) or (ch.off and ch.f_set > 0)):
                ch.message = "recenter: needs a setpoint and a valid reading"
            else:
                self._start_recovery(ch, "", quick=False, budget=False)

    def cancel_recovery(self, n: int):
        with self.lock:
            ch = self.channels[n]
            if ch.state == "recovering":
                self._disengage(ch, "stopped")

    def _is_reference(self, ch: Channel) -> bool:
        return self.reference is not None and self.reference.channel == ch.n

    def _hop_error(self, ch: Channel) -> float:
        """MHz between the laser and where it belongs: its setpoint, or for the reference laser where
        the reference says it should read."""
        if self._is_reference(ch):
            return (ch.raw - self.reference.expected()) * 1e6 if ch.raw > 0 else NAN
        return (ch.freq - ch.f_set) * 1e6 if ch.freq > 0 else NAN

    def _hop_reading(self, ch: Channel, t: float) -> bool:
        """The mode-hop guard's look at a new reading. True: keep it from the wavemeter lock."""
        e = self._hop_error(ch)
        if ch.runner is not None:
            ch.runner.on_reading(t, e - ch.guard.track)
            return True
        if ch.sls.on:  # (an SLS unit locks it: its guard calls its recoveries)
            return False
        hop = ch.hop
        if not hop.enabled or not ch.device or ch.state == "recovering":
            return False
        if hop.cavity:  # watch it while its DLC pro has it locked, and it is armed (Relock)
            ref = self._is_reference(ch)  # the reference laser has no button: always armed
            if ch.state != ("off" if ref else "on") or not ch.cavity_on:
                ch.guard.reset()
                return False
            track = not ref  # its target moves with the wavemeter's drift
        else:  # watch it while this program has it locked to the wavemeter
            if ch.state != "on":
                ch.guard.reset()
                return False
            track = False
        if ch.guard.update(t, e, hop.off_target(), hop.grace, track):
            off = e - ch.guard.track
            self._start_recovery(ch, f"jumped {off / 1e3:+.3f} GHz" if abs(off) >= 1e3 else (
                f"locked {off:+.0f} MHz off: the wrong peak" if hop.cavity else f"off by {off:+.0f} MHz"))
            return True
        return ch.guard.armed and math.isfinite(e) and abs(e - ch.guard.track) > hop.off_target()

    def _start_recovery(self, ch: Channel, why: str, quick: bool = True, budget: bool = True, relock: bool = False,
                        urgent: bool = False):
        """quick: try the piezo alone first if the laser is near its target. budget: counts towards
        max_per_hour (those the guard starts). relock: asked to relock a laser its DLC pro locks to a
        cavity: take its lock over even if it is off now, and watch it afterwards. Its wavemeter channel
        switched off, it is switched on, and the recovery begins with readings (WAIT_S at most), unless
        urgent (it does not lase: its current jump does not wait)."""
        hop = ch.hop
        now = time.monotonic()
        if budget and not ch.guard.allow(now, hop.max_per_hour):
            ch.guard.reset()
            msg = f"{why}: {hop.max_per_hour} recoveries within the hour already, not another"
            if ch.state == "on":
                self._disengage(ch, msg)
            else:
                ch.message = msg
            return
        ch.resume = ch.state == "on" or relock
        ch.lock.disengage("")
        c = self._controller(ch.device)
        c.cancel(ch.param)
        ch.state, ch.message = "recovering", ""
        m = self.monitors.get(ch.n)
        if m is not None:
            m.nudge()  # (its recorder stops at once: the recovery needs the DLC pro's data acquisition)
        ch.token += 1
        ch.hop_why, ch.hop_t = why, now
        for d in (ch.trans_hist, ch.shg_hist, ch.trans_fine, ch.shg_fine) if not ch.upkeeping else ():
            d.clear()  # (what it relocked to is learned afresh)
        ch.look_why = ch.look_due = ch.wait_fail = ""
        ch.pending, ch.wait_since = None, now
        if ch.off:  # no readings: its wavemeter channel on first
            ch.borrowed = True
            self._switch(ch, True)
        tok, key, laser, cavity = ch.token, ch.device, ch.laser, hop.cavity
        log.info("channel %d: %s, recovering", ch.n, why or "recenter")

        def job(client):
            p = f"laser{laser}:"
            piezo, cc = self._drives(ch, client, key, laser)  # (what its scan drives now)
            i0 = client.get_float(p + cc)
            a0 = client.get_float(p + CURRENT_ACT)
            clip = client.get_float(p + "dl:cc:current-clip")
            v0 = client.get_float(p + piezo)
            locked = cavity and (relock or client.get_bool(p + CAVITY_LOCK))
            with self.lock:
                if ch.token == tok:
                    self._begin_when_read(ch, (i0, a0, clip, v0, locked, quick, c), urgent)

        def failed(exc):
            with self.lock:
                if ch.token == tok:
                    self._recovery_failed(ch, f"cannot read the laser's settings: {exc}")

        c.submit(job, failed)

    def _begin_when_read(self, ch: Channel, args, urgent: bool):
        """Begin the recovery (args: _begin_recovery's), or if its wavemeter channel was switched on for it, once
        that gives a valid reading (on_readings), WAIT_S at most (_wait_over); at once if urgent."""
        if ch.wait_fail and not urgent:
            self._recovery_failed(ch, ch.wait_fail)
        elif urgent or not ch.borrowed or (not ch.off and ch.t > ch.wait_since and ch.freq > 0):
            self._begin_recovery(ch, *args)
        else:
            ch.pending = args

    def _wait_over(self, ch: Channel, why: str = ""):
        """No valid reading within WAIT_S of switching its channel on for the recovery: it begins all the same
        if the channel is measured (the laser may not lase: the recovery finds out); else (or why: switching
        it on did not work) there is no recovery, nothing having been touched."""
        args, ch.pending = ch.pending, None
        if not why and not ch.off:
            self._begin_recovery(ch, *args)
            return
        self._recovery_failed(ch, why or f"its wavemeter channel is switched off and did not come on in "
                                         f"{self.WAIT_S:g} s")

    def _begin_recovery(self, ch: Channel, i0, a0, clip, v0, locked, quick, controller):
        """i0, a0: the set and the measured current; the window is about the nominal current, which
        is measured, and becomes one of set currents with their difference now (with the piezo where it
        is: the recovery has it follow the feedforward): i_below below it, i_window above, and jump_max
        above for a laser that does not lase. A current outside the window (left high where a laser that
        had stopped lasing lased again, say) is brought into it first."""
        hop, lp = ch.hop, ch.lock.p
        if not hop.i_nom:
            hop.i_nom = round(a0, 3)  # the current before the first recovery is the nominal one
            self.dirty = True
        off = a0 - i0
        i_lo, i_hi = max(hop.i_nom - hop.i_below - off, 0.0), hop.i_nom + hop.i_window - off
        i_top = hop.i_nom + max(hop.jump_max, hop.i_window) - off
        v_lo, v_hi = v0 - hop.v_window, v0 + hop.v_window
        if hop.how() == "catch" and hop.pc_try > 0:  # (its tries from the optimum's piezo voltage +- pc_try)
            vo = hop.opt_v if hop.optimum else v0
            v_lo, v_hi = min(v_lo, vo - hop.pc_try), max(v_hi, vo + hop.pc_try)
        v_lo, v_hi = max(lp.v_min, v_lo), min(lp.v_max, v_hi)
        fine = ch.fine_at
        ref = (fine[1], fine[2]) if fine and time.monotonic() - fine[0] <= self.FINE_S else None
        ch.current, ch.current_set, ch.offset = a0, i0, v0
        if not i_lo <= min(i_hi, clip):
            self._recovery_failed(ch, f"current-clip, {clip + off:.2f} mA measured, is below its window ("
                                      f"{i_lo + off:.2f} … {i_hi + off:.2f} mA): set the nominal current in "
                                      f"Mode-hop recovery…")
            return
        if not v_lo <= v0 <= v_hi:
            what = "the offset" if ch.piezo == ACTUATOR else f"its piezo voltage ({ch.piezo})"
            self._recovery_failed(ch, f"{what}, {v0:.2f} V, is outside {lp.v_min:g} … {lp.v_max:g} V")
            return
        model = ch.laser_model() if hop.method in ("auto", "model") and not ch.upkeeping else None
        rec = Recenter(hop, i0, v0, i_lo, i_hi, v_lo, v_hi, cavity=locked, quick=quick, shg=hop.shg, act_offset=off,
                       i_top=i_top, i_clip=clip, ref=ref, upkeep=ch.upkeeping, model=model, target=ch.f_set)
        io = _HopIO(self, ch, controller)
        runner = Runner(rec, io, time.monotonic(), timeout=rec.timeout, finished=lambda r: self._recovery_done(ch, r))
        if runner.busy:
            ch.runner = runner
            if ch.off:  # (begun at once, not lasing: its channel is being switched on)
                runner.blind(True)

    def _recovery_failed(self, ch: Channel, why: str):
        """Before anything was touched."""
        if self._sls_recovered(ch, why):
            return
        if ch.upkeeping:  # (an upkeep that did not begin: watched again)
            ch.upkeeping, ch.state = False, "off"
            ch.note, ch.note_t = f"upkeep: none, {why}", time.monotonic()
            ch.token += 1
            ch.pending = None
            self._arm(ch, check=False)
            return
        ch.state, ch.resume = "off", False
        ch.message = f"{ch.hop_why + ': ' if ch.hop_why else ''}no recovery, {why}"
        ch.token += 1
        ch.pending = None
        self._release(ch)

    def _recovery_done(self, ch: Channel, r: Runner):
        if getattr(r, "abandoned", False):  # (an upkeep its PLL's guard stopped to act itself: said there, _end_upkeep)
            return
        ch.runner = None
        ch.guard.reset()
        ch.state = "off"
        ch.token += 1
        upkeep, ch.upkeeping = ch.upkeeping, False
        ch.upkeep_t = time.monotonic()
        self._release(ch)  # its wavemeter channel, if it was switched on for the recovery
        took = f"{r.now - ch.hop_t:.0f} s"
        why = f"{ch.hop_why}: " if ch.hop_why else ""
        rec = r.rec
        if upkeep and ch.beat_slave and ch.sls_guard is not None:
            self._beat_upkept(ch, r, took)
            return
        if self._sls_recovered(ch, "" if r.state == "done" else r.error or "failed"):
            pass  # (its guard goes on with it; what it did is noted below all the same)
        if r.state == "done":
            res = r.result
            if res.kv:
                ch.hop.kv = round(res.kv, 1)
            if res.spacing:
                ch.hop.spacing = round(res.spacing, 1)
            for name, value in res.learned.items():
                if name in HopParams.LEARNED:
                    setattr(ch.hop, name, value)
            self.dirty = True
            ch.current_set, ch.offset = res.current, res.offset
            ch.message = ""
            ch.note, ch.note_t = f"{why}{res.summary}{self._recenter_target(ch, res.lock_e)}, {took}", time.monotonic()
            log.info("channel %d: %s", ch.n, ch.note)
            self._keep_record(ch, r, ch.note)
            if ch.resume:
                ch.resume = False
                if ch.relock and not self._is_reference(ch):
                    if rec.cavity:
                        ch.cavity_on = True  # it relocked it (the poll may not have seen it yet)
                    self._arm(ch, check=False)  # watch it again
                else:
                    self.engage(ch.n)  # bumpless, at the new offset
            return
        ch.resume = False
        ch.current_set = r.i_written
        if upkeep and r.error != "stopped":  # (an upkeep that went wrong: watched again, relocked if it must be)
            ch.note = f"upkeep: {r.error}" + ("" if r.restored else f" (left at {rec.mA(r.i_written):.3f} mA)")
            ch.note_t = time.monotonic()
            log.warning("channel %d: %s", ch.n, ch.note)
            self._keep_record(ch, r, ch.note)
            self._arm(ch, take=False)
            return
        if r.restored:
            where = "current and offset put back"
        else:
            where = f"left at {rec.mA(r.i_written):.2f} mA, {r.v_written:.2f} {ch.unit}"
        if "dl:lock:lock-enabled" in r.journal:
            where += ", cavity lock left off"
        for name in ("lp_code", "lp_kind", "peak_y", "falc_sign", "falc_gain", "trans_zero"):  # worth keeping, even so
            if name in rec.learned:
                setattr(ch.hop, name, rec.learned[name])
                self.dirty = True
        head = "recovery stopped" if r.error == "stopped" else f"{why}recovery failed, {r.error}"
        ch.message = f"{head} ({where})"
        log.warning("channel %d: %s", ch.n, ch.message)
        self._keep_record(ch, r, ch.message)

    def _beat_upkept(self, ch: Channel, r: Runner, took: str):
        """An upkeep of a laser an SLS PLL holds is over (its PLL held it on its beat point all along, its piezo moving
        with its current): its guard goes on with it, its piezo trimmed as before."""
        ch.state = "on"  # (its guard's to tell from here: _sls_step)
        rec = r.rec
        if r.state == "done":
            ch.current_set = r.result.current
            ch.message = ""
            ch.note = f"upkeep: {r.result.summary}, its piezo with it ({ch.offset:.3f} V), {took}"
        else:
            ch.current_set = r.i_written
            ch.note = f"upkeep: {r.error}" + ("" if r.restored else f" (left at {rec.mA(r.i_written):.3f} mA)")
        ch.note_t = time.monotonic()
        log.log(logging.INFO if r.state == "done" else logging.WARNING, "channel %d: %s", ch.n, ch.note)
        self._keep_record(ch, r, ch.note)

    def _end_upkeep(self, ch: Channel, why: str):
        """Its PLL's guard acts (its PLL released, say) during an upkeep: the upkeep stops at once, its current and
        piezo left where they are (the guard has it from here)."""
        r = ch.runner
        if r is None or not ch.upkeeping:
            return
        r.abandoned = True
        ch.runner, ch.upkeeping, ch.state = None, False, "on"
        ch.upkeep_t = time.monotonic()
        ch.token += 1
        r.cancel(f"its PLL's guard: {why}", keep=True)
        ch.note, ch.note_t = f"upkeep stopped: {why}", time.monotonic()
        log.warning("channel %d: %s", ch.n, ch.note)
        self._keep_record(ch, r, ch.note)

    def _recenter_target(self, ch: Channel, lock_e: float) -> str:
        """Relocked to its cavity, lock_e MHz from its target as the wavemeter reads it: the target (its setpoint)
        moved there, so that the lockpoint is the target in the wavemeter's numbers. Returns a note."""
        if not math.isfinite(lock_e) or self._is_reference(ch) or not ch.f_set > 0:
            return ""
        shift = lock_e + ch.guard.track  # MHz from the setpoint (the readings had the drift followed taken off)
        ch.set_setpoint(round_sig(C_NM_THZ / (ch.f_set + shift * 1e-6)))
        ch.guard.track = 0.0
        self.dirty = True
        return f"; target re-centered on the lock, {shift:+.1f} MHz"

    def _keep_record(self, ch: Channel, r: Runner, outcome: str):
        """What the recovery saw and did: kept for a look (Channel.last), and logged."""
        rec = r.rec
        ch.last = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"), "channel": ch.n, "name": ch.name,
            "why": ch.hop_why or "recenter now", "ok": r.state == "done", "outcome": outcome,
            "took_s": round(r.now - ch.hop_t, 1), "i_window": [round(rec.mA(rec.i_lo), 4), round(rec.mA(rec.i_hi), 4)],
            "i_top": round(rec.mA(rec.i_top), 4), "v_window": [rec.v_lo, rec.v_hi], "seed": ch.seed,
            "ref": [round(rec.v_ref, 4), round(rec.mA(rec.i_ref, rec.v_ref), 4)],
            "optimum": [ch.hop.opt_i, ch.hop.opt_v, ch.hop.opt_shg, ch.hop.opt_t] if ch.hop.optimum else None,
            "kv": rec.kv, "ff": rec.ff, "unit": ch.unit, **rec.record,
            "notes": [[None if t is None else round(t - r.started, 1), text] for t, text in rec.record["notes"]],
        }
        if self.log_path:
            try:
                with open(self.log_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(ch.last, separators=(",", ":"), ensure_ascii=False) + "\n")
            except (OSError, TypeError, ValueError) as exc:
                log.warning("cannot log the recovery: %s", exc)

    def confirm_step(self, n: int):
        """Step by step: go on with what the recovery is waiting to do."""
        with self.lock:
            ch = self.channels[n]
            if ch.runner is not None:
                ch.runner.confirm()

    def check_setup(self, n: int):
        """Read (only) everything a recovery would use on this laser, and say what looks wrong:
        Channel.setup, a list of (level, text), once read."""
        with self.lock:
            ch = self.channels[n]
            if not ch.device:
                ch.setup = [("error", "no laser linked")]
                return
            ch.setup = None
            key, laser, hop, lp, ff, on_dac = ch.device, ch.laser, ch.hop, ch.lock.p, ch.ff_ao, ch.dac
            wlm = self._channel_report(ch)
            model = ch.model_dict if ch.laser_model() is not None else None

            def job(client):
                if on_dac:  # (an analog output: the board, what its outputs were set to)
                    report = client.report(laser, ff) + [("info", f"lock: {lp.v_min:g} … {lp.v_max:g} V, at most "
                                                                  f"±{lp.excursion:g} V from where it locked")]
                else:
                    report = setup_report(client, laser, hop, lp, model) + wlm
                with self.lock:
                    if ch.device == key:
                        ch.setup = report

            def failed(exc):
                with self.lock:
                    ch.setup = [("error", f"cannot read the controller: {exc}")]

            self._controller(key).submit(job, failed)

    def _channel_report(self, ch: Channel) -> list:
        """For the setup check: its wavemeter channel, if someone switches it off."""
        if not ch.hop.enabled:
            return []
        out = [("head", f"Wavemeter channel {ch.n}")]
        if ch.off:
            out.append(("warn", "switched off now ('Use' unticked in the WLM software)"))
        if not getattr(self.source, "switchable", False):
            out.append(("info", "this wavemeter source cannot switch channels: switched off, the laser is unlocked "
                                "(watched no more)"))
        elif ch.relock:
            out.append(("info", "switched off by someone, the laser stays watched by what its DLC pro says (its "
                                "lock's state, transmission, SHG lock and signal, seed power); when that says "
                                "something happened, the channel is switched on for a look (and a relock if it is "
                                "off target), then off again. This needs wlmData.dll, or with --http server.py from "
                                "this version (an older one reports the channel as gone: unlocked)"))
        else:
            out.append(("info", "switched off by someone, its lock is held where it is, and goes on when the channel "
                                "is ticked again; a recovery switches it on, and off again after"))
        if ch.looked:
            out.append(("info", f"last look, its channel switched on for it: {ch.looked}"))
        return out

    # ------------------------------------------------- adjustments from other PCs
    def remote(self, n: int, do: str, args: dict, who: str) -> tuple:
        """An adjustment of channel n asked by another PC (share.py) or by ARTIQ (rpc.py), done as if clicked here:
        (done, why not). who: that PC, said on the row for a while (Channel.remote_note). Done but nothing changed:
        (True, 'unchanged'). Its lock observed (observe, observe_end, observed: observe.py, nothing changed): (done,
        why not, what it found)."""
        with self.lock:
            ch = self.channels.get(n)
            if ch is None or n not in self.rows():
                return False, f"no channel {n} shown there"
            if do in OBSERVING:
                return self._remote_observe(n, do, args)
            if ch.state == "mapping" and do not in ("rename", "lock_params"):
                return False, "it is being mapped (stopped there, it goes back where it began)"
            ref = self._is_reference(ch)
            found = None  # (what it says besides: the setpoint it had)
            if do == "setpoint":
                nm = args.get("nm")
                if ref:
                    return False, "the reference laser has no setpoint"
                if self._follows(ch):
                    return False, (f"its setpoint is its beat point ({ch.sls.master_where}'s reading "
                                   f"{ch.sls.beat:+.1f} MHz): an SLS PLL holds it there")
                if nm is not None and not (_num(nm) and nm > 0):
                    return False, f"not a wavelength: {nm!r}"
                if nm == ch.setpoint_nm:
                    return True, "unchanged", {"was": nm}
                found = {"was": ch.setpoint_nm}
                self.set_setpoint(n, nm)
                what = f"setpoint {nm} nm" if nm else "setpoint cleared"
            elif do == "lock":
                if ch.state != "off":
                    return True, "unchanged"
                if ref:
                    return False, "the reference laser is locked to its cavity, not to the wavemeter"
                self.engage(n)
                if ch.state == "off":
                    return False, ch.message or "not locked"
                what = "Relock clicked" if ch.relock or ch.sls.on else "locked"
            elif do == "unlock":
                if ch.state == "recovering":
                    return False, "it is recovering: stop that instead"
                if ch.state == "off":
                    return True, "unchanged"
                self.disengage(n)
                what = "watching stopped" if ch.relock or ch.sls.on else "unlocked"
            elif do == "stop":
                if ch.state != "recovering":
                    return True, "unchanged"
                self.cancel_recovery(n)
                what = "recovery stopped"
            elif do == "continue":
                prompt = ch.hop_prompt
                if not prompt:
                    return False, "it is not waiting for a step to be allowed any more"
                if prompt != args.get("prompt"):
                    return False, f"the step it waits for is another one now: “{prompt}”"
                self.confirm_step(n)
                what = f"step allowed: {prompt}"
            elif do == "setpoint_to_reading":
                if ref or not ch.freq > 0 or ch.state == "recovering":
                    return False, "no valid reading" if not ch.freq > 0 else "not now"
                self.setpoint_to_current(n)
                what = f"setpoint = reading, {ch.setpoint_nm} nm"
            elif do == "recenter":
                if ch.state in ("recovering", "engaging"):
                    return False, f"not while {ch.state}"
                self.recenter(n)
                if ch.state != "recovering":
                    return False, ch.message or "not recentered"
                what = "Recenter now"
            elif do == "rename":
                name = str(args.get("name", "")).strip()[:80]
                if name == ch.name:
                    return True, "unchanged"
                self.rename(n, name)
                what = f"renamed “{name}”"
            elif do == "lock_params":
                p = args.get("p")
                if not isinstance(p, dict) or not all(_num(x) for x in p.values()):
                    return False, "not lock settings"
                self.set_params(n, LockParams.from_dict({**ch.lock.p.to_dict(), **p}))  # (those not said: as they are)
                what = "lock settings changed"
            else:
                return False, f"cannot {do!r} from another PC"
            ch.remote_note, ch.remote_t, ch.remote_wall = f"{what} from {who}", time.monotonic(), time.time()
            return (True, "") if found is None else (True, "", found)

    def _remote_observe(self, n: int, do: str, args: dict) -> tuple:
        tag = str(args.get("tag") or "")[:64]
        if not tag:
            return False, "which observation (no tag)"
        if do == "observe":
            mhz = args.get("mhz")
            if mhz is not None and not (_num(mhz) and mhz > 0):
                return False, f"not MHz: {mhz!r}"
            why = self.observe(n, tag, mhz)
            return (False, why, None) if why else (True, "", None)
        if do == "observe_end":
            return (True, "", None) if self.end_observe(tag) else (False, "no such observation (over long ago?)", None)
        start, count = args.get("start", 0), args.get("count", 0)
        if not all(isinstance(x, int) and not isinstance(x, bool) and x >= 0 for x in (start, count)):
            return False, "not a part of it", None
        got = self.observed(tag, start, min(count or OBSERVED_PART, OBSERVED_PART))
        return (True, "", got) if got is not None else (False, "no such observation (over long ago?)", None)

    # -------------------------------------------------------------- lock observed (observe.py)
    def observe(self, n: int, tag: str, mhz=None) -> str:
        """Observe channel n's lock from now on, as tag (the caller's name for it): '' if it is, else why not."""
        with self.lock:
            ch = self.channels.get(n)
            if ch is None or n not in self.rows():
                return f"no channel {n} shown there"
            self._forget_observations()
            old = self.observations.pop(tag, None)
            if old is not None and old in ch.observers:
                ch.observers.remove(old)
            while len(self.observations) >= self.OBSERVE_N:
                self._unobserve(next(iter(self.observations)))
            o = Observation(n, tag, time.time(), ch.state, ch.state_t, mhz)
            self.observations[tag] = o
            ch.observers.append(o)
            return ""

    def end_observe(self, tag: str) -> bool:
        """It ends now (its dropouts up to now are waited for: observed); False if there is no such."""
        with self.lock:
            o = self.observations.get(tag)
            if o is None:
                return False
            if o.t1 is None:
                o.end(time.time())
                o.ended = time.monotonic()
                o.deadline = o.ended + self.END_S
                self.observations[tag] = self.observations.pop(tag)  # (the newest, now: kept the longest)
            return True

    def observed(self, tag: str, start: int = 0, count: int = 0):
        """An ended observation's times (observe.py): None if there is no such; else {ready: its dropouts up to its
        end read (or no longer waited for), total, times: [[began, s], ...] from start, count of them (0: all)},
        ready, else without times."""
        with self.lock:
            o = self.observations.get(tag)
            if o is None or o.t1 is None:
                return None
            if o.result is None:
                m = self.monitors.get(o.n)
                caught_up = m is None or not m.is_alive() or (getattr(m, "seen", 0.0) >= o.t1 + self.PAST_S and (
                    getattr(m, "open_since", None) is None or m.open_since > o.t1))
                w = self.sls_watches.get(o.n)  # (its SLS unit polled since its end)
                caught_up = caught_up and (w is None or not w.is_alive() or bool(w.error) or w.seen >= o.t1)
                if not caught_up and time.monotonic() < o.deadline:
                    return {"ready": False, "total": 0}
                ch = self.channels.get(o.n)
                if ch is not None and ch.sls_drop is not None and ch.sls_drop[0] <= o.t1 and o in ch.observers:
                    o.drop(ch.sls_drop[0], o.t1 - ch.sls_drop[0])  # (a dropout going on: until its end)
                o.result = o.merged()
                if ch is not None and o in ch.observers:
                    ch.observers.remove(o)
            times = o.result[start:start + count] if count > 0 else o.result[start:]
            return {"ready": True, "total": len(o.result), "times": [list(x) for x in times], "lost": o.lost}

    def _unobserve(self, tag: str):
        o = self.observations.pop(tag, None)
        ch = self.channels.get(o.n) if o is not None else None
        if ch is not None and o in ch.observers:
            ch.observers.remove(o)

    def _forget_observations(self):
        """Those ended long enough ago to have been fetched, and those never ended."""
        now, wall = time.monotonic(), time.time()
        for tag, o in list(self.observations.items()):
            over = now - o.ended > self.OBSERVED_S if o.t1 is not None else wall - o.t0 > self.OBSERVE_S
            if over:
                self._unobserve(tag)

    # -------------------------------------------------------------- discovery
    def start_discovery(self):
        with self.lock:
            if self.scanning:
                return
            self.scanning = True
            self.devices_version += 1
        threading.Thread(target=self._discover, daemon=True, name="discovery").start()

    def _discover(self):
        try:
            try:
                found = discover(timeout=1.0, targets=self.discovery_targets)
            except OSError as e:
                log.warning("discovery failed: %s", e)
                found = {}
            with self.lock:
                for key, d in found.items():
                    if key in self.devices:
                        self.devices[key].ids = d.ids
                    else:
                        self.devices[key] = d
                todo = [dataclasses.replace(d) for d in self.devices.values()]
                self.devices_version += 1
            with ThreadPoolExecutor(max_workers=16) as ex:
                for d in ex.map(probe, todo):
                    with self.lock:
                        if d.error and d.key not in found and d.key not in self.known:
                            self.devices.pop(d.key, None)  # gone, and nobody cares about it
                        else:
                            self.devices[d.key] = d
                        self.devices_version += 1
        finally:
            with self.lock:
                self.scanning = False
                self.devices_version += 1

    def _probe_one(self, key: str):
        with self.lock:
            d = dataclasses.replace(self.devices.get(key) or Device.from_key(key))
        probe(d)
        with self.lock:
            self.devices[key] = d
            self.devices_version += 1

    def device_list(self) -> list:
        with self.lock:
            return sorted(self.devices.values(), key=lambda d: (bool(d.error), d.ip, d.port))

    # ----------------------------------------------------------------- misc
    def update_rate(self):
        with self.lock:
            now = time.monotonic()
            self.rate = self._n_readings / max(now - self._rate_t, 1e-3)
            self._n_readings, self._rate_t = 0, now

    def to_config(self) -> dict:
        with self.lock:
            cfg = dict(self.config)
            cfg["channels"] = {str(n): ch.to_config() for n, ch in sorted(self.channels.items())}
            cfg["known_devices"] = list(self.known)
            cfg["dac"] = self.dacs.to_config()
            if self.reference is not None:
                cfg["reference"] = self.reference.to_config()
            else:
                cfg.pop("reference", None)
            self.dirty = False
            return cfg
