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

Lock dropouts (dropouts.py): for each laser its DLC pro locks to a cavity, a monitor of its own watches the
transmission for the lock losing the laser even for a couple of milliseconds (HopParams.drop_ms between
samples), while its lock is on and no recovery runs (and DROP_SETTLE_S after); each dropout is logged (the
console, and a line of JSON in drop_path), and how many there were, once an hour.

Currents are shown, and the nominal current given, as measured (current-act);
the set current (current-set) is what a recovery writes, and is watched for
changes by hand.

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
import statistics
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

from .dlcpro import DLCPro, DecopError, Device, discover, probe
from .dropouts import Monitor as DropoutMonitor
from .dropouts import log as drop_log
from .lock import Lock, LockParams
from .modehop import SEED_FLOOR, Guard, HopParams, Recenter, Runner, perform
from .relock import TRANS, says_locked, setup_report, transmission_input, transmission_signal, usable_zero, volts
from .reference import Reference, RefParams
from .wavemeter import C_NM_THZ, NAN

log = logging.getLogger(__name__)

ACTUATOR = "scan:offset"  # the DLC pro parameter the lock writes, below laserN:
CURRENT = "dl:cc:current-set"
CURRENT_ACT = "dl:cc:current-act"
CAVITY_LOCK = "dl:lock:lock-enabled"
SHG_STATE = "nlo:shg:lock:state-txt"
SHG_INTRA = "nlo:pd:shg-int:photodiode"
WATCHED = (ACTUATOR, CURRENT, CURRENT_ACT, CAVITY_LOCK, "dl:lock:state-txt", SHG_STATE, SHG_INTRA)
FINE_N = 600   # polls: a level locked as usual is the median of the last this many running as it should...
FINE_MIN = 10  # ...once there are this many (until then, the one learned before)


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def round_sig(x: float, digits: int = 9) -> float:
    if not x or not math.isfinite(x):
        return x
    return round(x, digits - 1 - math.floor(math.log10(abs(x))))


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
        self.device = link.get("device") or None  # controller key: 'ip' or 'ip:port'
        self.laser = int(link.get("laser", 1))
        self.lock = Lock(LockParams.from_dict(cfg.get("lock")))
        self.hop = HopParams.from_dict(cfg.get("hop"))
        # live state, written by engine threads and read by the GUI
        self.freq = NAN       # THz, corrected by the reference if there is one; NaN if invalid
        self.raw = NAN        # THz, as the wavemeter read it
        self.status = "no data"
        self.t = 0.0          # monotonic time of the last new reading
        self.state = "off"    # off | engaging | on (locked, or for a cavity-locked laser: watched) | recovering
        self.message = ""     # why the lock is off, or what to do
        self.offset = None    # last known scan offset
        self.unit = "V"
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

    @property
    def param(self) -> str:
        return f"laser{self.laser}:{ACTUATOR}"

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
        if self.device:
            d["link"] = {"device": self.device, "laser": self.laser}
        if self.hop != HopParams():
            d["hop"] = self.hop.to_dict()
        return d

    @property
    def relock(self) -> bool:
        """Its DLC pro locks it to a cavity: engaging it arms the relock, the wavemeter lock never runs."""
        return self.hop.cavity

    @property
    def hop_stage(self) -> str:
        if self.state != "recovering":
            return ""
        if self.runner is not None:
            return self.runner.stage
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

    def __init__(self, key: str, host: str, port: int):
        super().__init__(daemon=True, name=f"dlcpro {key}")
        self.key = key
        self.client = DLCPro(host, port, timeout=1.0)
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
    everything else runs as a job (modehop.perform)."""

    def __init__(self, engine: "Engine", ch: Channel, controller: Controller):
        self.engine, self.ch, self.ctrl = engine, ch, controller
        self.laser = ch.laser

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

        if op[0] == "set" and op[1] in (CURRENT, ACTUATOR):
            self.ctrl.set(f"laser{self.laser}:{op[1]}", op[2], on_error=err, on_done=ok)
            return
        laser = self.laser
        self.ctrl.submit(lambda client: ok(perform(client, laser, op)), err)


class Engine:
    LEVEL_S = 10.0  # s: a cavity lock's transmission (an SHG signal) averaged over this long, to tell one that keeps
    #                 losing it
    LOOK_S = 15.0    # s: a look at a laser, its wavemeter channel switched on for it, lasts this long at most...
    LOOK_GAP = 60.0  # s: ...and comes this long after the last at the soonest
    LOOK_N = 5       # readings in a row on target (its locks saying locked) end a look; off target, it is relocked
    WAIT_S = 10.0    # s a recovery waits for readings once its wavemeter channel is switched on for it
    FINE_S = 900.0   # s: where it was last on target as it should be, if that recent, is where its right mode is

    def __init__(self, config: dict, make_source, discovery_targets=None, extra_devices=(), log_path=None,
                 drop_path=None):
        """log_path: a file every recovery is appended to, as a line of JSON; drop_path: the same for every lock
        dropout."""
        self.config = config
        self.log_path, self.drop_path = log_path, drop_path
        self.monitors = {}  # channel -> its DropoutMonitor
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

    # ------------------------------------------------------------- lifecycle
    def start(self):
        with self.lock:
            for ch in self.channels.values():
                if ch.device:
                    self._watch(ch)
        self.source.start()
        self.start_discovery()

    def stop(self):
        with self.lock:
            monitors, self.monitors = list(self.monitors.values()), {}
        for m in monitors:
            m.stop()  # (each puts its recorder's settings back)
        with self.lock:
            for ch in self.channels.values():
                if ch.state != "off":
                    self._disengage(ch, "")
        end = time.monotonic() + 20.0  # a recovery that was cut short puts current and offset back (slowly down)
        while time.monotonic() < end:
            with self.lock:
                runners = [ch.runner for ch in self.channels.values() if ch.runner is not None]
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
        for m in monitors:
            m.join(max(0.0, end + 5.0 - time.monotonic()))
        with self.lock:
            controllers = list(self.controllers.values())
        for c in controllers:
            c.stop()

    # ------------------------------------------- source callbacks (its thread)
    def on_state(self, ok: bool, msg: str):
        self.source_ok, self.source_msg = ok, msg

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
            if ch.runner is not None:
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
                chs[n].relock and chs[n].hop.enabled and chs[n].device)}
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
        if ch.state == "recovering" or ch.look_why:
            if not ch.borrowed:
                log.info("channel %d: switched off on the wavemeter during a %s: on again", ch.n,
                         "look" if ch.look_why else "recovery")
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
                ch.raw, ch.status, ch.t = r.freq, r.status, r.t
                ch.freq = r.freq if ref is None or r.ch == ref.channel else ref.correct(r.freq)
                withheld = self._hop_reading(ch, r.t) if ch.hop.enabled or ch.runner else False
                if ch.state == "on" and ch.freq > 0 and not withheld and not ch.relock:
                    self._step(ch)
                if ch.pending is not None and ch.freq > 0:  # the readings it waited for
                    args, ch.pending = ch.pending, None
                    self._begin_recovery(ch, *args)
                elif ch.look_why and ch.state == "on":
                    self._looking(ch)
            for ch in self.channels.values():  # a recovery waiting for time to pass
                if ch.runner is not None:
                    ch.runner.tick(readings[-1].t)
            self._timers(readings[-1].t)

    def _step(self, ch: Channel):
        u = ch.lock.update(ch.error_mhz)
        if u is not None:
            ch.offset = u
            tok = ch.token
            self._controller(ch.device).set(ch.param, u, lambda e: self._comm_failed(ch, tok, e))
        if not ch.lock.active:
            self._disengage(ch, ch.lock.message)

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
            tok, laser, params = ch.token, ch.laser, ch.lock.p

            def job(client):
                v = client.get_float(f"laser{laser}:{ACTUATOR}")
                try:
                    unit = client.get_str(f"laser{laser}:scan:unit") or "V"
                except DecopError:
                    unit = "V"
                with self.lock:
                    if ch.token != tok:
                        return
                    ch.offset, ch.unit = v, unit
                    if not params.v_min <= v <= params.v_max:
                        ch.state, ch.message = "off", f"offset {v:.3f} outside {params.v_min:g} … {params.v_max:g}"
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

    def _arm(self, ch: Channel, check: bool = True):
        """A laser its DLC pro locks to a cavity: watch it, and relock it after a hop. check: if it is
        not locked on target now, relock it now."""
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
        if check and not hop.optimum:  # clicked on a laser set up by hand: its optimum, the first time
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
        if ch.runner is not None:  # a recovery: it puts current and offset back, then reports
            ch.resume = False
            ch.runner.cancel(message or "stopped")
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

    def set_hidden(self, n: int, hidden: bool):
        with self.lock:
            ch = self.channels[n]
            if hidden and ch.state != "off":
                self._disengage(ch, "")
            ch.hidden = hidden
            self.layout_version += 1
            self.dirty = True

    def link(self, n: int, key: str, laser: int):
        key = Device.from_key(key).key
        with self.lock:
            for other in self.channels.values():
                if other.n != n and other.device == key and other.laser == laser:
                    self._unlink(other)
                    other.message = f"laser{laser} moved to channel {n}"
            ch = self.channels[n]
            self._unlink(ch)
            ch.device, ch.laser = key, int(laser)
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
            for param in WATCHED + ((ch.seed_name,) if ch.seed_name else ()):
                c.unwatch(f"laser{ch.laser}:{param}")
        self._unwatch_trans(ch)
        m = self.monitors.pop(ch.n, None)
        if m is not None:
            m.stop()
        ch.device, ch.offset, ch.message = None, None, ""
        ch.current = ch.current_set = ch.cavity_on = ch.seed = ch.lasing = ch.shg_level = None
        ch.cavity_txt = ch.shg_txt = ch.seed_name = ""
        ch.shg_hist.clear()
        ch.shg_fine.clear()

    def _watch(self, ch: Channel):
        """Poll the offset of the linked laser for display, and read its unit once."""
        key, laser = ch.device, ch.laser
        c = self._controller(key)

        def on_offset(v):
            with self.lock:
                if ch.device == key and ch.laser == laser:
                    if ch.state == "off":
                        ch.offset = v
                    if ch.current_set is not None and self._running_fine(ch):
                        ch.fine_at = (time.monotonic(), v, ch.current_set)  # (a cavity lock moves the offset)

        def read_unit(client):
            unit = client.get_str(f"laser{laser}:scan:unit") or "V"
            with self.lock:
                if ch.device == key and ch.laser == laser:
                    ch.unit = unit

        c.watch(f"laser{laser}:{ACTUATOR}", on_offset)
        c.submit(read_unit)
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
        for param, cb, kind, on in ((CURRENT, on_current_set, "float", hop.enabled),
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
        armed while its DLC pro has its cavity lock on (the reference laser: always, it has no button)."""
        hop = ch.hop
        if not hop.enabled or not ch.device:
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
        known yet), its transmission's signal id)."""
        with self.lock:
            if (ch.device, ch.laser) != key:
                return False, None, None
            if not ch.cavity_on or ch.state == "recovering":
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

    def _controller(self, key: str) -> Controller:
        c = self.controllers.get(key)
        if c is None:
            d = Device.from_key(key)
            c = self.controllers[key] = Controller(key, d.ip, d.port)
            c.start()
        return c

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
                        if ch.state == "on" and ch.raw > 0 and not ch.relock), default=0.0)

    def tick(self):
        """Call now and then from the GUI: keeps the reference's state current when readings stop,
        and times out recoveries that wait in vain."""
        with self.lock:
            now = time.monotonic()
            if self.reference is not None:
                self.reference.tick(now)
            for ch in list(self.channels.values()):
                if ch.runner is not None:
                    ch.runner.tick(now)
            self._timers(now)

    # ------------------------------------------------------- mode-hop recovery
    def set_hop_params(self, n: int, params: HopParams):
        with self.lock:
            ch = self.channels[n]
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

    def take_optimum(self, n: int):
        """The laser as it is now is at its optimum (set by hand): its current, piezo voltage and SHG signal locked
        are what the catch goes back to."""
        with self.lock:
            ch = self.channels[n]
            if not ch.device:
                ch.message = "right-click to link a laser first"
            elif ch.state == "recovering":
                ch.message = "take the optimum: not while it recovers"
            else:
                self._take_optimum(ch)

    def _take_optimum(self, ch: Channel):
        key, laser, shg = ch.device, ch.laser, ch.hop.shg
        names = (CURRENT_ACT, ACTUATOR) + ((SHG_STATE, SHG_INTRA) if shg else ())

        def job(client):
            got = perform(client, laser, ("get", names))
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
                self.dirty = True
                ch.note = (f"optimum taken: {act:.2f} mA, {v:.2f} {ch.unit}" + (
                    f", SHG {level:.3f} V locked" if level else ", SHG not locked: its level not known" if shg else ""))
                ch.note_t = time.monotonic()
                log.info("channel %d: %s", ch.n, ch.note)

        self._controller(key).submit(job)

    def recenter(self, n: int):
        """Recenter now, even without a hop: always maps, so it finds the best mode."""
        with self.lock:
            ch = self.channels[n]
            if ch.state in ("recovering", "engaging"):
                return
            if not ch.device:
                ch.message = "right-click to link a laser first"
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
        ch.token += 1
        ch.hop_why, ch.hop_t = why, now
        for d in (ch.trans_hist, ch.shg_hist, ch.trans_fine, ch.shg_fine):  # (what it relocked to is learned afresh)
            d.clear()
        ch.look_why = ch.look_due = ch.wait_fail = ""
        ch.pending, ch.wait_since = None, now
        if ch.off:  # no readings: its wavemeter channel on first
            ch.borrowed = True
            self._switch(ch, True)
        tok, laser, cavity = ch.token, ch.laser, hop.cavity
        log.info("channel %d: %s, recovering", ch.n, why or "recenter")

        def job(client):
            p = f"laser{laser}:"
            i0 = client.get_float(p + CURRENT)
            a0 = client.get_float(p + CURRENT_ACT)
            clip = client.get_float(p + "dl:cc:current-clip")
            v0 = client.get_float(p + ACTUATOR)
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
            self._recovery_failed(ch, f"the offset, {v0:.2f} V, is outside {lp.v_min:g} … {lp.v_max:g} V")
            return
        rec = Recenter(hop, i0, v0, i_lo, i_hi, v_lo, v_hi, cavity=locked, quick=quick, shg=hop.shg, act_offset=off,
                       i_top=i_top, i_clip=clip, ref=ref)
        io = _HopIO(self, ch, controller)
        runner = Runner(rec, io, time.monotonic(), timeout=rec.timeout, finished=lambda r: self._recovery_done(ch, r))
        if runner.busy:
            ch.runner = runner
            if ch.off:  # (begun at once, not lasing: its channel is being switched on)
                runner.blind(True)

    def _recovery_failed(self, ch: Channel, why: str):
        """Before anything was touched."""
        ch.state, ch.resume = "off", False
        ch.message = f"{ch.hop_why + ': ' if ch.hop_why else ''}no recovery, {why}"
        ch.token += 1
        ch.pending = None
        self._release(ch)

    def _recovery_done(self, ch: Channel, r: Runner):
        ch.runner = None
        ch.guard.reset()
        ch.state = "off"
        ch.token += 1
        self._release(ch)  # its wavemeter channel, if it was switched on for the recovery
        took = f"{r.now - ch.hop_t:.0f} s"
        why = f"{ch.hop_why}: " if ch.hop_why else ""
        rec = r.rec
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
            key, laser, hop, lp = ch.device, ch.laser, ch.hop, ch.lock.p
            wlm = self._channel_report(ch)

            def job(client):
                report = setup_report(client, laser, hop, lp) + wlm
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
            if self.reference is not None:
                cfg["reference"] = self.reference.to_config()
            else:
                cfg.pop("reference", None)
            self.dirty = False
            return cfg
