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

Currents are shown, and the nominal current given, as measured (current-act);
the set current (current-set) is what a recovery writes, and is watched for
changes by hand.
"""
from __future__ import annotations

import dataclasses
import json
import logging
import math
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

from .dlcpro import DLCPro, DecopError, Device, discover, probe
from .lock import Lock, LockParams
from .modehop import Guard, HopParams, Recenter, Runner, perform
from .relock import setup_report
from .reference import Reference, RefParams
from .wavemeter import C_NM_THZ, NAN

log = logging.getLogger(__name__)

ACTUATOR = "scan:offset"  # the DLC pro parameter the lock writes, below laserN:
CURRENT = "dl:cc:current-set"
CURRENT_ACT = "dl:cc:current-act"
CAVITY_LOCK = "dl:lock:lock-enabled"
SHG_STATE = "nlo:shg:lock:state-txt"
WATCHED = (ACTUATOR, CURRENT, CURRENT_ACT, CAVITY_LOCK, "dl:lock:state-txt", SHG_STATE)


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
        return self.runner.stage if self.runner is not None else "reading the laser's settings"

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
    def __init__(self, config: dict, make_source, discovery_targets=None, extra_devices=(), log_path=None):
        """log_path: a file every recovery is appended to, as a line of JSON."""
        self.config = config
        self.log_path = log_path
        self.lock = threading.RLock()
        self.channels = {int(k): Channel(int(k), c) for k, c in (config.get("channels") or {}).items()}
        self.active = []         # channels measured by the wavemeter, in order
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
        self.source.stop()
        with self.lock:
            for ch in self.channels.values():
                if ch.state != "off":
                    self._disengage(ch, "")
        end = time.monotonic() + 3.0  # a recovery that was cut short puts current and offset back
        while time.monotonic() < end:
            with self.lock:
                if not any(ch.runner is not None for ch in self.channels.values()):
                    break
            time.sleep(0.02)
        with self.lock:
            controllers = list(self.controllers.values())
        for c in controllers:
            c.stop()

    # ------------------------------------------- source callbacks (its thread)
    def on_state(self, ok: bool, msg: str):
        self.source_ok, self.source_msg = ok, msg

    def on_active(self, chans):
        with self.lock:
            for n in chans:
                if n not in self.channels:
                    self.channels[n] = Channel(n)
            for n in set(self.active) - set(chans):
                if self.channels[n].state != "off":
                    self._disengage(self.channels[n], "unlocked: channel no longer measured")
            self.active = list(chans)
            self.layout_version += 1

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
            for ch in self.channels.values():  # a recovery waiting for time to pass
                if ch.runner is not None:
                    ch.runner.tick(readings[-1].t)

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
        e = self._hop_error(ch)
        if check and (ch.cavity_on is False or (math.isfinite(e) and abs(e) > hop.tolerance)):
            if not math.isfinite(e):
                ch.message = "relock: needs a valid reading"
                return
            why = "cavity lock off" if ch.cavity_on is False else (
                f"off by {e / 1e3:+.3f} GHz" if abs(e) >= 1e3 else f"off by {e:+.0f} MHz")
            self._start_recovery(ch, why, budget=False, relock=True)
            return
        ch.state, ch.message = "on", ""
        ch.token += 1
        ch.guard.reset()

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
            for param in WATCHED:
                c.unwatch(f"laser{ch.laser}:{param}")
        ch.device, ch.offset, ch.message = None, None, ""
        ch.current = ch.current_set = ch.cavity_on = None
        ch.cavity_txt = ch.shg_txt = ""

    def _watch(self, ch: Channel):
        """Poll the offset of the linked laser for display, and read its unit once."""
        key, laser = ch.device, ch.laser
        c = self._controller(key)

        def on_offset(v):
            with self.lock:
                if ch.device == key and ch.laser == laser and ch.state == "off":
                    ch.offset = v

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

        def on_shg(v):
            with self.lock:
                if mine():
                    ch.shg_txt = v

        hop = ch.hop
        for param, cb, kind, on in ((CURRENT, on_current_set, "float", hop.enabled),
                                    (CURRENT_ACT, on_current, "float", hop.enabled),
                                    (CAVITY_LOCK, on_cavity, "bool", hop.enabled and hop.cavity),
                                    ("dl:lock:state-txt", on_txt, "str", hop.enabled and hop.cavity),
                                    (SHG_STATE, on_shg, "str", hop.enabled and hop.shg)):
            if on:
                c.watch(p + param, cb, kind)
            else:
                c.unwatch(p + param)

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

    # ------------------------------------------------------- mode-hop recovery
    def set_hop_params(self, n: int, params: HopParams):
        with self.lock:
            ch = self.channels[n]
            if ch.state == "on" and (params.cavity != ch.hop.cavity or (params.cavity and not params.enabled)):
                self._disengage(ch, "")  # locked to the wavemeter, or watched, as it no longer should be
            ch.hop = params
            ch.guard.reset()
            ch.guard.track = 0.0
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

    def recenter(self, n: int):
        """Recenter now, even without a hop: always maps, so it finds the best mode."""
        with self.lock:
            ch = self.channels[n]
            if ch.state in ("recovering", "engaging"):
                return
            if not ch.device:
                ch.message = "right-click to link a laser first"
            elif not math.isfinite(self._hop_error(ch)):
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
        if ch.guard.update(t, e, hop.tolerance, hop.grace, track):
            off = e - ch.guard.track
            self._start_recovery(ch, f"jumped {off / 1e3:+.3f} GHz" if abs(off) >= 1e3 else f"off by {off:+.0f} MHz")
            return True
        return ch.guard.armed and math.isfinite(e) and abs(e - ch.guard.track) > hop.tolerance

    def _start_recovery(self, ch: Channel, why: str, quick: bool = True, budget: bool = True, relock: bool = False):
        """quick: try the piezo alone first if the laser is near its target. budget: counts towards
        max_per_hour (those the guard starts). relock: asked to relock a laser its DLC pro locks to a
        cavity: take its lock over even if it is off now, and watch it afterwards."""
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
                    self._begin_recovery(ch, i0, a0, clip, v0, locked, quick, c)

        def failed(exc):
            with self.lock:
                if ch.token == tok:
                    self._recovery_failed(ch, f"cannot read the laser's settings: {exc}")

        c.submit(job, failed)

    def _begin_recovery(self, ch: Channel, i0, a0, clip, v0, locked, quick, controller):
        """i0, a0: the set and the measured current; the window is about the nominal current, which
        is measured, and becomes one of set currents with their difference now."""
        hop, lp = ch.hop, ch.lock.p
        if not hop.i_nom:
            hop.i_nom = round(a0, 3)  # the current before the first recovery is the nominal one
            self.dirty = True
        off = a0 - i0
        i_lo, i_hi = max(hop.i_nom - hop.i_window - off, 0.0), min(hop.i_nom + hop.i_window - off, clip)
        v_lo, v_hi = max(lp.v_min, v0 - hop.v_window), min(lp.v_max, v0 + hop.v_window)
        ch.current, ch.current_set, ch.offset = a0, i0, v0
        if not i_lo <= i0 <= i_hi:
            self._recovery_failed(ch, f"the current, {a0:.2f} mA measured, is outside {i_lo + off:.2f} … "
                                      f"{i_hi + off:.2f} mA: set the nominal current in Mode-hop recovery…")
            return
        if not v_lo <= v0 <= v_hi:
            self._recovery_failed(ch, f"the offset, {v0:.2f} V, is outside {lp.v_min:g} … {lp.v_max:g} V")
            return
        rec = Recenter(hop, i0, v0, i_lo, i_hi, v_lo, v_hi, cavity=locked, quick=quick, shg=hop.shg, act_offset=off)
        io = _HopIO(self, ch, controller)
        runner = Runner(rec, io, time.monotonic(), finished=lambda r: self._recovery_done(ch, r))
        if runner.busy:
            ch.runner = runner

    def _recovery_failed(self, ch: Channel, why: str):
        """Before anything was touched."""
        ch.state, ch.resume = "off", False
        ch.message = f"{ch.hop_why + ': ' if ch.hop_why else ''}no recovery, {why}"
        ch.token += 1

    def _recovery_done(self, ch: Channel, r: Runner):
        ch.runner = None
        ch.guard.reset()
        ch.state = "off"
        ch.token += 1
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
            ch.note, ch.note_t = f"{why}{res.summary}, {took}", time.monotonic()
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
        for name in ("lp_code", "lp_kind", "peak_y", "falc_sign"):  # worth keeping, even so
            if name in rec.learned:
                setattr(ch.hop, name, rec.learned[name])
                self.dirty = True
        head = "recovery stopped" if r.error == "stopped" else f"{why}recovery failed, {r.error}"
        ch.message = f"{head} ({where})"
        log.warning("channel %d: %s", ch.n, ch.message)
        self._keep_record(ch, r, ch.message)

    def _keep_record(self, ch: Channel, r: Runner, outcome: str):
        """What the recovery saw and did: kept for a look (Channel.last), and logged."""
        rec = r.rec
        ch.last = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"), "channel": ch.n, "name": ch.name,
            "why": ch.hop_why or "recenter now", "ok": r.state == "done", "outcome": outcome,
            "took_s": round(r.now - ch.hop_t, 1), "i_window": [round(rec.mA(rec.i_lo), 4), round(rec.mA(rec.i_hi), 4)],
            "v_window": [rec.v_lo, rec.v_hi],
            "kv": rec.kv, "unit": ch.unit, **rec.record,
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

            def job(client):
                report = setup_report(client, laser, hop, lp)
                with self.lock:
                    if ch.device == key:
                        ch.setup = report

            def failed(exc):
                with self.lock:
                    ch.setup = [("error", f"cannot read the controller: {exc}")]

            self._controller(key).submit(job, failed)

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
