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
"""
from __future__ import annotations

import dataclasses
import logging
import math
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

from .dlcpro import DLCPro, DecopError, Device, discover, probe
from .lock import Lock, LockParams
from .reference import Reference, RefParams
from .wavemeter import C_NM_THZ, NAN

log = logging.getLogger(__name__)

ACTUATOR = "scan:offset"  # the DLC pro parameter the lock writes, below laserN:


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
        # live state, written by engine threads and read by the GUI
        self.freq = NAN       # THz, corrected by the reference if there is one; NaN if invalid
        self.raw = NAN        # THz, as the wavemeter read it
        self.status = "no data"
        self.t = 0.0          # monotonic time of the last new reading
        self.state = "off"    # off | engaging | on
        self.message = ""     # why the lock is off, or what to do
        self.offset = None    # last known scan offset
        self.unit = "V"
        self.token = 0        # bumped on every engage/disengage; late callbacks compare it

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
        return d


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
        self._sets = {}     # param -> (value, on_error); the latest value wins
        self._jobs = deque()
        self._watch = {}    # param -> callback(value)
        self._next_poll = 0.0
        self._retry_at = 0.0
        self._running = True

    def set(self, param, value, on_error=None):
        with self._cv:
            self._sets[param] = (value, on_error)
            self._cv.notify()

    def cancel(self, param):
        with self._cv:
            self._sets.pop(param, None)

    def submit(self, fn, on_error=None):
        """Run fn(client) in this thread; on_error(exc) if it raises."""
        with self._cv:
            self._jobs.append((fn, on_error))
            self._cv.notify()

    def watch(self, param, callback):
        with self._cv:
            self._watch[param] = callback
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
            for param, (value, on_error) in sets.items():  # lock writes first
                self._call(lambda c, p=param, v=value: c.set(p, v), on_error)
            for fn, on_error in jobs:
                self._call(fn, on_error)
            for param, callback in watch.items():
                self._call(lambda c, p=param, cb=callback: cb(c.get_float(p)), None)
        self.client.close()

    def _call(self, fn, on_error):
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


class Engine:
    def __init__(self, config: dict, make_source, discovery_targets=None, extra_devices=()):
        self.config = config
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
                if ch.state == "on" and ch.freq > 0:
                    self._step(ch)

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
            ch.state, ch.message = "engaging", ""
            ch.token += 1
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

    def disengage(self, n: int, message: str = ""):
        with self.lock:
            ch = self.channels[n]
            if ch.state != "off":
                self._disengage(ch, message)

    def _disengage(self, ch: Channel, message: str):
        ch.lock.disengage(message)
        ch.state, ch.message = "off", message
        ch.token += 1
        c = self.controllers.get(ch.device)
        if c is not None:
            c.cancel(ch.param)

    def set_setpoint(self, n: int, nm):
        with self.lock:
            ch = self.channels[n]
            ch.set_setpoint(nm)
            if ch.setpoint_nm is None and ch.state != "off":
                self._disengage(ch, "unlocked: setpoint cleared")
            self.dirty = True

    def setpoint_to_current(self, n: int):
        with self.lock:
            ch = self.channels[n]
            if ch.freq > 0:
                ch.set_setpoint(round_sig(C_NM_THZ / ch.freq))
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
            c.unwatch(ch.param)
        ch.device, ch.offset, ch.message = None, None, ""

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
                        if ch.state == "on" and ch.raw > 0), default=0.0)

    def tick(self):
        """Call now and then from the GUI: keeps the reference's state current when readings stop."""
        with self.lock:
            if self.reference is not None:
                self.reference.tick(time.monotonic())

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
