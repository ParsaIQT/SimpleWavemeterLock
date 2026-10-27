"""Simulated SLS lock electronics (sls.py), for the demo and the tests.

  FakeSls  an SLS unit's HTTP server on 127.0.0.1: ?parameter=X, ?parameter=X&value=Y, ?getValues,
           ?appCommand=, ?sysCommand=, answered as the units do (Name=value lines); its parameters a dict (a callable
           is asked at every read, and is read-only). down: 'close' (it drops every connection) or 'hang' (it never
           answers).
  SimPdh   an SLS Laser Servo locking a master (a SimLaser with cavity=True: its frequency fixed while it is locked)
           to the cavity. drop(seconds): its lock lost that long (the master swept by its autolock's search, its
           transmission low), then caught again (LockCount up one, LockTime from zero).
  SimPll   an SLS PLL (one channel, the SuperHeterodyne's parameters; or servo N of a 3-channel PLL, ServoN...)
           beat-locking a slave (a SimLaser with a DLC pro) `beat` MHz above its master, its fast output on the
           slave's current (fast_ma mA per V) and its slow output on its piezo (slow_v V per V), through its DLC pro's
           inputs (FakeDLCPro.external). As primitive as the lab's: its manual lock, engaged with the slave within
           CAPTURE_MHZ of its beat point, locks; otherwise, or once its master's lock is lost, it has lost the beat
           for good: its outputs run to their rails (run_sign: the fast one taking the slave's current down, it stops
           lasing, and its TA, without seed, switches itself off), and stay there until it is released (manual lock
           off: its outputs back to zero). Its autolock catches the slave within AUTO_MHZ of its beat point.
"""
from __future__ import annotations

import math
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qsl, urlsplit

from .sls import as_bool


def _fmt(value) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{value:.15E}"
    return str(value)


def duration(s: float) -> str:
    """As the units say how long they have held: '0d 01:00:21'."""
    s = int(max(s, 0.0))
    return f"{s // 86400}d {s // 3600 % 24:02d}:{s // 60 % 60:02d}:{s % 60:02d}"


class FakeSls:
    """An SLS unit's HTTP server (see the module). on_set[name](value) after a write; log: the writes, in order."""

    def __init__(self, name: str, params: dict | None = None, host: str = "127.0.0.1", port: int = 0):
        self.name = name
        self.params = dict(params or {})
        self.on_set = {}
        self.log = []       # (name, value) written, in order
        self.asked = 0      # requests answered
        self.down = ""      # '': it answers; 'close': it drops connections; 'hang': it never answers
        self.delay = 0.0    # s before each answer
        self.mutex = threading.RLock()
        fake = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"  # (connections kept open; the client copes either way)

            def do_GET(self):
                if fake.down == "hang":
                    time.sleep(5.0)
                if fake.down:
                    self.close_connection = True
                    return
                if fake.delay:
                    time.sleep(fake.delay)
                body = fake.answer(urlsplit(self.path).query).encode()
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "text/plain")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):  # (an answer no longer waited for: a slow unit's)
                    self.close_connection = True
                    return
                fake.asked += 1

            def log_message(self, *args):
                pass

        self._srv = ThreadingHTTPServer((host, port), Handler)
        self._srv.daemon_threads = True
        self.host, self.port = host, self._srv.server_address[1]
        threading.Thread(target=self._srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True,
                         name=f"sim sls {name}").start()

    @property
    def key(self) -> str:
        return f"{self.host}:{self.port}"

    def value(self, name: str):
        v = self.params[name]
        return v() if callable(v) else v

    def written(self, name: str) -> list:
        return [v for k, v in self.log if k == name]

    def answer(self, query: str) -> str:
        """What it says to GET /?query."""
        if query.split("&")[0] == "getValues":
            with self.mutex:
                names = list(self.params)
            return "".join(f"{k}={_fmt(self.value(k))}\n" for k in names)
        q = dict(parse_qsl(query, keep_blank_values=True))
        if "parameter" in q:
            name = q["parameter"]
            if name not in self.params:
                return f"Unknown parameter: {name}\n"
            if "value" not in q:
                return f"{name}={_fmt(self.value(name))}\n"
            with self.mutex:
                old = self.params[name]
                if callable(old):
                    return f"Parameter {name} is read only\n"
                text = q["value"]
                try:
                    new = (as_bool(text) if isinstance(old, bool) else float(text) if isinstance(old, float)
                           else int(float(text)) if isinstance(old, int) else text)
                except ValueError:
                    new = None
                if new is None:
                    return f"Invalid value for {name}: {text}\n"
                self.params[name] = new
                self.log.append((name, new))
            cb = self.on_set.get(name)
            if cb is not None:
                cb(new)
            return f"{name}={text}\n"
        for key in ("appCommand", "sysCommand"):
            if key in q:
                self.log.append((key, q[key]))
                return "OK\n"
        return "Unknown command\n"

    def close(self):
        self._srv.shutdown()
        self._srv.server_close()


class SimPdh:
    """An SLS Laser Servo locking `laser` (cavity=True) to the cavity, its parameters on `sls` (see the module)."""

    def __init__(self, laser, sls: FakeSls, clock=time.monotonic, seed=None):
        self.laser, self.sls, self.clock = laser, sls, clock
        self.rng = random.Random(seed)
        self.count = 1
        self.since = clock() - 3621.0   # locked since then
        self.lost_until = -math.inf     # its lock lost until then
        self.drops = 0
        sls.params.update({
            "AutoLockEnable": True, "LockPoint": 2, "SweepType": 0, "LockDetectEnable": True,
            "AutoLockState": self.state, "Transmission": self.transmission, "TransmissionThreshold": 0.2,
            "LockCount": lambda: self.count, "LockTime": self.lock_time,
            "CurrentServoOutput": lambda: self.rng.gauss(0.0, 0.01) if self.locked() else 0.0,
            "PZTServoOutput": lambda: -0.18 + self.rng.gauss(0.0, 0.002) if self.locked() else 0.0,
            "PDHFrequency": 1.01e7, "OffsetFrequency": 1.0e8, "CavityFSR": 1.5e9,
            "DebugMessage5": lambda: "Servos Enabled (No TX)" if self.locked() else "PZT acq carrier",
            "AppVersion": "AppVer:Jul 11 2024 11:08:14", "GUIVersion": "GUIVer:v2.7",
        })
        sls.on_set["AutoLockEnable"] = self._enabled

    def locked(self) -> bool:
        return bool(self.sls.params["AutoLockEnable"]) and self.clock() >= self.lost_until

    def drop(self, seconds: float):
        """Its lock lost for that long, then caught again."""
        now = self.clock()
        self.lost_until = self.since = now + seconds
        self.count += 1
        self.drops += 1
        self.laser.unlock(seconds)

    def state(self) -> str:
        if not self.sls.params["AutoLockEnable"]:
            return "Idle"
        return f"Locked: {self.count} ({duration(self.lock_time())})" if self.locked() else "Acquiring"

    def transmission(self) -> float:
        return 3.56 + self.rng.gauss(0.0, 0.01) if self.locked() else 0.02 + abs(self.rng.gauss(0.0, 0.01))

    def lock_time(self) -> float:
        return max(self.clock() - self.since, 0.0) if self.locked() else 0.0

    def _enabled(self, on):
        if on:  # (caught a second after its autolock is switched on)
            self.lost_until = self.since = self.clock() + 1.0
            self.count += 1
            self.laser.unlock(1.0)
        else:
            self.laser.unlock(1e9)


class SimPll:
    """An SLS PLL beat-locking `slave` to `master` (see the module); master_lock: the master's SimPdh (its lock lost,
    the beat is lost); beat: MHz, its lock frequency; offset: MHz, the slave less its master while it holds it (an AOM
    in the beat's path makes up the difference: 0, the slave on its master's frequency; None: beat); slow_servo:
    False, its slow servo unused (as on Sirius): its fast output alone holds the slave, and runs out of range as the
    slave drifts, unless something else (this program's trim) moves its piezo."""

    CAPTURE_MHZ = 1.0   # its manual lock engaged with the slave this near its beat point: it locks
    AUTO_MHZ = 30.0     # its autolock catches it this near
    IF_MHZ = 0.64       # half its IF band: the beat's amplitude falls off beyond
    AMP = 0.88          # V: the beat's amplitude in band
    FAST_RAIL, SLOW_RAIL = 1.0, 5.0  # V
    FAST_S = 0.05       # s its fast output takes to its rail once the beat is lost
    SLOW_RATE = 4.0     # V/s its slow output runs at, then

    def __init__(self, sls: FakeSls, slave, master, master_lock: SimPdh | None = None, beat: float = 80.0,
                 servo: int = 0, fast_ma: float = 4.0, slow_v: float = 2.0, run_sign: int = -1,
                 clock=time.monotonic, seed=None, offset: float | None = None, slow_servo: bool = True):
        self.sls, self.slave, self.master, self.master_lock = sls, slave, master, master_lock
        self.beat, self.servo, self.slow_servo = beat, servo, slow_servo
        self.offset = beat if offset is None else offset
        self.fast_ma, self.run_sign = fast_ma, run_sign
        self.slow_v = slow_v if slow_servo else 0.0  # V of piezo a V of its slow output adds (unused: none)
        self.clock = clock
        self.rng = random.Random(seed)
        self.mutex = threading.RLock()
        self.manual = self.auto = False
        self.locked = False      # it holds the beat
        self.fast = self.slow = 0.0  # V: its outputs
        self.lost_t = None       # engaged, it lost the beat then: its outputs running away since
        self.lost_slow = 0.0
        self.count = 0
        self.since = clock()
        self.losses = 0          # times it lost the beat engaged
        self.engages = 0         # manual lock engaged
        if servo:
            self.n = {k: f"Servo{servo}{v}" for k, v in (
                ("manual", "ManualLockEnable"), ("auto", "AutoLockEnable"), ("state", "LockState"),
                ("status", "LockStatus"), ("phase", "PhaseError"), ("fast", "FastServoOutput"),
                ("slow", "SlowServoOutput"), ("count", "LockCount"), ("held", "LockTime"), ("freq", "LockFrequency"),
                ("detect", "LockDetectEnable"))}
            for k in (1, 2, 3):  # (the other two channels idle)
                if k != servo:
                    sls.params.update({f"Servo{k}AutoLockEnable": False, f"Servo{k}ManualLockEnable": False,
                                       f"Servo{k}LockState": "IDLE", f"Servo{k}LockStatus": f"Servo{k} Idle",
                                       f"Servo{k}FastServoOutput": 0.0, f"Servo{k}SlowServoOutput": 0.0,
                                       f"Servo{k}PhaseError": 0.0, f"Servo{k}LockFrequency": 8.0e7})
            sls.params.update({"MonitorSelect": 1, "AppVersion": "AppVer:Dec  5 2024 13:28:38"})
        else:
            self.n = {"manual": "PLLManualLockEnable", "auto": "AutoLockEnable", "state": "PLLLockState",
                      "status": "PLLLockStatus", "phase": "PLLPhase", "fast": "CurrentServoOutput",
                      "slow": "PZTServoOutput", "count": "PLLLockCount", "held": "PLLLockTime",
                      "freq": "LockFrequency", "detect": "LockDetectEnable"}
            sls.params.update({"PLLAutoLockEnable": lambda: self.auto, "PLLAmplitude": self.amplitude,
                               "PLLAmpThreshold": 0.3, "AutoLockState": lambda: "LOCKED" if self.locked else "IDLE",
                               "IFBandwidthNShift": 4, "PFDRange": 2, "AppVersion": "AppVer:Oct  9 2024 14:53:35"})
        n = self.n
        sls.params.update({
            n["manual"]: False, n["auto"]: False, n["state"]: self.state, n["status"]: self.status,
            n["phase"]: self.phase, n["fast"]: lambda: self._out()[0], n["slow"]: lambda: self._out()[1],
            n["count"]: lambda: self.count, n["held"]: lambda: self.clock() - self.since if self.locked else 0.0,
            n["freq"]: abs(beat) * 1e6, n["detect"]: True, "SweepTime": 0.24, "ExtRefEnable": False,
            "GUIVersion": "GUIVer:v3.0",
        })
        sls.on_set[n["manual"]] = self._manual
        sls.on_set[n["auto"]] = self._auto
        slave.beat = self
        slave.dlc.external[slave.n] = self.outputs

    # ----------------------------------------------------------------------------------------- what it does
    def engage(self):
        """Its manual lock engaged at its box (as by hand)."""
        self.sls.params[self.n["manual"]] = True
        self._manual(True)

    def hold(self):
        """Its manual lock engaged and holding the slave (as it would after a good engagement), wherever it is."""
        with self.slave._mutex, self.mutex:
            self.sls.params[self.n["manual"]] = True
            self.manual = True
            free = self._free(self.clock())
            if free is not None:
                self._lock(self.clock(), free)

    def outputs(self):
        """(V, mA) its outputs add to the slave's piezo and current."""
        fast, slow = self._out()
        return slow * self.slow_v, fast * self.fast_ma

    def _out(self):
        self.update()
        return self.fast, self.slow

    def _master_ok(self) -> bool:
        return self.master_lock is None or self.master_lock.locked()

    def _free(self, now: float):
        """MHz: the slave, its outputs at zero, from its beat point (None: either does not lase)."""
        s = self.slave
        fm = self.master.true_freq(now)
        if fm is None or not s.ecdl.lasing:
            return None
        v, i = s.dlc.offset(s.n), s.dlc.current_own(s.n)
        f = s.ecdl.f0 + (s.ecdl.mode_ghz(v, i) + s.wobble * 1e-3) * 1e-3
        return (f - fm) * 1e6 - self.offset

    def _need(self, free: float) -> float:
        """V of slow output that puts the slave on its beat point."""
        return -free * 1e-3 / (self.slave.ecdl.kv * self.slow_v)

    def _need_fast(self, free: float) -> float:
        """V of fast output that does (its slow servo unused)."""
        return -free * 1e-3 / (self.slave.ecdl.ki * self.fast_ma)

    def _holding(self, free: float) -> bool:
        """It holds the slave there: its outputs where they must be, within their rails."""
        if self.slow_servo:
            need = self._need(free)
            if abs(need) < self.SLOW_RAIL:
                self.slow, self.fast = need, self.rng.gauss(0.0, 0.002)
                return True
            return False
        need = self._need_fast(free)
        if abs(need) < self.FAST_RAIL:
            self.slow, self.fast = 0.0, need + self.rng.gauss(0.0, 0.002)
            return True
        return False

    def update(self):
        """Where it is now: holding the slave (its slow output where it must be), or running away."""
        with self.slave._mutex, self.mutex:
            now = self.clock()
            if not (self.manual or self.auto):
                self.locked, self.fast, self.slow, self.lost_t = False, 0.0, 0.0, None
                self._settle()
                return
            if self.locked:
                free = self._free(now)
                if free is not None and self._master_ok() and self._holding(free):
                    return
                self._lose(now)
            if self.auto and self._master_ok():  # its autolock catches it when near
                free = self._free(now)
                if free is not None and abs(free) <= self.AUTO_MHZ:
                    self._lock(now, free)
                    return
            if self.lost_t is not None:  # (its integrators running to their rails)
                dt = now - self.lost_t
                self.fast = self.run_sign * self.FAST_RAIL * min(dt / self.FAST_S, 1.0)
                self.slow = min(max(self.lost_slow + self.run_sign * self.SLOW_RATE * dt, -self.SLOW_RAIL),
                                self.SLOW_RAIL)
                self._settle()

    def _settle(self):
        """The slave, at what its controller and these outputs make of its piezo and current: lasing or not."""
        s = self.slave
        s.ecdl.lase(s.dlc.offset(s.n) + self.slow * self.slow_v, s.dlc.current_own(s.n) + self.fast * self.fast_ma)

    def _lose(self, now: float):
        self.locked, self.lost_t, self.lost_slow = False, now, self.slow
        self.losses += 1

    def _lock(self, now: float, free: float):
        self.locked, self.lost_t = True, None
        if self.slow_servo:
            self.slow, self.fast = self._need(free), 0.0
        else:
            self.slow, self.fast = 0.0, self._need_fast(free)
        self.count += 1
        self.since = now

    def _manual(self, on):
        with self.slave._mutex, self.mutex:
            self.manual = bool(on)
            if not on:
                self.locked, self.lost_t, self.fast, self.slow = False, None, 0.0, 0.0
                self._settle()
                return
            self.engages += 1
            now = self.clock()
            free = self._free(now) if self._master_ok() else None
            if free is not None and abs(free) <= self.CAPTURE_MHZ:
                self._lock(now, free)
            else:
                self.locked, self.lost_t, self.lost_slow = False, now, self.slow

    def _auto(self, on):
        with self.slave._mutex, self.mutex:
            self.auto = bool(on)
            if not on and not self.manual:
                self.locked, self.lost_t, self.fast, self.slow = False, None, 0.0, 0.0
                self._settle()

    # ----------------------------------------------------------------------------------------- what it shows
    def held(self, now: float, f: float) -> float:
        """The slave's frequency (THz), f as it would be by itself: its master's plus the beat while it holds it."""
        self.update()
        if not self.locked:
            return f
        fm = self.master.true_freq(now)
        return f if fm is None else fm + (self.offset + self.rng.gauss(0.0, 0.02)) * 1e-6

    def _error(self, now: float):
        """MHz: the slave from its beat point, its outputs and all (None: either does not lase)."""
        free = self._free(now)
        if free is None:
            return None
        e = self.slave.ecdl
        return free + 1e3 * (e.kv * self.slow * self.slow_v + e.ki * self.fast * self.fast_ma)

    def amplitude(self) -> float:
        self.update()
        if self.locked:
            return self.AMP + self.rng.gauss(0.0, 0.01)
        d = self._error(self.clock())
        noise = abs(self.rng.gauss(0.0, 0.005))
        return noise if d is None else self.AMP / (1.0 + (d / self.IF_MHZ) ** 2) + noise

    def phase(self) -> float:
        self.update()
        return self.rng.gauss(0.0, 0.05) if self.locked else self.rng.uniform(-math.pi, math.pi)

    def state(self) -> str:
        self.update()
        if self.manual:
            return "MANUAL"
        if self.auto:
            return "AUTO_LOCKED" if self.locked else "AUTO_SEARCHING"
        return "IDLE"

    def status(self) -> str:
        self.update()
        who = f"Servo{self.servo}" if self.servo else "PLL"
        if self.manual:
            return f"{who} Manual Lock"
        if self.auto:
            return (f"{who} Auto Lock: {self.count} ({duration(self.clock() - self.since)})" if self.locked
                    else f"{who} Auto Lock searching")
        return f"{who} Idle"
