"""Simulation: a fake multichannel wavemeter and fake DLC pro controllers.

Used by `python -m wmlock --sim` and by the tests. The fake controllers speak
the DeCoP command line over TCP and answer UDP discovery on 127.0.0.1, so the
whole stack (discovery, protocol, lock, reference, mode-hop recovery) runs for
real. Each laser is an external-cavity diode laser (ECDL) whose mode and
frequency follow the scan offset and diode current of its fake controller,
optionally locked to a reference cavity by a simulated DLC pro lock; the
simulated wavemeter adds a slow common error and noise on top.
"""
from __future__ import annotations

import math
import random
import re
import socket
import threading
import time

from .dlcpro import DISCOVERY_PORT
from .wavemeter import C_NM_THZ, Source


def _decop(value) -> str:
    if isinstance(value, bool):
        return "#t" if value else "#f"
    if isinstance(value, float):
        return f"{value:.6f}"
    return value  # strings are stored quoted


class FakeDLCPro:
    """DeCoP command-line server with a handful of parameters (floats, booleans, strings)."""

    _REF = re.compile(r"\(param-ref '([\w:-]+)\)$")
    _SET = re.compile(r"\(param-set! '([\w:-]+) (\S+)\)$")

    def __init__(self, serial, label, lasers, host="127.0.0.1", port=0, eol="\r\n", echo=False):
        self.serial, self.label, self.host = serial, label, host
        self.eol, self.echo = eol, echo
        self.params = {
            "system-type": '"DLCpro"', "serial-number": f'"{serial}"',
            "system-label": f'"{label}"', "fw-ver": '"3.3.3"',
        }
        for n, d in lasers.items():
            p = f"laser{n}:"
            self.params.update({
                p + "product-name": f'"{d.get("product", "DL pro")}"', p + "type": '"DL pro"',
                p + "label": f'"{d.get("label", "")}"', p + "scan:unit": '"V"',
                p + "scan:enabled": False, p + "scan:offset": float(d.get("offset", 70.0)),
                p + "emission": True,
                p + "dl:cc:current-set": float(d.get("current", 150.0)),
                p + "dl:cc:current-act": float(d.get("current", 150.0)),
                p + "dl:cc:current-clip": float(d.get("clip", 200.0)),
                p + "dl:lock:lock-enabled": bool(d.get("locked", False)),
                p + "dl:lock:state-txt": '"off"',
            })
        self.sets = 0
        self.log = []  # every accepted write: (parameter, value)
        self._srv = socket.create_server((host, port))
        self._srv.settimeout(0.2)
        self.port = self._srv.getsockname()[1]
        self._conns = []
        self._closed = False
        threading.Thread(target=self._serve, daemon=True).start()

    @property
    def key(self) -> str:
        return f"{self.host}:{self.port}"

    def offset(self, n: int) -> float:
        return self.params[f"laser{n}:scan:offset"]

    def current(self, n: int) -> float:
        return self.params[f"laser{n}:dl:cc:current-set"]

    def written(self, param: str) -> list:
        """Values written to `param`, oldest first."""
        return [v for p, v in self.log if p == param]

    def discovery_reply(self) -> bytes:
        return (f'("{self.serial}" "DLCpro" "3.3.3" "00:00:00:00:00:00" "" "{self.label}" 0 '
                f'"{self.host}" {self.port} 0)').encode()

    def drop_connections(self):
        for c in self._conns:
            try:
                c.shutdown(socket.SHUT_RDWR)
                c.close()
            except OSError:
                pass
        self._conns.clear()

    def close(self):
        self._closed = True
        self._srv.close()
        self.drop_connections()

    def _serve(self):
        while not self._closed:
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.settimeout(None)
            self._conns.append(conn)
            threading.Thread(target=self._client, args=(conn,), daemon=True).start()

    def _client(self, conn):
        prompt = (self.eol + "> ").encode()
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.sendall(b"DeCoF Command Line" + prompt)
            buf = b""
            while True:
                data = conn.recv(4096)
                if not data:
                    return
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    cmd = line.decode().strip()
                    reply = self.execute(cmd)
                    if self.echo:
                        reply = cmd + self.eol + reply
                    conn.sendall(reply.encode() + prompt)
        except OSError:
            return
        finally:
            conn.close()

    def execute(self, cmd: str) -> str:
        m = self._REF.match(cmd)
        if m:
            v = self.params.get(m.group(1))
            if v is None:
                return f"Error: -14 unknown parameter '{m.group(1)}'"
            return _decop(v)
        m = self._SET.match(cmd)
        if m:
            name, value = m.groups()
            old = self.params.get(name)
            if isinstance(old, bool):
                if value not in ("#t", "#f"):
                    return "-3"
                new = value == "#t"
            elif isinstance(old, float):
                try:
                    new = float(value)
                except ValueError:
                    return "-3"
            else:
                return f"Error: -14 cannot set '{name}'"
            if name.endswith("dl:cc:current-set") and new > self.params[name[:-3] + "clip"]:
                return "Error: -21 above current-clip"
            self.params[name] = new
            self.sets += 1
            self.log.append((name, new))
            return "0"
        return "Error: -1 unknown command"


class FakeDiscovery(threading.Thread):
    """Answers 'whoareyou?' like real controllers do."""

    def __init__(self, devices, port=DISCOVERY_PORT, host="127.0.0.1"):
        super().__init__(daemon=True, name="fake discovery")
        self.devices = devices
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self._closed = False
        self.start()

    def run(self):
        while not self._closed:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                return
            if data.strip() == b"whoareyou?":
                for d in self.devices:
                    self.sock.sendto(d.discovery_reply(), addr)

    def close(self):
        self._closed = True
        self.sock.close()


class SimWavemeter:
    """The wavemeter's own error: a slow relative drift common to all channels, plus noise.

    drift: optional function of time giving the relative error (for tests)."""

    def __init__(self, amplitude=1.5e-8, period=1800.0, walk=2e-11, noise=0.3, drift=None):
        self.amplitude, self.period = amplitude, period  # relative error, s
        self.walk_rate, self.noise = walk, noise          # per sqrt(s); MHz rms per reading
        self.drift = drift
        self.walk = 0.0
        self.t = time.monotonic()
        self.phase = random.uniform(0, 2 * math.pi)

    def jump(self, relative: float):
        """A sudden step of the wavemeter's error (e.g. its own recalibration)."""
        self.walk += relative

    def epsilon(self, now: float) -> float:
        if self.drift is not None:
            return self.drift(now) + self.walk
        dt, self.t = max(now - self.t, 0.0), now
        self.walk += random.gauss(0.0, self.walk_rate * math.sqrt(dt))
        return self.amplitude * math.sin(2 * math.pi * now / self.period + self.phase) + self.walk

    def read(self, f: float, now: float) -> float:
        return f * (1.0 + self.epsilon(now)) + random.gauss(0.0, self.noise) * 1e-6


class ECDL:
    """An external-cavity diode laser (a DL pro): which external-cavity mode lases, and where.

    Offsets in GHz from f0, at scan offset (piezo) V, diode current I (mA) and temperature T (K):
        mode m      f_m   = m*fsr + kv*(V - v0) + ki*(I - i0) + kt*T
        selection   f_sel =         ksv*(V - v0) + ksi*(I - i0) + kst*T + shift
    The selection filter is the grating together with the diode chip's own modes; the current
    moves it fast (chip modes) and the mode slowly, the piezo the other way round. The lasing
    mode keeps lasing while its misalignment |f_m - f_sel| < fsr/2 + hysteresis, then the laser
    hops to the best aligned mode. Near the edge (`multimode`) it runs on two modes at once and
    the wavemeter reads noise, sometimes nonsense. With the defaults a mode is stable over
    2.7 mA of current (3.4 mA counting hysteresis) and 70 V of piezo, and starts out centered.
    """

    def __init__(self, f0, fsr=6.0, kv=0.4, ki=-0.25, ksv=None, ksi=-2.0, kt=-4.0, kst=-1.0,
                 hysteresis=0.6, multimode=0.3, v0=70.0, i0=150.0, seed=None):
        self.f0, self.fsr = f0, fsr
        self.kv, self.ki = kv, ki
        self.ksv = 0.8 * kv if ksv is None else ksv
        self.ksi, self.kt, self.kst = ksi, kt, kst
        self.hysteresis, self.multimode = hysteresis, multimode
        self.v0, self.i0 = v0, i0
        self.T = 0.0       # K
        self.shift = 0.0   # GHz: the selection filter moved by itself (e.g. the diode chip warmed)
        self.m = 0
        self.rng = random.Random(seed)

    def misalignment(self, m: int, v: float, i: float) -> float:
        return (m * self.fsr + (self.kv - self.ksv) * (v - self.v0) + (self.ki - self.ksi) * (i - self.i0)
                + (self.kt - self.kst) * self.T - self.shift)

    def lase(self, v: float, i: float) -> float:
        """Settle which mode lases at (v, i), with hysteresis; returns its misalignment (GHz)."""
        a = self.misalignment(self.m, v, i)
        if abs(a) > self.fsr / 2 + self.hysteresis:
            self.m -= round(a / self.fsr)
            a = self.misalignment(self.m, v, i)
        return a

    def mode_ghz(self, v: float, i: float) -> float:
        """Frequency of the lasing mode, GHz from f0 (call lase() first)."""
        return self.m * self.fsr + self.kv * (v - self.v0) + self.ki * (i - self.i0) + self.kt * self.T

    def freq(self, v: float, i: float, extra_mhz: float = 0.0) -> float:
        """What the laser emits at (v, i), THz, as a wavemeter would see it."""
        a = self.lase(v, i)
        ghz = self.mode_ghz(v, i) + extra_mhz * 1e-3
        if abs(a) > self.fsr / 2 - self.multimode:  # two modes: noisy, and now and then nonsense
            ghz += self.rng.gauss(0.0, 0.03)
            if self.rng.random() < 0.05:
                ghz -= math.copysign(self.rng.uniform(0.2, 0.8) * self.fsr, a)
        return self.f0 + ghz * 1e-3

    def nudge(self, ghz: float):
        """The selection filter moves by `ghz` (a temperature step of the diode, say): enough
        and the laser hops."""
        self.shift += ghz


class SimCavityLock:
    """A DLC pro lock (Pound-Drever-Hall through a FALC) to a reference cavity, acting on the
    laser's piezo, as far as the wavemeter can tell.

    Enabled, it locks to the cavity resonance nearest the laser's free-running frequency (TEM00
    every fsr GHz, and weaker TEM01 resonances `tem` x fsr above them) and follows it as long as
    its output range lasts and the laser stays on its mode. If the laser hops, it relocks to
    whatever resonance is nearest then: 'locked, but on another peak'. Disabled, its output
    goes to zero."""

    def __init__(self, f_res, fsr=1.5, tem=0.37, range_v=30.0):
        self.f_res, self.fsr, self.tem, self.range_v = f_res, fsr, tem, range_v
        self.enabled = False
        self.peak = None   # THz: the resonance it is locked to
        self.dv = 0.0      # V: its output, added to the piezo

    @property
    def state(self) -> str:
        return "locked" if self.peak is not None else ("relocking" if self.enabled else "off")

    def nearest(self, f: float) -> float:
        k = (f - self.f_res) * 1e3 / self.fsr
        k00 = round(k)
        k01 = round(k - self.tem) + self.tem
        return self.f_res + (k00 if abs(k - k00) <= abs(k - k01) else k01) * self.fsr * 1e-3

    def jump(self, peaks: float = 1.0):
        """Relock `peaks` free spectral ranges away (as after a knock)."""
        if self.peak is not None:
            self.peak += peaks * self.fsr * 1e-3

    def frequency(self, laser: ECDL, v: float, i: float, extra_mhz: float = 0.0) -> float:
        if not self.enabled:
            self.peak, self.dv = None, 0.0
            return laser.freq(v, i, extra_mhz)
        if self.peak is not None:  # follow the resonance with the piezo
            m = laser.m
            laser.lase(v + self.dv, i)
            for _ in range(3):
                if laser.m != m:
                    break
                free = laser.f0 + laser.mode_ghz(v + self.dv, i) * 1e-3 + extra_mhz * 1e-6
                self.dv += (self.peak - free) * 1e3 / laser.kv
                laser.lase(v + self.dv, i)
            if laser.m != m or abs(self.dv) > self.range_v:
                self.peak = None  # it hopped, or ran out of range: lost
        if self.peak is None:  # the lock's relock: the nearest resonance from here
            laser.lase(v + self.dv, i)
            free = laser.f0 + laser.mode_ghz(v + self.dv, i) * 1e-3 + extra_mhz * 1e-6
            peak = self.nearest(free)
            dv = self.dv + (peak - free) * 1e3 / laser.kv
            if abs(dv) <= self.range_v:
                self.peak, self.dv = peak, dv
                laser.lase(v + dv, i)
        if self.peak is not None:
            return self.peak
        return laser.freq(v + self.dv, i, extra_mhz)


class SimLaser:
    """One laser on the wavemeter.

    With a controller: an ECDL tuned by that controller's scan offset and diode current, drifting
    by itself, and optionally locked to a reference cavity by its (simulated) DLC pro lock.
    cavity=True: the demo's reference laser, locked to its cavity by something the program does
    not see, so a fixed frequency unless a fault is injected."""

    def __init__(self, ch, wavelength_nm, tuning=0.4, dlc=None, n=1, offset0=70.0,
                 drift=3.0, signal=True, cavity=False, lock=None, seed=None):
        self.ch, self.f0 = ch, C_NM_THZ / wavelength_nm
        self.tuning = tuning      # GHz per V of scan offset (sign matters)
        self.dlc, self.n, self.offset0 = dlc, n, offset0
        self.drift = drift        # MHz/sqrt(s) random walk of a free-running laser
        self.signal, self.cavity = signal, cavity
        self.walk = 0.0
        self.t = time.monotonic()
        self.phase = random.uniform(0, 2 * math.pi)
        self.hop = 0.0            # MHz: a cavity-locked laser on another mode
        self._hop_until = math.inf
        self._unlocked_until = self._dark_until = 0.0
        self.ecdl = None
        if dlc is not None:
            self.ecdl = ECDL(self.f0, kv=tuning, v0=offset0, i0=dlc.current(n), seed=seed)
        self.lock = lock          # SimCavityLock: its DLC pro locks it to a cavity
        self._mutex = threading.Lock()

    def true_freq(self, now: float):
        """Where the laser really is (THz), or None when no light reaches the wavemeter."""
        if not self.signal or now < self._dark_until:
            return None
        with self._mutex:
            dt, self.t = max(now - self.t, 0.0), now
            if self.cavity:
                if now >= self._hop_until:
                    self.hop, self._hop_until = 0.0, math.inf
                mhz = self.hop
                if now < self._unlocked_until:  # free running, swept by the relock search
                    self.walk += random.gauss(0.0, 20.0 * math.sqrt(dt))
                    mhz += self.walk + 300.0 * math.sin(2 * math.pi * now / 4.0)
                else:
                    self.walk = 0.0
                return self.f0 + mhz * 1e-6
            self.walk += random.gauss(0.0, self.drift * math.sqrt(dt))
            wobble = self.walk + 20.0 * math.sin(now / 30.0 + self.phase)  # MHz
            if self.ecdl is None:
                return self.f0 + wobble * 1e-6
            p, params = f"laser{self.n}:", self.dlc.params
            v, i = params[p + "scan:offset"], params[p + "dl:cc:current-set"]
            params[p + "dl:cc:current-act"] = i
            if self.lock is None:
                return self.ecdl.freq(v, i, wobble)
            self.lock.enabled = params[p + "dl:lock:lock-enabled"]
            f = self.lock.frequency(self.ecdl, v, i, wobble)
            params[p + "dl:lock:state-txt"] = f'"{self.lock.state}"'
            return f

    # faults of a laser with a controller
    def hop_mode(self, direction: int = 1):
        """The diode warms or cools a little: the laser hops one mode up (+1) or down (-1)."""
        with self._mutex:
            e = self.ecdl
            e.nudge(direction * (e.fsr / 2 + e.hysteresis + 0.3) + e.misalignment(e.m, *self._vi()))

    def _vi(self):
        p = f"laser{self.n}:"
        v = self.dlc.params[p + "scan:offset"] + (self.lock.dv if self.lock is not None else 0.0)
        return v, self.dlc.params[p + "dl:cc:current-set"]

    # faults of the reference laser (cavity=True)
    def mode_hop(self, mhz: float, seconds: float | None = None):
        self.hop = mhz
        self._hop_until = time.monotonic() + seconds if seconds else math.inf

    def unlock(self, seconds: float):
        self._unlocked_until = time.monotonic() + seconds

    def dark(self, seconds: float):
        self._dark_until = time.monotonic() + seconds


class SimSource(Source):
    """Cycles through the lasers like a fibre switch, one exposure per channel."""

    label = "simulation"

    def __init__(self, sink, lasers, exposure=0.02, wavemeter=None):
        super().__init__(sink, exposure)
        self.lasers = lasers
        self.wavemeter = wavemeter or SimWavemeter()

    def cavity_laser(self):
        return next((laser for laser in self.lasers if laser.cavity), None)

    def laser(self, ch: int):
        return next((laser for laser in self.lasers if laser.ch == ch), None)

    def run(self):
        self._set_active([laser.ch for laser in self.lasers])
        self._set_state(True)
        i = 0
        while not self._stop_ev.wait(self.interval):
            laser = self.lasers[i % len(self.lasers)]
            i += 1
            now = time.monotonic()
            f = laser.true_freq(now)
            self._push({laser.ch: -1.0 if f is None else self.wavemeter.read(f, now)}, now)  # -1: no signal


class Demo:
    """Three fake controllers: two with two DL pros each, wavemeter-locked; one with an SHG laser
    that its DLC pro locks to a cavity. Plus a channel without light, and a 729 nm laser locked
    to a cavity: the reference."""

    CHANNELS = [  # ch, vacuum wavelength (nm), tuning (GHz/V), controller, laser, name
        (1, 396.959120, +0.35, 0, 1, "397"),
        (2, 866.452050, -0.30, 0, 2, "866"),
        (3, 854.443950, +0.40, 1, 1, "854"),
        (4, 422.791900, +0.30, 1, 2, "423"),
        (5, 780.241210, +0.30, None, 0, "780"),
        (6, 729.347520, 0.0, None, 0, "729 ref"),
        (7, 739.052750, +0.40, 2, 1, "739 SHG"),
    ]
    REFERENCE = 6
    SHG = 7  # cavity-locked by its DLC pro: try the mode-hop recovery on it

    def __init__(self, discovery_port=DISCOVERY_PORT, wavemeter=None, exposure=0.02):
        self.dlcs = [
            FakeDLCPro("SIM-000123", "Ca+ blue", {1: {"product": "DL pro 397"}, 2: {"product": "DL pro 866"}}),
            FakeDLCPro("SIM-000456", "Ca+ red", {1: {"product": "DL pro 854"}, 2: {"product": "DL pro 423"}}),
            FakeDLCPro("SIM-000789", "Yb+ SHG", {1: {"product": "TA-SHG pro 739", "locked": True}}),
        ]
        try:
            self.responder = FakeDiscovery(self.dlcs, port=discovery_port)
        except OSError:  # port taken: the GUI gets the devices directly instead
            self.responder = None
        self.lasers = [
            SimLaser(ch, wl, k, self.dlcs[c] if c is not None else None, n,
                     drift=0.5 if ch == self.SHG else 3.0,
                     signal=c is not None or ch == self.REFERENCE, cavity=ch == self.REFERENCE)
            for ch, wl, k, c, n, _ in self.CHANNELS
        ]
        shg = self.laser(self.SHG)
        shg.lock = SimCavityLock(shg.f0)
        self.wavemeter = wavemeter or SimWavemeter()
        self.exposure = exposure
        self.names = {ch: name for ch, *_, name in self.CHANNELS}
        self.targets = ["127.0.0.1"]  # never broadcast on the real LAN in demo mode
        self.extra_devices = [] if self.responder else [d.key for d in self.dlcs]

    def make_source(self, sink):
        return SimSource(sink, self.lasers, self.exposure, self.wavemeter)

    def laser(self, ch: int) -> SimLaser:
        return next(laser for laser in self.lasers if laser.ch == ch)

    def wavelength(self, ch: int) -> float:
        return next(wl for c, wl, *_ in self.CHANNELS if c == ch)

    def close(self):
        if self.responder:
            self.responder.close()
        for d in self.dlcs:
            d.close()
