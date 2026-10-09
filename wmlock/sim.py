"""Simulation: a fake multichannel wavemeter and fake DLC pro controllers.

Used by `python -m wmlock --sim` and by the tests. The fake controllers speak
the DeCoP command line over TCP and answer UDP discovery on 127.0.0.1, so the
whole stack (discovery, protocol, lock, reference) runs for real. Each laser's
true frequency follows the scan offset of its fake controller plus its own
drift; the simulated wavemeter adds a slow common error and noise on top.
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


class FakeDLCPro:
    """DeCoP command-line server with a handful of parameters."""

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
                p + "scan:enabled": "#f", p + "scan:offset": float(d.get("offset", 70.0)),
            })
        self.sets = 0
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
            return f"{v:.6f}" if isinstance(v, float) else v
        m = self._SET.match(cmd)
        if m:
            name, value = m.groups()
            if not isinstance(self.params.get(name), float):
                return f"Error: -14 cannot set '{name}'"
            try:
                self.params[name] = float(value)
            except ValueError:
                return "-3"
            self.sets += 1
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


class SimLaser:
    """A free-running laser tuned by its controller's scan offset (and drifting by itself),
    or one locked to a cavity: fixed frequency, unless a fault is injected."""

    def __init__(self, ch, wavelength_nm, tuning=0.4, dlc=None, n=1, offset0=70.0,
                 drift=3.0, signal=True, cavity=False):
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

    def true_freq(self, now: float):
        """Where the laser really is (THz), or None when no light reaches the wavemeter."""
        if not self.signal or now < self._dark_until:
            return None
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
        offset = self.dlc.offset(self.n) if self.dlc else self.offset0
        mhz = self.tuning * 1e3 * (offset - self.offset0) + self.walk + 20.0 * math.sin(now / 30.0 + self.phase)
        return self.f0 + mhz * 1e-6

    # faults of a cavity-locked laser
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
    """Two fake controllers with two lasers each, a channel without light, and a 729 nm laser
    locked to a cavity: the reference."""

    CHANNELS = [  # ch, vacuum wavelength (nm), tuning (GHz/V), controller, laser, name
        (1, 396.959120, +0.35, 0, 1, "397"),
        (2, 866.452050, -0.30, 0, 2, "866"),
        (3, 854.443950, +0.40, 1, 1, "854"),
        (4, 422.791900, +0.30, 1, 2, "423"),
        (5, 780.241210, +0.30, None, 0, "780"),
        (6, 729.347520, 0.0, None, 0, "729 ref"),
    ]
    REFERENCE = 6

    def __init__(self, discovery_port=DISCOVERY_PORT, wavemeter=None, exposure=0.02):
        self.dlcs = [
            FakeDLCPro("SIM-000123", "Ca+ blue", {1: {"product": "DL pro 397"}, 2: {"product": "DL pro 866"}}),
            FakeDLCPro("SIM-000456", "Ca+ red", {1: {"product": "DL pro 854"}, 2: {"product": "DL pro 423"}}),
        ]
        try:
            self.responder = FakeDiscovery(self.dlcs, port=discovery_port)
        except OSError:  # port taken: the GUI gets the devices directly instead
            self.responder = None
        self.lasers = [
            SimLaser(ch, wl, k, self.dlcs[c] if c is not None else None, n,
                     signal=c is not None or ch == self.REFERENCE, cavity=ch == self.REFERENCE)
            for ch, wl, k, c, n, _ in self.CHANNELS
        ]
        self.wavemeter = wavemeter or SimWavemeter()
        self.exposure = exposure
        self.names = {ch: name for ch, *_, name in self.CHANNELS}
        self.targets = ["127.0.0.1"]  # never broadcast on the real LAN in demo mode
        self.extra_devices = [] if self.responder else [d.key for d in self.dlcs]

    def make_source(self, sink):
        return SimSource(sink, self.lasers, self.exposure, self.wavemeter)

    def laser(self, ch: int) -> SimLaser:
        return next(laser for laser in self.lasers if laser.ch == ch)

    def close(self):
        if self.responder:
            self.responder.close()
        for d in self.dlcs:
            d.close()
