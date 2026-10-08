"""Simulation: a fake multichannel wavemeter and fake DLC pro controllers.

Used by `python -m wmlock --sim` and by the tests. The fake controllers speak
the DeCoP command line over TCP and answer UDP discovery on 127.0.0.1, so the
whole stack (discovery, protocol, lock) runs for real. Each simulated laser's
frequency follows the scan offset of its fake controller, plus drift and noise.
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


class SimLaser:
    def __init__(self, ch, wavelength_nm, tuning=0.4, dlc=None, n=1, offset0=70.0,
                 drift=3.0, noise=0.3, signal=True):
        self.ch, self.f0 = ch, C_NM_THZ / wavelength_nm
        self.tuning = tuning      # GHz per V of scan offset (sign matters)
        self.dlc, self.n, self.offset0 = dlc, n, offset0
        self.drift, self.noise = drift, noise  # MHz/sqrt(s) random walk, MHz rms
        self.signal = signal
        self.walk = 0.0
        self.t = time.monotonic()
        self.phase = random.uniform(0, 2 * math.pi)

    def measure(self, now: float) -> float:
        """Frequency in THz, or a wlmData error code."""
        if not self.signal:
            return -1.0  # ErrNoSignal
        dt, self.t = now - self.t, now
        self.walk += random.gauss(0.0, self.drift * math.sqrt(max(dt, 0.0)))
        offset = self.dlc.offset(self.n) if self.dlc else self.offset0
        mhz = (self.tuning * 1e3 * (offset - self.offset0) + self.walk
               + 20.0 * math.sin(now / 30.0 + self.phase) + random.gauss(0.0, self.noise))
        return self.f0 + mhz * 1e-6


class SimSource(Source):
    """Cycles through the lasers like a fibre switch, one exposure per channel."""

    label = "simulation"

    def __init__(self, sink, lasers, exposure=0.02):
        super().__init__(sink, exposure)
        self.lasers = lasers

    def run(self):
        self._set_active([laser.ch for laser in self.lasers])
        self._set_state(True)
        i = 0
        while not self._stop_ev.wait(self.interval):
            laser = self.lasers[i % len(self.lasers)]
            i += 1
            now = time.monotonic()
            self._push({laser.ch: laser.measure(now)}, now)


class Demo:
    """Two fake controllers with two lasers each, plus a channel without light."""

    CHANNELS = [  # ch, vacuum wavelength (nm), tuning (GHz/V), controller, laser, name
        (1, 396.959120, +0.35, 0, 1, "397"),
        (2, 866.452050, -0.30, 0, 2, "866"),
        (3, 854.443950, +0.40, 1, 1, "854"),
        (4, 422.791900, +0.30, 1, 2, "423"),
        (5, 780.241210, +0.30, None, 0, "780"),
    ]

    def __init__(self, discovery_port=DISCOVERY_PORT):
        self.dlcs = [
            FakeDLCPro("SIM-000123", "Ca+ blue", {1: {"product": "DL pro 397"}, 2: {"product": "DL pro 866"}}),
            FakeDLCPro("SIM-000456", "Ca+ red", {1: {"product": "DL pro 854"}, 2: {"product": "DL pro 423"}}),
        ]
        try:
            self.responder = FakeDiscovery(self.dlcs, port=discovery_port)
        except OSError:  # port taken: the GUI gets the devices directly instead
            self.responder = None
        self.lasers = [
            SimLaser(ch, wl, k, self.dlcs[c] if c is not None else None, n, signal=c is not None)
            for ch, wl, k, c, n, _ in self.CHANNELS
        ]
        self.names = {ch: name for ch, *_, name in self.CHANNELS}
        self.targets = ["127.0.0.1"]  # never broadcast on the real LAN in demo mode
        self.extra_devices = [] if self.responder else [d.key for d in self.dlcs]

    def make_source(self, sink):
        return SimSource(sink, self.lasers)

    def close(self):
        if self.responder:
            self.responder.close()
        for d in self.dlcs:
            d.close()
