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

import base64
import math
import random
import re
import socket
import threading
import time

from .dlcpro import DISCOVERY_PORT
from .wavemeter import C_NM_THZ, Source


def _decop(value) -> str:
    if callable(value):
        value = value()
    if isinstance(value, bool):
        return "#t" if value else "#f"
    if isinstance(value, float):
        return f"{value:.6f}"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, bytes):
        return "&" + base64.b64encode(value).decode()
    if isinstance(value, tuple):
        return "(" + " ".join(_decop(v) for v in value) + ")"
    return value  # strings are stored quoted


class FakeDLCPro:
    """DeCoP command-line server with a handful of parameters (floats, ints, booleans, strings,
    binary data, tuples; a callable is asked for the value on every read) and commands.

    on_set[param](value) is called after a write; commands[name](*args) runs (exec 'name ...)
    and returns its output, or raises CommandError.

    A laser's measured current (current-act) is its set current plus current-offset, plus the
    feedforward from the piezo (feedforward-factor mA/V times the scan offset, when enabled), plus
    a fixed bias (its config's "act_offset", mA): set and measured current differ, as they do."""

    _REF = re.compile(r"\(param-ref '([\w:-]+)\)$")
    _SET = re.compile(r"\(param-set! '([\w:-]+) (\S+)\)$")
    _EXEC = re.compile(r"\(exec '([\w:-]+)((?: +\S+)*)\)$")

    def __init__(self, serial, label, lasers, host="127.0.0.1", port=0, eol="\r\n", echo=False, serve=True):
        """serve=False: no TCP server, only execute() (for synchronous tests)."""
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
                p + "scan:enabled": bool(d.get("scan", False)), p + "scan:offset": float(d.get("offset", 70.0)),
                p + "scan:amplitude": float(d.get("amplitude", 2.5)), p + "scan:frequency": 20.0,
                p + "emission": True,
                p + "dl:cc:current-set": float(d.get("current", 150.0)),
                p + "dl:cc:current-act": lambda n=n: self.current_act(n),
                p + "dl:cc:current-offset": 0.0,
                p + "dl:cc:feedforward-enabled": bool(d.get("feedforward", 0.0)),
                p + "dl:cc:feedforward-factor": float(d.get("feedforward", 0.0)),
                p + "dl:cc:current-clip": float(d.get("clip", 200.0)),
                p + "dl:lock:lock-enabled": bool(d.get("locked", False)),
                p + "dl:lock:state-txt": '"off"',
            })
        self.bias = {n: float(d.get("act_offset", 0.0)) for n, d in lasers.items()}
        self.sets = 0
        self.log = []  # every accepted write: (parameter, value); every command: (name, args)
        self.on_set = {}
        self.commands = {}
        self.mutex = threading.RLock()  # held while a command line is executed
        self._conns = []
        self._closed = False
        self._srv, self.port = None, 0
        if serve:
            self._srv = socket.create_server((host, port))
            self._srv.settimeout(0.2)
            self.port = self._srv.getsockname()[1]
            threading.Thread(target=self._serve, daemon=True).start()

    @property
    def key(self) -> str:
        return f"{self.host}:{self.port}"

    def offset(self, n: int) -> float:
        return self.params[f"laser{n}:scan:offset"]

    def current(self, n: int) -> float:
        return self.params[f"laser{n}:dl:cc:current-set"]

    def current_act(self, n: int) -> float:
        """The diode current, as measured: what the laser runs at."""
        p, d = f"laser{n}:", self.params
        i = d[p + "dl:cc:current-set"] + d[p + "dl:cc:current-offset"] + self.bias.get(n, 0.0)
        if d[p + "dl:cc:feedforward-enabled"]:
            i += d[p + "dl:cc:feedforward-factor"] * d[p + "scan:offset"]
        return i

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
        if self._srv is not None:
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
        with self.mutex:
            return self._execute(cmd)

    def _execute(self, cmd: str) -> str:
        m = self._REF.match(cmd)
        if m:
            v = self.params.get(m.group(1))
            if v is None:
                return f"Error: -14 unknown parameter '{m.group(1)}'"
            try:
                return _decop(v)
            except CommandError as e:
                return f"Error: {e}"
        m = self._SET.match(cmd)
        if m:
            name, value = m.groups()
            old = self.params.get(name)
            if callable(old):
                return f"Error: -14 cannot set '{name}'"
            if isinstance(old, bool):
                if value not in ("#t", "#f"):
                    return "-3"
                new = value == "#t"
            elif isinstance(old, (float, int)):
                try:
                    new = float(value) if isinstance(old, float) else int(value)
                except ValueError:
                    return "-3"
            else:
                return f"Error: -14 cannot set '{name}'"
            if name.endswith("dl:cc:current-set") and new > self.params[name[:-3] + "clip"]:
                return "Error: -21 above current-clip"
            self.params[name] = new
            hook = self.on_set.get(name)
            if hook is not None:
                try:
                    hook(new)
                except CommandError as e:
                    self.params[name] = old
                    return f"Error: {e}"
            self.sets += 1
            self.log.append((name, new))
            return "0"
        m = self._EXEC.match(cmd)
        if m:
            name, args = m.group(1), [_parse_arg(a) for a in m.group(2).split()]
            fn = self.commands.get(name)
            if fn is None:
                return f"Error: -1 unknown command '{name}'"
            self.log.append((name, tuple(args)))
            try:
                return fn(*args)
            except CommandError as e:
                return f"Error: {e}"
            except TypeError:
                return "Error: -3 wrong arguments"
        return "Error: -1 unknown command"


class CommandError(Exception):
    """A fake controller refuses a command, e.g. CommandError('-7 lockpoint not found')."""


def _parse_arg(text: str):
    if text in ("#t", "#f"):
        return text == "#t"
    if len(text) >= 2 and text[0] == text[-1] == '"':
        return text[1:-1]
    try:
        return int(text)
    except ValueError:
        return float(text)


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
    mode keeps lasing while its misalignment f_m - f_sel stays within fsr/2 + hysteresis either
    way (more one way than the other with `skew`), then the laser hops to the best aligned mode:
    sweeping up, it hops onto a mode later than it hops off it sweeping down, and in between
    either mode can lase, whichever it was on. Shortly before it hops off (`multimode`, and no
    more than 0.15 fsr more) it runs on two modes at once and the wavemeter reads noise,
    sometimes nonsense. With the defaults a mode is alone over 2.7 mA of current (it holds over
    3.4 mA) and 70 V of piezo, and starts out centered.
    """

    def __init__(self, f0, fsr=6.0, kv=0.4, ki=-0.25, ksv=None, ksi=-2.0, kt=-4.0, kst=-1.0,
                 hysteresis=0.6, multimode=0.3, v0=70.0, i0=150.0, seed=None, mm_noise=0.03, skew=0.0):
        self.f0, self.fsr = f0, fsr
        self.mm_noise = mm_noise  # GHz rms the wavemeter reads where it runs on two modes
        self.kv, self.ki = kv, ki
        self.ksv = 0.8 * kv if ksv is None else ksv
        self.ksi, self.kt, self.kst = ksi, kt, kst
        self.hysteresis, self.multimode, self.skew = hysteresis, multimode, skew
        self.v0, self.i0 = v0, i0
        self.T = 0.0       # K
        self.shift = 0.0   # GHz: the selection filter moved by itself (e.g. the diode chip warmed)
        self.m = 0
        self.rng = random.Random(seed)

    def misalignment(self, m: int, v: float, i: float) -> float:
        return (m * self.fsr + (self.kv - self.ksv) * (v - self.v0) + (self.ki - self.ksi) * (i - self.i0)
                + (self.kt - self.kst) * self.T - self.shift)

    def limits(self):
        """The misalignments (GHz) at which the lasing mode hops off: below, above."""
        return (-(self.fsr / 2 + self.hysteresis * (1 - self.skew)),
                self.fsr / 2 + self.hysteresis * (1 + self.skew))

    def zone(self, a: float):
        """(|a| where a neighbouring mode starts to lase too, |a| where this one hops off), on a's side."""
        lo, hi = self.limits()
        full = hi if a >= 0 else -lo
        return max(self.fsr / 2 - self.multimode, full - self.multimode - 0.15 * self.fsr), full

    def margins(self, v: float, i: float):
        """mA from (v, i) to where another mode could lase as well, and to where this one hops off,
        the nearer way each (negative: past it). Call lase() first."""
        a = self.misalignment(self.m, v, i)
        lo, hi = self.limits()
        c = abs(self.ki - self.ksi)
        return min(a - (hi - self.fsr), (lo + self.fsr) - a) / c, min(a - lo, hi - a) / c

    def lase(self, v: float, i: float) -> float:
        """Settle which mode lases at (v, i), with hysteresis; returns its misalignment (GHz)."""
        a = self.misalignment(self.m, v, i)
        lo, hi = self.limits()
        if not lo < a < hi:
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
        if abs(a) > self.zone(a)[0] and self.mm_noise:  # two modes: noisy, now and then nonsense
            ghz += self.rng.gauss(0.0, self.mm_noise)
            if self.rng.random() < 0.05:
                ghz -= math.copysign(self.rng.uniform(0.2, 0.8) * self.fsr, a)
        return self.f0 + ghz * 1e-3

    def components(self, v: float, i: float):
        """The modes it emits at (v, i): [(power fraction, GHz from f0)] (call lase() first). Shortly
        before it hops off (zone) the neighbouring mode takes up to half the power."""
        a = self.misalignment(self.m, v, i)
        f = self.mode_ghz(v, i)
        edge, full = self.zone(a)
        p2 = 0.5 * min(max((abs(a) - edge) / max(full - edge, 1e-9), 0.0), 1.0)
        if p2 <= 0.0:
            return [(1.0, f)]
        return [(1.0 - p2, f), (p2, f - math.copysign(self.fsr, a))]

    def nudge(self, ghz: float):
        """The selection filter moves by `ghz` (a temperature step of the diode, say): enough
        and the laser hops."""
        self.shift += ghz


class SimLocked:
    """A laser its DLC pro locks to a reference cavity, optionally frequency-doubled in an SHG
    cavity its DLC pro locks to it (a TA-SHG pro): everything the DLC pro shows of it and every
    parameter and command mode-hop recovery uses, registered on a FakeDLCPro.

    Reference cavity: TEM00 resonances every `fsr` GHz (one at f_res), weaker transverse modes
    (`tem`: offset in FSR, relative height) between; transmission on Fine In 1 (t00 V at a TEM00
    peak), the PDH error signal on Fast In 3 (amplitude `pdh` V, sidebands 20 MHz out), with a DC
    offset `pdh_dc` plus the FALC pro's input offset. The lock (lock-enabled):
      lockpoints=True   locks to the resonance at the selected lockpoint (select-lockpoint
                        x y type, which must name one of the candidates the DLC pro found in
                        the last scan, type = the candidate's type character as a number);
                        no lockpoint, no lock
      lockpoints=False  (lock without lockpoint) locks to the resonance nearest the laser
    and follows it with the piezo while the laser stays on its mode; the error signal's DC level
    puts it off the resonance center (less transmission), and if it is more than half the PDH
    amplitude it cannot lock at all. If the laser hops, the lock relocks to the resonance nearest
    it then: locked, but on another peak. Disabling the lock while scan:enabled is on lets the
    scan run, as a DLC pro does.

    SHG: the intra-cavity signal while its cavity is scanned has an Airy peak (plus small
    transverse modes) per FSR for each mode the master emits (ECDL.components); its lock locks to
    the master's main mode when that has most of the power, and holds while the scan is off."""

    N = 1000  # samples in a scope trace

    def __init__(self, dlc: "FakeDLCPro", n: int, ecdl: "ECDL", f_res: float, shg: bool = True,
                 lockpoints: bool = True, fsr: float = 1.5, lw: float = 6.0, tem=((0.37, 0.18), (0.71, 0.08)),
                 t00: float = 0.8, pdh: float = 0.4, pdh_dc: float = 0.0, falc: int = 1, range_v: float = 30.0,
                 falc_in_trace: bool = True, falc_sign: int = 1, seed=None):
        """falc_in_trace: the error signal on the scope includes the FALC's input offset, which adds
        to it (falc_sign 1) or subtracts (-1)."""
        self.dlc, self.n, self.ecdl = dlc, n, ecdl
        self.p = p = f"laser{n}:"
        self.f_res, self.fsr, self.lw, self.tem, self.t00 = f_res, fsr, lw, tuple(tem), t00
        self.pdh, self.pdh_dc, self.sideband = pdh, pdh_dc, 20.0
        self.has_shg, self.lockpoints, self.falc, self.range_v = shg, lockpoints, falc, range_v
        self.falc_in_trace, self.falc_sign = falc_in_trace, falc_sign
        self.rng = random.Random(seed)
        self.mutex = threading.RLock()
        self.wobble = 0.0       # MHz: the free-running laser's drift (from SimLaser)
        self.peak = None        # THz: the resonance the lock holds
        self.dv = 0.0           # V: the lock's piezo correction
        self.lockpoint = None   # (x, y, type)
        self.cands = []         # lockpoint candidates found in the last scan
        self.relocks = 0
        # GHz (incommensurate with the master's mode spacing, or a second mode would hide under the first),
        # V of SHG piezo per FSR, finesse
        self.shg_fsr, self.shg_vfsr, self.shg_finesse = 1.37, 9.0, 150.0
        self.shg_tem = ((0.31, 0.06), (0.62, 0.035))
        self.shg_h, self.shg_floor = 1.2, 0.03
        self.shg_phase = self.rng.random()
        d = dlc.params
        d.update({
            p + "dl:lock:lock-without-lockpoint": not lockpoints,
            p + "dl:lock:state": lambda: self.state()[0],
            p + "dl:lock:state-txt": lambda: f'"{self.state()[1]}"',
            p + "dl:lock:spectrum-input-channel": 0, p + "dl:lock:error-channel": 2,
            p + "dl:lock:falc-selection": falc, p + "dl:lock:relock:enabled": True,
            p + "dl:lock:candidate-filter:top": True, p + "dl:lock:candidate-filter:bottom": False,
            p + "dl:lock:candidate-filter:positive-edge": False, p + "dl:lock:candidate-filter:negative-edge": False,
            p + "dl:lock:candidates": self._candidates_blob,
            p + "dl:lock:lockpoint:position": lambda: tuple(self.lockpoint[:2]) if self.lockpoint else (0.0, 0.0),
            p + "dl:lock:lockpoint:type": lambda: '"top"' if self.lockpoint else '""',
            p + "scope:variant": 1, p + "scope:channel1:signal": 0, p + "scope:channel2:signal": 2,
            p + "scope:channelx:xy-signal": 34, p + "scope:timescale": 50.0,
            p + "scope:channelx:unit": lambda: '"V"' if d[p + "scope:variant"] == 1 else '"ms"',
            p + "scope:data": self._scope_blob,
            "io:fine-1:value-act": lambda: self.level(0), "io:fast-3:value-act": lambda: self.level(2),
        })
        if falc:
            d.update({f"falc{falc}:input:offset": 0.0, f"falc{falc}:status-txt": '"ok"',
                      f"falc{falc}:serial-number": '"FALC-SIM"',
                      f"falc{falc}:main:lock-state": lambda: self.peak is not None})
        if shg:
            d.update({
                p + "nlo:shg:lock:lock-enabled": True, p + "nlo:shg:scan:enabled": False,
                p + "nlo:shg:lock:state": lambda: self.shg_state()[0],
                p + "nlo:shg:lock:state-txt": lambda: f'"{self.shg_state()[1]}"',
                p + "nlo:shg:scan:amplitude": 14.0, p + "nlo:shg:scan:frequency": 20.0,
                p + "nlo:shg:scan:offset": 50.0,
                p + "nlo:shg:scope:variant": 0, p + "nlo:shg:scope:channel1:signal": 82,
                p + "nlo:shg:scope:channel2:signal": 90, p + "nlo:shg:scope:channelx:xy-signal": 90,
                p + "nlo:shg:scope:timescale": 50.0, p + "nlo:shg:scope:data": self._shg_blob,
                p + "nlo:pd:shg-int:photodiode": self._intra_now, p + "nlo:pd:shg:power": self._shg_power,
            })
        dlc.on_set[p + "dl:lock:lock-enabled"] = self._lock_enabled
        dlc.on_set[p + "scan:enabled"] = lambda on: self._lose_selection()
        dlc.commands[p + "dl:lock:select-lockpoint"] = self._select
        dlc.commands[p + "dl:lock:find-candidates"] = lambda: (self._master_scan(), "")[1]

    # ------------------------------------------------------------- helpers
    def _get(self, name):
        return self.dlc.params[self.p + name]

    def _vi(self):
        return self._get("scan:offset"), self.dlc.current_act(self.n)

    def dc(self) -> float:
        """The error signal's DC level as the FALC sees it: its own offset plus the input offset."""
        off = self.dlc.params.get(f"falc{self.falc}:input:offset", 0.0) if self.falc else 0.0
        return self.pdh_dc + self.falc_sign * off

    def shown_dc(self) -> float:
        """The error signal's DC level as the scope shows it."""
        return self.dc() if self.falc_in_trace else self.pdh_dc

    def _airy(self, f: float):
        """(transmission V, error signal V without DC) of the reference cavity at f THz."""
        d = (f - self.f_res) * 1e3 / self.fsr  # in FSR
        t = e = 0.0
        for off, h in ((0.0, 1.0),) + self.tem:
            u = (d - off) % 1.0
            u = u - 1.0 if u >= 0.5 else u
            x = 2 * u * self.fsr * 1e3 / self.lw  # detuning in half widths
            t += h / (1 + x * x)
            if off == 0.0:
                s = 2 * self.sideband / self.lw
                e += -x / (1 + x * x) + 0.5 * (x - s) / (1 + (x - s) ** 2) + 0.5 * (x + s) / (1 + (x + s) ** 2)
        return self.t00 * t, self.pdh * e

    def _nearest(self, f: float):
        """The resonance (any transverse mode) nearest f THz, and its relative height."""
        d = (f - self.f_res) * 1e3 / self.fsr
        best = None
        for off, h in ((0.0, 1.0),) + self.tem:
            k = round(d - off) + off
            if best is None or abs(d - k) < abs(d - best[0]):
                best = (k, h)
        return self.f_res + best[0] * self.fsr * 1e-3, best[1]

    def _free(self, v: float, i: float) -> float:
        self.ecdl.lase(v, i)
        return self.ecdl.f0 + self.ecdl.mode_ghz(v, i) * 1e-3 + self.wobble * 1e-6

    def _offset_mhz(self) -> float:
        """How far from the resonance center the lock holds the laser (from the DC level)."""
        r = max(min(self.dc() / self.pdh, 0.5), -0.5)  # -x/(1+x^2) = -dc/pdh
        x = (1 - math.sqrt(max(1 - 4 * r * r, 0.0))) / (2 * r) if abs(r) > 1e-12 else 0.0
        return x * self.lw / 2

    def _hold(self) -> bool:
        """Follow the resonance with the piezo; False if the laser hopped or ran out of range."""
        v, i = self._vi()
        m = self.ecdl.m
        target = self.peak + self._offset_mhz() * 1e-6
        for _ in range(4):
            self.ecdl.lase(v + self.dv, i)
            if self.ecdl.m != m:
                return False
            self.dv += (target - self._free(v + self.dv, i)) * 1e3 / self.ecdl.kv
        self.ecdl.lase(v + self.dv, i)
        return self.ecdl.m == m and abs(self.dv) <= self.range_v

    def _engage(self, near: float | None = None):
        """Lock: to the resonance at `near` THz (a lockpoint) or else nearest the laser."""
        v, i = self._vi()
        if abs(self.dc()) >= 0.5 * self.pdh:  # the error signal never crosses zero: no lock
            self.peak, self.dv = None, 0.0
            return
        f = near if near is not None else self._free(v + self.dv, i)
        peak, _ = self._nearest(f)
        self.peak = peak
        if not self._hold():
            self.peak, self.dv = None, 0.0

    def _lock_enabled(self, on: bool):
        with self.mutex:
            self.peak, self.dv = None, 0.0
            if not on:
                return
            if self.lockpoints:
                if self.lockpoint is None:
                    return  # nothing selected: it does not lock
                v, i = self._vi()
                self._engage(near=self._free(self.lockpoint[0], i))
            else:
                self._engage()

    def _lose_selection(self):
        pass  # the DLC pro keeps the selected lockpoint across scans

    # ------------------------------------------------------- what it shows
    def locked(self) -> bool:
        return bool(self._get("dl:lock:lock-enabled")) and self.peak is not None

    def state(self):
        with self.mutex:
            if self._get("dl:lock:lock-enabled"):
                return (5, "locked") if self.peak is not None else (6, "relocking")
            return (3, "lockpoint selected") if self.lockpoint else (1, "scope")

    def frequency(self, wobble: float = 0.0) -> float:
        """What the wavemeter sees, THz."""
        with self.mutex:
            self.wobble = wobble
            v, i = self._vi()
            if self._get("dl:lock:lock-enabled"):
                if self.peak is not None and not self._hold():
                    self.peak, self.dv = None, 0.0
                    self.relocks += 1
                if self.peak is None and (not self.lockpoints or self.lockpoint is not None):
                    self._engage()  # ReLock: whatever resonance is nearest the laser now
                if self.peak is not None:
                    return self.peak + self._offset_mhz() * 1e-6
            if self._get("scan:enabled"):  # sweeping: the wavemeter catches it anywhere in the scan
                m = self.ecdl.m
                x = v + self._get("scan:amplitude") * (self.rng.random() - 0.5)
                f = self.ecdl.freq(x, i, wobble)
                self.ecdl.m = m
                self.ecdl.lase(v, i)
                return f
            return self.ecdl.freq(v + self.dv, i, wobble)

    def level(self, sig: int) -> float:
        """The present value of an input: Fine In 1 (transmission) or Fast In 3 (error signal)."""
        with self.mutex:
            v, i = self._vi()
            f = self.peak + self._offset_mhz() * 1e-6 if self.locked() else self._free(v + self.dv, i)
            t, e = 0.0, 0.0
            for power, ghz in self.ecdl.components(v + self.dv, i):
                ti, ei = self._airy(f + (ghz - self.ecdl.mode_ghz(v + self.dv, i)) * 1e-3)
                t, e = t + power * ti, e + power * ei
            noise = self.rng.gauss(0.0, 0.002)
            return (t if sig == 0 else e + self.shown_dc()) + noise

    def _master_scan(self):
        """One scan period: [(x, transmission, error)]; finds the lockpoint candidates (tops)."""
        v, i = self._vi()
        amp = self._get("scan:amplitude")
        m0 = self.ecdl.m
        rows = []
        for k in range(self.N):
            t = k / self.N
            x = v + amp * ((2 * t if t < 0.5 else 2 - 2 * t) - 0.5)
            self.ecdl.lase(x, i)
            tr = er = 0.0
            for power, ghz in self.ecdl.components(x, i):
                ti, ei = self._airy(self.ecdl.f0 + ghz * 1e-3 + self.wobble * 1e-6)
                tr, er = tr + power * ti, er + power * ei
            rows.append((x, tr + self.rng.gauss(0.0, 0.002), er + self.shown_dc() + self.rng.gauss(0.0, 0.002)))
        self.ecdl.m = m0
        self.ecdl.lase(v + self.dv, i)
        up = rows[:self.N // 2]
        cands = []
        for k in range(1, len(up) - 1):
            x, t, _ = up[k]
            if t > 0.05 * self.t00 and t > up[k - 1][1] and t >= up[k + 1][1]:
                cands.append((x, t, "t"))
        self.cands = cands
        return rows

    def _scope_rows(self):
        if self._get("scan:enabled") and not self.locked():
            return self._master_scan()
        v, _ = self._vi()
        return [(v + self.dv, self.level(0), self.level(2)) for _ in range(self.N)]

    def _scope_blob(self) -> bytes:
        from .dlcpro import encode_blocks, float_block

        with self.mutex:
            rows = self._scope_rows()
            pick = {0: 1, 100: 1, 2: 2, 34: 0, 50: 0, 101: 0}

            def signal(sig):
                k = pick.get(sig)
                return [r[k] for r in rows] if k is not None else [0.0] * len(rows)

            x = signal(self._get("scope:channelx:xy-signal")) if self._get("scope:variant") == 1 else \
                [50.0 * k / self.N for k in range(self.N)]
            return encode_blocks([float_block("x", x), float_block("y", signal(self._get("scope:channel1:signal"))),
                                  float_block("Y", signal(self._get("scope:channel2:signal")))])

    def _candidates_blob(self) -> bytes:
        from .dlcpro import encode_blocks, points_block

        with self.mutex:
            if self._get("scan:enabled") and not self.locked():
                self._master_scan()
            blocks = [points_block("c", self.cands)]
            if self.lockpoint:
                blocks.append(points_block("l", [self.lockpoint]))
            return encode_blocks(blocks + [("s", bytes([self.state()[0]]))])

    def _select(self, x, y, kind):
        with self.mutex:
            for cx, cy, ct in self.cands:
                if abs(cx - x) <= 0.01 and abs(cy - y) <= 0.05 * max(cy, 0.1) and kind == ord(ct):
                    self.lockpoint = (cx, cy, ct)
                    return ""
            raise CommandError("-7 lockpoint not found")

    # ------------------------------------------------------------------ SHG
    def shg_state(self):
        with self.mutex:
            if not self._get("nlo:shg:lock:lock-enabled"):
                return 0, "off"
            v, i = self._vi()
            if max(p for p, _ in self.ecdl.components(v + self.dv, i)) < 0.6:
                return 6, "relocking"
            return 5, "locked"

    def _intra(self, s: float, comps) -> float:
        """Intra-cavity signal at SHG piezo voltage s, for the master's modes [(power, GHz)]."""
        w = 1.0 / self.shg_finesse
        out = 0.0
        for power, ghz in comps:
            ph = ghz / self.shg_fsr + s / self.shg_vfsr + self.shg_phase
            for off, h in ((0.0, 1.0),) + self.shg_tem:
                u = (ph - off) % 1.0
                u = min(u, 1.0 - u)
                out += power * h / (1 + (2 * u / w) ** 2)
        return self.shg_floor + self.shg_h * out

    def _comps(self):
        v, i = self._vi()
        self.ecdl.lase(v + self.dv, i)
        return self.ecdl.components(v + self.dv, i)

    def _intra_now(self) -> float:
        with self.mutex:
            comps = self._comps()
            if self.shg_state()[0] == 5:
                return self.shg_floor + self.shg_h * max(p for p, _ in comps) + self.rng.gauss(0.0, 0.003)
            s = self._get("nlo:shg:scan:offset")
            if self._get("nlo:shg:scan:enabled"):
                s += self._get("nlo:shg:scan:amplitude") * (self.rng.random() - 0.5)
            return self._intra(s, comps) + self.rng.gauss(0.0, 0.003)

    def _shg_power(self) -> float:
        with self.mutex:
            if self.shg_state()[0] == 5:
                return 25.0 * max(p for p, _ in self._comps()) ** 2
            return 0.1

    def _shg_blob(self) -> bytes:
        from .dlcpro import encode_blocks, float_block

        with self.mutex:
            comps = self._comps()
            s0, amp = self._get("nlo:shg:scan:offset"), self._get("nlo:shg:scan:amplitude")
            scanning = self._get("nlo:shg:scan:enabled") and self.shg_state()[0] != 5
            rows = []
            for k in range(self.N):
                t = k / self.N
                s = s0 + amp * ((2 * t if t < 0.5 else 2 - 2 * t) - 0.5) if scanning else s0
                rows.append((s, self._intra(s, comps) + self.rng.gauss(0.0, 0.003)))
            if self.shg_state()[0] == 5:
                level = self._intra_now()
                rows = [(s0, level + self.rng.gauss(0.0, 0.003)) for _ in range(self.N)]

            def signal(sig):
                if sig == 82:
                    return [r[1] for r in rows]
                if sig in (90, 91):
                    return [r[0] for r in rows]
                return [0.0] * len(rows)

            x = signal(self._get("nlo:shg:scope:channelx:xy-signal")) if self._get("nlo:shg:scope:variant") == 1 \
                else [50.0 * k / self.N for k in range(self.N)]
            ch1, ch2 = self._get("nlo:shg:scope:channel1:signal"), self._get("nlo:shg:scope:channel2:signal")
            return encode_blocks([float_block("x", x), float_block("y", signal(ch1)), float_block("Y", signal(ch2))])

    # ---------------------------------------------------------------- faults
    def jump(self, peaks: float = 1.0):
        """The lock relocks `peaks` FSR away (after a knock, say)."""
        with self.mutex:
            if self.peak is not None:
                self.peak += peaks * self.fsr * 1e-3
                v, i = self._vi()
                if not self._hold():
                    self.peak, self.dv = None, 0.0

    def settle(self, lockpoint: bool = True):
        """Start locked on the resonance at the laser, as if locked by hand (lockpoint selected)."""
        with self.mutex:
            v, i = self._vi()
            f = self._free(v, i)
            peak, h = self._nearest(f)
            if lockpoint and self.lockpoints:
                self.lockpoint = (v + (peak - f) * 1e3 / self.ecdl.kv, h * self.t00, "t")
            self.dlc.params[self.p + "dl:lock:lock-enabled"] = True
            self.peak = peak
            if not self._hold():
                self.peak, self.dv = None, 0.0


class SimLaser:
    """One laser on the wavemeter.

    With a controller: an ECDL tuned by that controller's scan offset and diode current, drifting
    by itself, and optionally locked to a reference cavity by its (simulated) DLC pro lock, with an
    SHG stage (`lock`, a SimLocked, made with lock_it()). cavity=True: the demo's reference laser,
    locked to its cavity by something the program does not see, so a fixed frequency unless a
    fault is injected."""

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
        self.lock = lock          # SimLocked: its DLC pro locks it to a cavity
        self._mutex = threading.Lock()
        if dlc is not None:
            self.ecdl = ECDL(self.f0, kv=tuning, v0=offset0, i0=dlc.current_act(n), seed=seed)
            for name in ("dl:cc:current-set", "scan:offset"):
                dlc.on_set[f"laser{n}:{name}"] = self._settle

    def _settle(self, _value):
        """At every change of its current or offset, as it is made: the laser follows it, hopping on
        the way as it would (else it would only see where a quick ramp ends up)."""
        with self._mutex:
            if self.lock is None or not self.lock.locked():  # (locked, its lock follows it)
                self.ecdl.lase(*self._vi())

    def lock_it(self, **kwargs) -> "SimLocked":
        """Let its DLC pro lock it to a reference cavity with a resonance where it is now; see SimLocked."""
        self.lock = SimLocked(self.dlc, self.n, self.ecdl, self.ecdl.freq(self.offset0, self.dlc.current_act(self.n)),
                              **kwargs)
        return self.lock

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
            v, i = params[p + "scan:offset"], self.dlc.current_act(self.n)
            if self.lock is None:
                return self.ecdl.freq(v, i, wobble)
            return self.lock.frequency(wobble)

    # faults of a laser with a controller
    def hop_mode(self, direction: int = 1):
        """The diode warms or cools a little: the laser hops one mode up (+1) or down (-1)."""
        with self._mutex:
            e = self.ecdl
            lo, hi = e.limits()
            e.nudge(e.misalignment(e.m, *self._vi()) - ((lo - 0.3) if direction > 0 else (hi + 0.3)))

    def go_multimode(self, direction: int = 1):
        """The diode drifts to where a neighbouring mode takes a third of the power, without a hop."""
        with self._mutex:
            e = self.ecdl
            a = e.misalignment(e.m, *self._vi())
            edge, full = e.zone(direction)
            e.nudge(a - direction * (edge + 0.66 * (full - edge)))

    def _vi(self):
        p = f"laser{self.n}:"
        v = self.dlc.params[p + "scan:offset"] + (self.lock.dv if self.lock is not None else 0.0)
        return v, self.dlc.current_act(self.n)

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
    """Three fake controllers: two with two DL pros each, wavemeter-locked; one with a TA-SHG pro
    whose DLC pro locks its master to a reference cavity (lockpoint selected by hand, PDH through
    a FALC pro) and its SHG cavity to the master, and feeds its piezo forward to its current (the
    measured current is 0.42 mA below the set one). Plus a channel without light, and a 729 nm
    laser locked to a cavity: the reference."""

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
            FakeDLCPro("SIM-000789", "Yb+ SHG", {1: {"product": "TA-SHG pro 739", "scan": True,
                                                    "feedforward": -0.006}}),  # measured 0.42 mA below set
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
        e = shg.ecdl  # a master whose modes go multimode well before they hop, quietly for the wavemeter
        e.multimode, e.hysteresis, e.mm_noise = 0.9, 0.4, 0.004
        shg.lock_it(shg=True, lockpoints=True, pdh_dc=0.009)
        shg.lock.settle()
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
