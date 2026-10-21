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

from . import dac as daclib
from .dlcpro import CURRENT_SET, DISCOVERY_PORT, OUT_CC, OUT_PC, PIEZO_SET, SCAN_OFFSET, SCAN_OUT
from .wavemeter import C_NM_THZ, Found, Source


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
    a fixed bias (its config's "act_offset", mA): set and measured current differ, as they do.

    Its scan generator drives the piezo (scan:output-channel 50), or with "scan_out" in its config
    something else: 51 the current. The scan's offset is then that output's set value (the current,
    in mA, within current-clip), and the piezo is set by dl:pc:voltage-set, as by hand; whatever
    else is set (current-set, voltage-set while the scan drives the piezo) is kept but drives
    nothing. Switched to another output, the scan's offset takes that output's value: nothing
    jumps."""

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
            out = int(d.get("scan_out", OUT_PC))
            self.params.update({
                p + "product-name": f'"{d.get("product", "DL pro")}"', p + "type": '"DL pro"',
                p + "label": f'"{d.get("label", "")}"', p + "scan:unit": '"mA"' if out == OUT_CC else '"V"',
                p + "scan:enabled": bool(d.get("scan", False)), p + SCAN_OUT: out,
                p + SCAN_OFFSET: float(d.get("current", 150.0) if out == OUT_CC else d.get("offset", 70.0)),
                p + "scan:amplitude": float(d.get("amplitude", 2.5)), p + "scan:frequency": 20.0,
                p + "emission": True,
                p + PIEZO_SET: float(d.get("offset", 70.0)),
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
        self._drove = {}  # laser -> (piezo, current) when its scan was last switched to another output
        self.on_set = {f"laser{n}:{SCAN_OUT}": lambda out, n=n: self._scan_moved(n) for n in lasers}
        self.commands = {}
        self.busy_checks = []  # (kind 'get' | 'set' | 'exec', name) -> True: refused now as busy (decop -1032)
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

    def drives(self, n: int) -> tuple:
        """(piezo, current): the parameters that set laser n's piezo voltage and current, by what its scan drives."""
        out = self.params[f"laser{n}:{SCAN_OUT}"]
        return (SCAN_OFFSET if out == OUT_PC else PIEZO_SET), (SCAN_OFFSET if out == OUT_CC else CURRENT_SET)

    def offset(self, n: int) -> float:
        """Its piezo voltage (V): the scan's offset, as a rule."""
        return self.params[f"laser{n}:{self.drives(n)[0]}"]

    piezo = offset

    def current(self, n: int) -> float:
        """Its set current (mA)."""
        return self.params[f"laser{n}:{self.drives(n)[1]}"]

    def current_act(self, n: int) -> float:
        """The diode current, as measured: what the laser runs at."""
        p, d = f"laser{n}:", self.params
        i = self.current(n) + d[p + "dl:cc:current-offset"] + self.bias.get(n, 0.0)
        if d[p + "dl:cc:feedforward-enabled"]:
            i += d[p + "dl:cc:feedforward-factor"] * self.offset(n)
        return i

    def _scan_moved(self, n: int):
        """Its scan was switched to another output: its offset takes that output's value, and the one it left keeps its
        own (nothing jumps)."""
        p, d = f"laser{n}:", self.params
        was_v, was_i = self._drove[n]
        out = d[p + SCAN_OUT]
        d[p + PIEZO_SET], d[p + CURRENT_SET] = was_v, was_i
        d[p + SCAN_OFFSET] = was_i if out == OUT_CC else was_v if out == OUT_PC else d[p + SCAN_OFFSET]
        d[p + "scan:unit"] = '"mA"' if out == OUT_CC else '"V"'

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

    def _busy(self, kind: str, name: str) -> bool:
        return any(check(kind, name) for check in self.busy_checks)

    def _execute(self, cmd: str) -> str:
        m = self._REF.match(cmd)
        if m:
            if self._busy("get", m.group(1)):
                return "Error: -1032 data acquisition busy"
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
            if self._busy("set", name):
                return "Error: -1032 data acquisition busy"
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
            head, _, rest = name.partition(":")
            n = int(head[5:]) if head.startswith("laser") and head[5:].isdigit() else None
            if n is not None and f"{head}:{SCAN_OUT}" in self.params:
                if rest == self.drives(n)[1] and new > self.params[f"{head}:dl:cc:current-clip"]:
                    return "Error: -21 above current-clip"
                if rest == SCAN_OUT:  # (where the outputs were, for _scan_moved)
                    self._drove[n] = (self.offset(n), self.current(n))
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
            if self._busy("exec", name):
                return "Error: -1032 data acquisition busy"
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

    With i_stop set, it stops lasing altogether when the current drops below i_stop, and starts
    again only once the current is above i_start (above where it normally runs): setting the
    current back is not enough. Not lasing, it gives the wavemeter nothing to read and a cavity
    nothing to resonate with; the TA gets hardly any seed (power()), or with `ase` that fraction of
    its lasing power as broadband light (an AR-coated chip without its feedback), which its seed
    photodiode cannot tell from lasing.
    """

    def __init__(self, f0, fsr=6.0, kv=0.4, ki=-0.25, ksv=None, ksi=-2.0, kt=-4.0, kst=-1.0,
                 hysteresis=0.6, multimode=0.3, v0=70.0, i0=150.0, seed=None, mm_noise=0.03, skew=0.0,
                 i_stop=None, i_start=None):
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
        self.i_stop, self.i_start = i_stop, i_start  # mA (measured): stops lasing below, starts again above
        self.lasing = True
        self.slope_mw, self.i_th = 0.25, i0 - 80.0   # its power: mW per mA above threshold
        self.ase = 0.0     # not lasing, this fraction of that power still goes on (broadband)

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
        """Settle whether it lases, and on which mode, at (v, i), with hysteresis; returns that
        mode's misalignment (GHz)."""
        start = self.i_start if self.i_start is not None else self.i_stop
        if self.lasing:
            if self.i_stop is not None and i < self.i_stop:
                self.lasing = False
        elif start is None or i > start:
            self.lasing = True  # on whichever mode is best aligned now
            self.m -= round(self.misalignment(self.m, v, i) / self.fsr)
        a = self.misalignment(self.m, v, i)
        lo, hi = self.limits()
        if not lo < a < hi:
            self.m -= round(a / self.fsr)
            a = self.misalignment(self.m, v, i)
        return a

    def mode_ghz(self, v: float, i: float) -> float:
        """Frequency of the lasing mode, GHz from f0 (call lase() first)."""
        return self.m * self.fsr + self.kv * (v - self.v0) + self.ki * (i - self.i0) + self.kt * self.T

    def power(self, i: float) -> float:
        """mW it sends on (to a TA's seed photodiode, say): lasing, above threshold; else a trace, or
        broadband light (ase)."""
        p = max(self.slope_mw * (i - self.i_th), 0.0)
        return p if self.lasing else max(self.ase * p, 0.3)

    def freq(self, v: float, i: float, extra_mhz: float = 0.0) -> float:
        """What the laser emits at (v, i), THz, as a wavemeter would see it (NaN: not lasing)."""
        a = self.lase(v, i)
        if not self.lasing:
            return math.nan
        ghz = self.mode_ghz(v, i) + extra_mhz * 1e-3
        if abs(a) > self.zone(a)[0] and self.mm_noise:  # two modes: noisy, now and then nonsense
            ghz += self.rng.gauss(0.0, self.mm_noise)
            if self.rng.random() < 0.05:
                ghz -= math.copysign(self.rng.uniform(0.2, 0.8) * self.fsr, a)
        return self.f0 + ghz * 1e-3

    def components(self, v: float, i: float):
        """The modes it emits at (v, i): [(power fraction, GHz from f0)] (call lase() first). Shortly
        before it hops off (zone) the neighbouring mode takes up to half the power. None, not lasing."""
        if not self.lasing:
            return []
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

    def stop(self):
        """It stops lasing (the current dipped for a moment, say): it starts again only above i_start."""
        self.lasing = False


class SimTA:
    """The tapered amplifier of a TA pro or TA-SHG pro, seeded by the laser: its seed photodiode
    (amp:pd:seed:power, and amp:seed-limits:power) and its protection: with the seed below
    power-min for longer than power-min-shutdown-delay (seconds here) it switches itself off
    (amp:cc:enabled), to be switched on again by hand. Time comes from `clock`; the protection is
    looked at whenever anything of the TA is read. It only looks at whether the laser lases (as
    whatever drives the laser last settled it): settling it here, in the controller's thread,
    could race a lock that is following the laser."""

    def __init__(self, dlc: "FakeDLCPro", n: int, ecdl: "ECDL", clock=time.monotonic, power_min: float = 8.0,
                 delay: float = 2.0, noise: float = 0.05, seed=None):
        self.dlc, self.n, self.ecdl, self.clock = dlc, n, ecdl, clock
        self.p = p = f"laser{n}:amp:"
        self.noise = noise
        self.rng = random.Random(seed)
        self._low_since = None
        self.trips = 0
        dlc.params.update({
            p + "pd:seed:power": self.seed_power, p + "seed-limits:power": self.seed_power,
            p + "seed-limits:power-min": float(power_min),
            p + "seed-limits:power-min-shutdown-delay": float(delay),
            p + "seed-limits:status": lambda: 0 if self.ok() else 2,
            p + "seed-limits:status-txt": lambda: '"ok"' if self.ok() else '"seed power too low"',
            p + "cc:enabled": True, p + "cc:emission": self.on,
        })

    def seed_power(self) -> float:
        """mW: what the laser sends into the TA."""
        self.watch()
        return max(self.ecdl.power(self.dlc.current_act(self.n)) + self.rng.gauss(0.0, self.noise), 0.0)

    def ok(self) -> bool:
        return self.ecdl.power(self.dlc.current_act(self.n)) >= self.dlc.params[self.p + "seed-limits:power-min"]

    def watch(self):
        """Its protection: switch off once the seed has been too low for too long."""
        if not self.dlc.params[self.p + "cc:enabled"]:
            self._low_since = None
            return
        if self.ok():
            self._low_since = None
            return
        now = self.clock()
        if self._low_since is None:
            self._low_since = now
        elif now - self._low_since > self.dlc.params[self.p + "seed-limits:power-min-shutdown-delay"]:
            self.dlc.params[self.p + "cc:enabled"] = False
            self._low_since = None
            self.trips += 1

    def on(self) -> bool:
        """Amplifying (else nothing reaches the SHG cavity)."""
        self.watch()
        return bool(self.dlc.params[self.p + "cc:enabled"])


class SimLocked:
    """A laser its DLC pro locks to a reference cavity, optionally frequency-doubled in an SHG
    cavity its DLC pro locks to it (a TA-SHG pro): everything the DLC pro shows of it and every
    parameter and command mode-hop recovery uses, registered on a FakeDLCPro.

    Reference cavity: TEM00 resonances every `fsr` GHz (one at f_res), weaker transverse modes
    (`tem`: offset in FSR, relative height) between; transmission on Fine In 1 (t00 V at a TEM00
    peak, on a DC offset `t_dc`; its lock detection, the window, watches it too; its present value
    while the master scans, unlocked, is that somewhere in the scan), the PDH error signal on
    Fast In 3 (amplitude `pdh` V, sign `pdh_sign`, sidebands `sideband` MHz out, their features
    `pdh_sb` of the carrier's, with `pdh_phase` of absorption from a demodulation phase off), with a DC
    offset `pdh_dc` plus the FALC pro's input offset (times `falc_gain`). `carriers`: other lines in
    the light, [(MHz from the laser, power relative to it)], each with peaks and features of its own
    (sidebands of a fiber EOM, say). LAB is a cavity like the one in the lab: lines 0.3 MHz wide,
    sidebands 20 MHz out, two weaker lines 52 MHz either side, all three sampled a sample or so wide by
    a scan of 0.31 V. The lock (lock-enabled):
      lockpoints=True   locks to the resonance at the selected lockpoint (select-lockpoint
                        x y type, which must name one of the candidates the DLC pro found
                        when last asked, find-candidates, in the scan running then; type = the
                        candidate's type character as a number); no lockpoint, no lock
      lockpoints=False  (lock without lockpoint) locks to the resonance nearest the laser
    (lockpoints is lock-without-lockpoint, as written to it).
    and follows it with the piezo while the laser stays on its mode; the error signal's DC level
    puts it off the resonance center (less transmission), and if it is more than half the PDH
    amplitude it cannot lock at all. If the laser hops, the lock relocks to the resonance nearest
    it then: locked, but on another peak. A master running on two modes (the second with more
    than SLIP of the power) cannot hold the lock for long: it loses it every HOLD_S or so, and
    ReLock searches for SEARCH_S before it catches it again (meanwhile the transmission is low,
    the wavemeter sees the laser near where it was). Disabling the lock while scan:enabled is on
    lets the scan run, as a DLC pro does. hop_search: s that ReLock searches after the lock lost the
    laser (it hopped, say) before it catches the resonance nearest it (0: at once). A blip (blip()) is
    the lock losing the laser for a few milliseconds and having it again: only the transmission shows it.
    recorder: the DLC pro's recorder (SimRecorder), recording the transmission.

    SHG: the intra-cavity signal while its cavity is scanned has an Airy peak (plus small
    transverse modes) per FSR for each mode the master emits (ECDL.components); its lock locks to
    the master's main mode when that has most of the power, and holds while the scan is off. The
    master seeds a TA (SimTA, `ta`) on the way: with the TA off, nothing reaches the SHG cavity.
    The master's light for the reference cavity and the wavemeter is picked off before the TA.
    A master that stops lasing loses its cavity lock, and the wavemeter has nothing to read."""

    N = 1000  # samples in a scope trace
    SLIP = 0.2        # a second mode with more of the master's power than this: the lock keeps losing it,
    HOLD_S = 1.0      # holding it about this long (s, on average),
    SEARCH_S = 1.5    # then ReLock searching this long,
    SEARCH_MHZ = 60.0  # sweeping the laser this far either way

    LAB = dict(lw=0.3, t00=0.05, pdh=1.7, pdh_sign=-1, sideband=20.2, pdh_sb=0.76, pdh_phase=-0.25,
               carriers=((-51.6, 0.14), (51.6, 0.18)), t_noise=0.0003, e_noise=0.002)
    LAB_SCAN = 0.31  # V: the scan amplitude, about the laser

    def __init__(self, dlc: "FakeDLCPro", n: int, ecdl: "ECDL", f_res: float, shg: bool = True,
                 lockpoints: bool = True, fsr: float = 1.5, lw: float = 6.0, tem=((0.37, 0.18), (0.71, 0.08)),
                 t00: float = 0.8, pdh: float = 0.4, pdh_dc: float = 0.0, falc: int = 1, range_v: float = 30.0,
                 falc_in_trace: bool = True, falc_sign: int = 1, seed=None, ta: bool | None = None,
                 clock=time.monotonic, t_dc: float = 0.0, hop_search: float = 0.0, recorder: bool = True,
                 sideband: float = 20.0, pdh_sb: float = 0.5, pdh_phase: float = 0.0, pdh_sign: int = 1,
                 carriers=(), falc_gain: float | None = None, t_noise: float = 0.002, e_noise: float = 0.002):
        """falc_in_trace: the error signal on the scope includes the FALC's input offset, which adds
        to it (falc_sign 1) or subtracts (-1), times falc_gain if given (V of error signal per V of offset,
        its sign included). ta: a TA between master and SHG cavity (by default with an SHG stage); clock:
        its time, for its protection."""
        self.dlc, self.n, self.ecdl = dlc, n, ecdl
        self.p = p = f"laser{n}:"
        self.f_res, self.fsr, self.lw, self.tem, self.t00, self.t_dc = f_res, fsr, lw, tuple(tem), t00, t_dc
        self.pdh, self.pdh_dc, self.sideband = pdh, pdh_dc, sideband
        self.pdh_sb, self.pdh_phase, self.pdh_sign = pdh_sb, pdh_phase, pdh_sign
        self.carriers = tuple(carriers)
        self.t_noise, self.e_noise = t_noise, e_noise
        self.has_shg, self.falc, self.range_v = shg, falc, range_v
        self.hop_search = hop_search
        self.falc_in_trace = falc_in_trace
        self.falc_gain = float(falc_sign if falc_gain is None else falc_gain)
        self.rng = random.Random(seed)
        self.mutex = threading.RLock()
        self.wobble = 0.0       # MHz: the free-running laser's drift (from SimLaser)
        self.peak = None        # THz: the resonance the lock holds
        self.dv = 0.0           # V: the lock's piezo correction
        self.lockpoint = None   # (x, y, type)
        self.cands = []         # lockpoint candidates found when last asked (find-candidates)
        self.found = []         # ...and in the last scan
        self.relocks = 0
        self.clock = clock
        self._slip_at = None    # when the lock will lose a master on two modes
        self.blips = []         # (start, end) of the lock's blips, in its clock
        self._rec_rng = random.Random(None if seed is None else seed + 1)  # the recorder's noise
        self._search_until = -math.inf
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
            p + "dl:lock:window:enabled": True, p + "dl:lock:window:input-channel": 0,
            p + "dl:lock:window:level-low": lambda: self.t_dc + 0.2 * self.t00,
            p + "dl:lock:window:level-high": 10.0, p + "dl:lock:window:level-hysteresis": 0.01,
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
        self.recorder = SimRecorder(dlc, n, self.trans_at, clock=clock, seed=seed) if recorder else None
        self.ta = None
        if shg if ta is None else ta:
            self.ta = SimTA(dlc, n, ecdl, clock=clock, seed=seed)
        dlc.on_set[p + "dl:lock:lock-enabled"] = self._lock_enabled
        dlc.on_set[p + "scan:enabled"] = lambda on: self._lose_selection()
        dlc.commands[p + "dl:lock:select-lockpoint"] = self._select
        dlc.commands[p + "dl:lock:find-candidates"] = self._find_candidates

    # ------------------------------------------------------------- helpers
    def _get(self, name):
        return self.dlc.params[self.p + name]

    @property
    def lockpoints(self) -> bool:
        """It locks to a selected lockpoint (else to the resonance nearest the laser)."""
        return not self._get("dl:lock:lock-without-lockpoint")

    def _vi(self):
        return self.dlc.piezo(self.n), self.dlc.current_act(self.n)

    def dc(self) -> float:
        """The error signal's DC level as the FALC sees it: its own offset plus the input offset."""
        off = self.dlc.params.get(f"falc{self.falc}:input:offset", 0.0) if self.falc else 0.0
        return self.pdh_dc + self.falc_gain * off

    def shown_dc(self) -> float:
        """The error signal's DC level as the scope shows it."""
        return self.dc() if self.falc_in_trace else self.pdh_dc

    def _airy(self, f: float):
        """(transmission V, error signal V without DC) of the reference cavity for light at f THz: its line
        and the other lines in it (carriers)."""
        t, e = self._line(f)
        for mhz, power in self.carriers:
            ti, ei = self._line(f + mhz * 1e-6)
            t, e = t + power * ti, e + power * ei
        return t, e

    def _line(self, f: float):
        """(transmission V, error signal V without DC) of the reference cavity for one line at f THz: the PDH
        error signal of a TEM00 resonance, dispersion at the carrier and at the sidebands (with some absorption
        there, if the demodulation phase is off)."""
        d = (f - self.f_res) * 1e3 / self.fsr  # in FSR
        t = e = 0.0
        for off, h in ((0.0, 1.0),) + self.tem:
            u = (d - off) % 1.0
            u = u - 1.0 if u >= 0.5 else u
            x = 2 * u * self.fsr * 1e3 / self.lw  # detuning in half widths
            t += h / (1 + x * x)
            if off == 0.0:
                s = 2 * self.sideband / self.lw
                lo, hi = x + s, x - s
                e += (-x / (1 + x * x) + self.pdh_sb * (hi / (1 + hi * hi) + lo / (1 + lo * lo))
                      + self.pdh_sb * self.pdh_phase * (1 / (1 + hi * hi) - 1 / (1 + lo * lo)))
        return self.t00 * t, self.pdh_sign * self.pdh * e

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
        r = max(min(self.dc() / (self.pdh_sign * self.pdh), 0.5), -0.5)  # -x/(1+x^2) = -dc/pdh
        x = (1 - math.sqrt(max(1 - 4 * r * r, 0.0))) / (2 * r) if abs(r) > 1e-12 else 0.0
        return x * self.lw / 2

    def _hold(self) -> bool:
        """Follow the resonance with the piezo; False if the laser hopped or ran out of range."""
        v, i = self._vi()
        m = self.ecdl.m
        target = self.peak + self._offset_mhz() * 1e-6
        self.ecdl.lase(v + self.dv, i)
        if not self.ecdl.lasing:
            return False
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
        self.ecdl.lase(v + self.dv, i)
        if abs(self.dc()) >= 0.5 * self.pdh or not self.ecdl.lasing:  # no error signal crossing zero: no lock
            self.peak, self.dv = None, 0.0
            return
        f = near if near is not None else self._free(v + self.dv, i)
        peak, _ = self._nearest(f)
        self.peak = peak
        if not self._hold():
            self.peak, self.dv = None, 0.0

    def _slips(self) -> bool:
        """The master runs on two modes, the second with more than SLIP of the power: no lock holds it."""
        v, i = self._vi()
        comps = self.ecdl.components(v + self.dv, i)
        return len(comps) > 1 and min(p for p, _ in comps) > self.SLIP

    def _searching(self) -> bool:
        return self.clock() < self._search_until

    def _tick(self):
        """Locked to a master on two modes: lose it now and then, and search before locking again."""
        if self.peak is None or not self._slips():
            self._slip_at = None
            return
        now = self.clock()
        if self._slip_at is None:
            self._slip_at = now + self.rng.expovariate(1.0 / self.HOLD_S)
        elif now >= self._slip_at:
            self.peak, self.dv, self._slip_at = None, 0.0, None
            self.relocks += 1
            self._search_until = now + self.SEARCH_S

    def _lock_enabled(self, on: bool):
        with self.mutex:
            self.peak, self.dv = None, 0.0
            self._slip_at, self._search_until = None, -math.inf
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
                self._tick()
                return (5, "locked") if self.peak is not None else (6, "relocking")
            return (3, "lockpoint selected") if self.lockpoint else (1, "scope")

    def frequency(self, wobble: float = 0.0) -> float:
        """What the wavemeter sees, THz."""
        with self.mutex:
            self.wobble = wobble
            v, i = self._vi()
            if self._get("dl:lock:lock-enabled"):
                self._tick()
                if self.peak is not None and not self._hold():
                    self.peak, self.dv = None, 0.0
                    self.relocks += 1
                    if self.hop_search > 0:
                        self._search_until = self.clock() + self.hop_search
                if self.peak is None and not self._searching() and (not self.lockpoints or self.lockpoint is not None):
                    self._engage()  # ReLock: whatever resonance is nearest the laser now
                if self.peak is not None:
                    return self.peak + self._offset_mhz() * 1e-6
            if self._get("scan:enabled") and self.ecdl.lasing:  # sweeping: the wavemeter catches it anywhere
                m = self.ecdl.m
                x = v + self._get("scan:amplitude") * (self.rng.random() - 0.5)
                f = self.ecdl.freq(x, i, wobble)
                self.ecdl.m = m
                self.ecdl.lase(v, i)
                return f
            return self.ecdl.freq(v + self.dv, i, wobble)

    def level(self, sig: int) -> float:
        """The present value of an input: Fine In 1 (transmission) or Fast In 3 (error signal); unlocked and
        scanning, somewhere in the scan."""
        with self.mutex:
            v, i = self._vi()
            on = self._get("dl:lock:lock-enabled")
            if on:
                self._tick()
            x, m0 = v + self.dv, self.ecdl.m
            scanning = self._get("scan:enabled") and not on
            if scanning:
                x = v + self._get("scan:amplitude") * (self.rng.random() - 0.5)
            f = self.peak + self._offset_mhz() * 1e-6 if self.locked() else self._free(x, i)
            if on and not self.locked() and self._searching():
                f += self.rng.uniform(-1.0, 1.0) * self.SEARCH_MHZ * 1e-6  # ReLock sweeping it
            t, e = 0.0, 0.0
            for power, ghz in self.ecdl.components(x, i):
                ti, ei = self._airy(f + (ghz - self.ecdl.mode_ghz(x, i)) * 1e-3)
                t, e = t + power * ti, e + power * ei
            if scanning:
                self.ecdl.m = m0
                self.ecdl.lase(v + self.dv, i)
            if self.locked() and self._blipped(self.clock()):
                t = 0.0  # (for a moment, no transmission)
            noise = self.rng.gauss(0.0, self.t_noise if sig == 0 else self.e_noise)
            return (t + self.t_dc if sig == 0 else e + self.shown_dc()) + noise

    def _blipped(self, t: float) -> bool:
        return any(a <= t < b for a, b in self.blips)

    def trans_at(self, times) -> list:
        """The transmission (Fine In 1) at those times (of its clock), for the recorder: as it is now, locked
        (else its DC offset), but where the lock had a blip. Only looked at: nothing is moved along, and its
        noise has a generator of its own (the recorder reads a lot, which must not change the rest)."""
        with self.mutex:
            base = self.t_dc
            if self.locked():
                v, i = self._vi()
                x = v + self.dv
                f = self.peak + self._offset_mhz() * 1e-6
                main = self.ecdl.mode_ghz(x, i)
                base += sum(power * self._airy(f + (ghz - main) * 1e-3)[0]
                            for power, ghz in self.ecdl.components(x, i))
            rng = self._rec_rng
            return [(self.t_dc if self._blipped(t) else base) + rng.gauss(0.0, self.t_noise) for t in times]

    def _master_scan(self):
        """One scan period: [(x, transmission, error)]; self.found: the lockpoint candidates in it (tops)."""
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
            rows.append((x, tr + self.t_dc + self.rng.gauss(0.0, self.t_noise),
                         er + self.shown_dc() + self.rng.gauss(0.0, self.e_noise)))
        self.ecdl.m = m0
        self.ecdl.lase(v + self.dv, i)
        up = rows[:self.N // 2]
        cands = []
        for k in range(1, len(up) - 1):
            x, t, _ = up[k]
            if t > self.t_dc + 0.05 * self.t00 and t > up[k - 1][1] and t >= up[k + 1][1]:
                cands.append((x, t, "t"))
        self.found = cands
        return rows

    def _find_candidates(self):
        """find-candidates: the lockpoint candidates in the scan running now (only then does the DLC pro
        show any)."""
        with self.mutex:
            if self._get("scan:enabled") and not self.locked():
                self._master_scan()
                self.cands = self.found
            return ""

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
            comps = self._comps()
            if not comps or max(p for p, _ in comps) < 0.6:
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
        """The master's modes as they reach the SHG cavity: none with the TA off (or not lasing). Only
        looked at: settling the laser here (a poll of the SHG lock's state, say) would take a hop
        away from the cavity lock, which must see it."""
        v, i = self._vi()
        if self.ta is not None and not self.ta.on():
            return []
        return self.ecdl.components(v + self.dv, i)

    def _intra_now(self) -> float:
        with self.mutex:
            comps = self._comps()
            if self.shg_state()[0] == 5:
                main = max((p for p, _ in comps), default=0.0)
                return self.shg_floor + self.shg_h * main + self.rng.gauss(0.0, 0.003)
            s = self._get("nlo:shg:scan:offset")
            if self._get("nlo:shg:scan:enabled"):
                s += self._get("nlo:shg:scan:amplitude") * (self.rng.random() - 0.5)
            return self._intra(s, comps) + self.rng.gauss(0.0, 0.003)

    def _shg_power(self) -> float:
        with self.mutex:
            if self.shg_state()[0] == 5:
                return 25.0 * max((p for p, _ in self._comps()), default=0.0) ** 2
            return 0.1 if self._comps() else 0.0

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

    def blip(self, seconds: float):
        """The lock loses the laser for `seconds` (milliseconds, say) and has it again at once: only the
        transmission shows it, meanwhile."""
        with self.mutex:
            now = self.clock()
            self.blips = [b for b in self.blips if b[1] > now - 600.0] + [(now, now + seconds)]

    def knock(self, seconds: float = 1.5):
        """The lock loses the laser for a moment (a knock, say): ReLock searches `seconds`, then catches the
        resonance nearest the laser, the one it had."""
        with self.mutex:
            if self.peak is not None:
                self.peak, self.dv = None, 0.0
                self.relocks += 1
                self._search_until = self.clock() + seconds

    def settle(self, lockpoint: bool = True):
        """Start locked on the resonance at the laser, as if locked by hand (lockpoint selected)."""
        with self.mutex:
            v, i = self._vi()
            f = self._free(v, i)
            peak, h = self._nearest(f)
            if lockpoint and self.lockpoints:
                self.lockpoint = (v + (peak - f) * 1e3 / self.ecdl.kv, self.t_dc + h * self.t00, "t")
            self.dlc.params[self.p + "dl:lock:lock-enabled"] = True
            self.peak = peak
            if not self._hold():
                self.peak, self.dv = None, 0.0


class SimRecorder:
    """A DLC pro's recorder (laserN:recorder) on a FakeDLCPro: channel 1 recorded every recording-time /
    sample-count-set s once it is enabled, until it has sample-count-set samples (then it disables itself;
    the data stays until the next recording). Samples are made when they are read (get-data, binary blocks
    'x' the time in s and 'y' channel 1): level_at(times), what the input showed then, as far as the
    simulation knows it. While it records, the DLC pro refuses the scope's data, its lock going on and finding
    lock points as busy, now and then (conflicts): its data acquisition is shared."""

    LIMIT = 1 << 20
    ACQUIRES = (("get", "scope:data"), ("set", "dl:lock:lock-enabled"), ("exec", "dl:lock:find-candidates"))

    def __init__(self, dlc: "FakeDLCPro", n: int, level_at, clock=time.monotonic, seed=None):
        self.dlc, self.level_at, self.clock = dlc, level_at, clock
        self.p = p = f"laser{n}:recorder:"
        self.t0 = None     # when the recording going on began
        self.t_data = 0.0  # ...the one whose data there is
        self.done = 0      # samples of a recording that has stopped
        self.reads = 0     # get-data calls
        self.busy = False  # get-data refused while it records (as a real one might)
        self.conflicts = 0.0  # while it records, the scope's data, its laser's lock going on and finding lock points
        #                       are refused as busy this often (the DLC pro's data acquisition is shared; the lab's
        #                       refused now and then: decop -1032 "data acquisition busy")
        self.refused = 0   # ...how many times
        self._acquires = {(kind, f"laser{n}:{name}") for kind, name in self.ACQUIRES}
        self._rng = random.Random(seed)
        dlc.busy_checks.append(self._conflict)
        d = dlc.params
        d.update({
            p + "enabled": False, p + "state": lambda: self._state()[0],
            p + "state-txt": lambda: f'"{self._state()[1]}"', p + "trigger-mode": 0, p + "recording-mode": 0,
            p + "recording-time": 10.0, p + "sample-count-set": 10000, p + "sample-count-limit": self.LIMIT,
            p + "sample-count": lambda: d[p + "sample-count-set"], p + "sampling-interval": lambda: self.dt(),
            p + "sampling-rate": lambda: 1.0 / self.dt(), p + "memory-size": self.LIMIT,
            p + "inputs:channel1:signal": 0, p + "inputs:channel2:signal": -3, p + "inputs:channelx:signal": -2,
            p + "inputs:channel1:low-pass-filter:enabled": True,
            p + "inputs:channel1:low-pass-filter:cut-off-frequency": 1000.0,
            p + "data:recorded-sample-count": lambda: self.count(),
            p + "data:last-recorded-sample": lambda: self.count() - 1,
            p + "data:recorded-sampling-interval": lambda: self.dt(),
        })
        dlc.on_set[p + "enabled"] = self._enabled
        dlc.commands[p + "data:get-data"] = self._get_data

    def dt(self) -> float:
        d = self.dlc.params
        return d[self.p + "recording-time"] / max(d[self.p + "sample-count-set"], 1)

    def _enabled(self, on: bool):
        if on:
            self.t0 = self.t_data = self.clock()
            self.done = 0
        elif self.t0 is not None:
            self.done, self.t0 = self.count(), None

    def count(self) -> int:
        size = self.dlc.params[self.p + "sample-count-set"]
        if self.t0 is None:
            return self.done
        n = min(int((self.clock() - self.t0) / self.dt()), size)
        if n >= size:  # full: it stops
            self.done, self.t0 = size, None
            self.dlc.params[self.p + "enabled"] = False
        return n

    def _state(self):
        if self.t0 is not None:
            return 1, "recording"
        return (2, "finished") if self.done else (0, "idle")

    def recording(self) -> bool:
        return self.t0 is not None and self.count() < self.dlc.params[self.p + "sample-count-set"]

    def _conflict(self, kind: str, name: str) -> bool:
        """Refuse that now, as busy? (conflicts)"""
        if not self.conflicts or (kind, name) not in self._acquires or not self.recording():
            return False
        hit = self._rng.random() < self.conflicts
        self.refused += hit
        return hit

    def _get_data(self, start, count):
        from .dlcpro import encode_blocks, float_block

        start, count = int(start), int(count)
        n = self.count()
        if self.busy and self.t0 is not None:
            raise CommandError("-11 busy recording")
        if start < 0 or count <= 0 or start >= max(n, 1):
            raise CommandError("-3 no such samples")
        ks = range(start, min(start + count, n))
        dt = self.dt()
        times = [self.t_data + k * dt for k in ks]
        self.reads += 1
        blob = encode_blocks([float_block("x", [k * dt for k in ks]), float_block("y", self.level_at(times))])
        return "&" + base64.b64encode(blob).decode()


class SimLaser:
    """One laser on the wavemeter.

    With a controller: an ECDL tuned by that controller's scan offset and diode current, drifting
    by itself, and optionally locked to a reference cavity by its (simulated) DLC pro lock, with an
    SHG stage (`lock`, a SimLocked, made with lock_it()). cavity=True: the demo's reference laser,
    locked to its cavity by something the program does not see, so a fixed frequency unless a
    fault is injected."""

    def __init__(self, ch, wavelength_nm, tuning=0.4, dlc=None, n=1, offset0=70.0,
                 drift=3.0, signal=True, cavity=False, lock=None, seed=None, dac=None):
        """dac: (a dac.SimBoard, its output, the output fed forward or None, GHz per V of that): a laser without a
        digital controller, its piezo driver (tuning: GHz per V at the output, about offset0) and its current fed
        by the board's outputs."""
        self.ch, self.f0 = ch, C_NM_THZ / wavelength_nm
        self.dac = dac
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
            for name in (CURRENT_SET, SCAN_OFFSET, PIEZO_SET):
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
            if self.dac is not None:  # its piezo driver and current fed by an analog output board
                board, ao, ff, ff_ghz = self.dac
                ghz = self.tuning * (board.volts[ao] - self.offset0) + (ff_ghz * board.volts[ff] if ff is not None
                                                                        else 0.0)
                return self.f0 + ghz * 1e-3 + wobble * 1e-6
            if self.ecdl is None:
                return self.f0 + wobble * 1e-6
            v, i = self.dlc.piezo(self.n), self.dlc.current_act(self.n)
            f = self.ecdl.freq(v, i, wobble) if self.lock is None else self.lock.frequency(wobble)
            return f if math.isfinite(f) else None  # not lasing

    # faults of a laser with a controller
    def hop_mode(self, direction: int = 1):
        """The diode warms or cools a little: the laser hops one mode up (+1) or down (-1)."""
        with self._mutex:
            e = self.ecdl
            lo, hi = e.limits()
            e.nudge(e.misalignment(e.m, *self._vi()) - ((lo - 0.3) if direction > 0 else (hi + 0.3)))

    def stop_lasing(self, ase: float | None = None):
        """The current dips for a moment and the laser stops lasing; at its current it does not start
        again by itself (if it has a lasing threshold, ECDL.i_stop and i_start). ase: from now on, not
        lasing it still sends that fraction of its power to the TA (ECDL.ase)."""
        with self._mutex:
            if ase is not None:
                self.ecdl.ase = ase
            self.ecdl.stop()
            if self.lock is not None and self.lock.locked():
                self.lock.peak, self.lock.dv = None, 0.0

    def go_multimode(self, direction: int = 1):
        """The diode drifts to where a neighbouring mode takes a third of the power, without a hop (its
        cavity lock, if any, then keeps losing it: SimLocked.SLIP)."""
        with self._mutex:
            e = self.ecdl
            a = e.misalignment(e.m, *self._vi())
            edge, full = e.zone(direction)
            e.nudge(a - direction * (edge + 0.66 * (full - edge)))

    def _vi(self):
        v = self.dlc.piezo(self.n) + (self.lock.dv if self.lock is not None else 0.0)
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
    """Cycles through the lasers like a fibre switch, one exposure per channel, skipping those whose 'Use' is
    unticked (use: switch() ticks and unticks it as this program would, untick() and tick() as someone at the
    WLM software would). A channel ticked again shows its old value until it is measured anew, as the WLM's
    does."""

    label = "simulation"
    model = "simulated WS7"
    switchable = True

    def __init__(self, sink, lasers, exposure=0.02, wavemeter=None, serial=None, model=None, demo=None):
        """serial, model: the simulated wavemeter's (serial None: it does not say); demo: where select() finds
        another one's lasers (Demo.meter)."""
        super().__init__(sink, exposure)
        self.lasers = lasers
        self.wavemeter = wavemeter or SimWavemeter()
        self.use = {laser.ch: True for laser in lasers}
        self.switched = []   # (ch, on): what switch() did, in order
        self._shown = {}     # ch -> the value it shows (kept while it is not measured)
        self._again = set()  # ticked again: shows its old value once more, then is measured
        self.wanted, self._model, self.demo = serial, model or type(self).model, demo
        self._selects = []   # another simulated wavemeter to read (select), on this thread

    def select(self, serial):
        """Read that simulated wavemeter (of the demo's) from now on, in a moment."""
        self._selects.append(serial)

    def _select_now(self):
        serial = self._selects.pop()
        self._selects.clear()
        found = self.demo.meter(serial) if self.demo is not None else None
        if found is None or serial == self.wanted:
            return
        self.wanted, self._model, self.lasers, self.wavemeter = serial, *found
        self.use = {laser.ch: True for laser in self.lasers}
        self._shown, self._again = {}, set()
        self._last, self._stale, self._active, self._off = {}, {}, None, None
        self._set_meter(self._model, serial)

    def cavity_laser(self):
        return next((laser for laser in self.lasers if laser.cavity), None)

    def laser(self, ch: int):
        return next((laser for laser in self.lasers if laser.ch == ch), None)

    def untick(self, ch: int):
        """Someone unticks the channel's 'Use' in the WLM software (for a faster rate on theirs, say)."""
        self.use[ch] = False

    def tick(self, ch: int):
        self._use(ch, True, record=False)

    def _use(self, ch: int, on: bool, record: bool = True) -> str:
        if ch not in self.use:
            return "channel not available"
        if on and not self.use[ch]:
            self._again.add(ch)
        self.use[ch] = on
        if record:
            self.switched.append((ch, on))
        return ""

    def _channels(self):
        used = [laser for laser in self.lasers if self.use.get(laser.ch, True)]
        self._set_active([laser.ch for laser in used], [laser.ch for laser in self.lasers if laser not in used])
        return used

    def run(self):
        if self.wanted:
            self._set_meter(self._model, self.wanted)
        self._channels()
        self._set_state(True)
        i = 0
        while not self._stop_ev.wait(self.interval):
            if self._selects:
                self._select_now()
            self._do_switches(self._use)
            used = self._channels()
            if not used:
                continue
            laser = used[i % len(used)]
            i += 1
            now = time.monotonic()
            if laser.ch in self._again:
                self._again.discard(laser.ch)
                if laser.ch in self._shown:  # its old value, once more
                    self._push({laser.ch: self._shown[laser.ch]}, now)
                    continue
            f = laser.true_freq(now)
            value = self._shown[laser.ch] = -1.0 if f is None else self.wavemeter.read(f, now)  # -1: no signal
            self._push({laser.ch: value}, now)


class Demo:
    """Three fake controllers: two with two DL pros each, wavemeter-locked (the 423 nm one's scan generator drives its
    current, as on the lab's second TA-SHG pro: its piezo is set by dl:pc:voltage-set); one with a TA-SHG pro
    whose DLC pro locks its master to a reference cavity (lockpoint selected by hand, PDH through
    a FALC pro) and its SHG cavity to the master, and feeds its piezo forward to its current (the
    measured current is 0.42 mA below the set one); its master stops lasing 1.3 mA below its
    current, its TA switches itself off after 5 s without seed, and its lock searches a moment after
    a hop before it catches a peak again. Plus a channel without light, and a 729 nm laser locked to
    a cavity: the reference."""

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
    DAC = 8  # with dac=True: a laser without a digital controller, on a simulated analog output board
    DAC_LASER = (8, 935.17976, 8.0, 2, 3, 0.4, "935 (DAC)")  # ch, nm, GHz/V, output, feed-forward output, GHz/V, name
    SECOND = [  # meters=2: a second wavemeter on the same PC, with lasers of its own (controller 3: a fourth DLC pro)
        (1, 369.524540, +0.30, 3, 1, "369 Yb"),
        (2, 398.910750, +0.30, 3, 2, "399 Yb"),
        (3, 638.610400, +0.40, None, 0, "638"),
    ]
    MODELS = ("simulated WS7", "simulated WS6")

    def __init__(self, discovery_port=DISCOVERY_PORT, wavemeter=None, exposure=0.02, dac=False, meters=1,
                 serials=None):
        """meters: 2, a second simulated wavemeter on this PC (SECOND); serials: the simulated wavemeters' serial
        numbers (else 1001, 1002)."""
        self.dlcs = [
            FakeDLCPro("SIM-000123", "Ca+ blue", {1: {"product": "DL pro 397"}, 2: {"product": "DL pro 866"}}),
            FakeDLCPro("SIM-000456", "Ca+ red", {1: {"product": "DL pro 854"},
                                                 2: {"product": "DL pro 423", "scan_out": OUT_CC}}),
            FakeDLCPro("SIM-000789", "Yb+ SHG", {1: {"product": "TA-SHG pro 739", "scan": True,
                                                    "feedforward": -0.006}}),  # measured 0.42 mA below set
        ]
        if meters > 1:
            self.dlcs.append(FakeDLCPro("SIM-000999", "Yb+ cooling", {1: {"product": "DL pro 369"},
                                                                      2: {"product": "DL pro 399"}}))
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
        e = shg.ecdl  # a master whose modes go multimode well before they hop, quietly for the wavemeter,
        e.multimode, e.hysteresis, e.mm_noise = 0.9, 0.4, 0.004  # and that stops lasing 1.3 mA below its
        e.i_stop, e.i_start = e.i0 - 1.3, e.i0 + 2.4  # current, starting again only 2.4 mA above it
        shg.lock_it(shg=True, lockpoints=True, pdh_dc=0.009, hop_search=SimLocked.SEARCH_S)
        self.dlcs[2].params["laser1:amp:seed-limits:power-min-shutdown-delay"] = 5.0
        shg.lock.settle()
        self.wavemeter = wavemeter or SimWavemeter()
        self.exposure = exposure
        self.names = {ch: name for ch, *_, name in self.CHANNELS}
        self.targets = ["127.0.0.1"]  # never broadcast on the real LAN in demo mode
        self.extra_devices = [] if self.responder else [d.key for d in self.dlcs]
        self.board = None
        if dac:  # an old laser: its piezo driver on output 2 of a simulated board, its current fed forward on 3
            number = max(daclib.SIM_BOARDS, default=-1) + 1
            self.board = daclib.SIM_BOARDS[number] = daclib.SimBoard(number)
            self.dac_key = daclib.key("sim", number)
            ch, nm, ghz, ao, ff, ff_ghz, name = self.DAC_LASER
            self.lasers.append(SimLaser(ch, nm, ghz, offset0=0.3, drift=3.0, dac=(self.board, ao, ff, ff_ghz)))
            self.names[ch] = name
        self.serials = list(serials or [1001 + k for k in range(meters)])[:meters]
        self.meters = {self.serials[0]: (self.MODELS[0], self.lasers, self.wavemeter)}
        self.second_names = {}
        if meters > 1:  # (its own drift: another wavemeter)
            lasers = [SimLaser(ch, wl, k, self.dlcs[c] if c is not None else None, n, drift=3.0)
                      for ch, wl, k, c, n, _ in self.SECOND]
            self.meters[self.serials[1]] = (self.MODELS[1], lasers, SimWavemeter())
            self.second_names = {ch: name for ch, *_, name in self.SECOND}

    def make_source(self, sink, serial=None, model="", interval=None, channels=None):
        """A source for the sink: of the first simulated wavemeter, or of the one with that serial number."""
        serial = serial if serial in self.meters else self.serials[0]
        model, lasers, wavemeter = self.meters[serial]
        return SimSource(sink, lasers, self.exposure, wavemeter, serial=serial, model=model, demo=self)

    def meter(self, serial):
        """A simulated wavemeter's (model, lasers, its error), or None."""
        return self.meters.get(serial)

    def scan(self) -> list:
        """The simulated wavemeters of this simulated PC (as WlmHub.scan)."""
        return [Found(k, self.meters[s][0], s) for k, s in enumerate(self.serials)]

    @staticmethod
    def free(serial) -> bool:
        return True

    @staticmethod
    def swap(a, b):
        wa, wb = a.wanted, b.wanted
        a.select(wb)
        b.select(wa)

    def laser(self, ch: int) -> SimLaser:
        return next(laser for laser in self.lasers if laser.ch == ch)

    def wavelength(self, ch: int) -> float:
        if self.board is not None and ch == self.DAC:
            return self.DAC_LASER[1]
        return next(wl for c, wl, *_ in self.CHANNELS if c == ch)

    def close(self):
        if self.responder:
            self.responder.close()
        for d in self.dlcs:
            d.close()
        if self.board is not None:
            daclib.SIM_BOARDS.pop(self.board.number, None)
