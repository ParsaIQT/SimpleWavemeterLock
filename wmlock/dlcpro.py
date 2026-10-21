"""Minimal TOPTICA DLC pro client and LAN discovery (no dependencies).

Protocol, as implemented by TOPTICA's own SDK (toptica.lasersdk):
  * command line on TCP 1998; every reply, the welcome banner included, ends
    with the prompt '\\n> '
  * (param-ref 'laser1:scan:offset)        -> '70.5', '"text"', '#t', '(1.5 0.2)',
                                              '&<base64>' (binary) or 'Error: ...'
  * (param-set! 'laser1:scan:offset 70.5)  -> status: 0 ok, >0 warning, <0 error
  * (exec 'laser1:dl:lock:select-lockpoint 70.1 0.8 116)
                                           -> output lines, the last one the return
                                              value if any; 'Error: ...' on failure
  * discovery: UDP 'whoareyou?' to port 60010 on each subnet broadcast address;
    every device answers ("sn" "..." "..." "..." "..." "label" n "ip" cmd_port mon_port)
  * binary data (scope traces, lock candidates) comes in blocks: an id letter, the
    payload length in ASCII, a zero byte, the payload. Scope: 'x' the x axis, 'y'
    channel 1, 'Y' channel 2 (float32, little endian). Lock: 'c' candidates, 'l' the
    selected lockpoint, 't' the tracked position (x, y float32 and a type char each),
    's' the lock state (one byte: 3 selected, 5 locked).
"""
from __future__ import annotations

import base64
import binascii
import ipaddress
import re
import socket
import struct
import threading
import time
from array import array
from dataclasses import dataclass, field
from sys import byteorder

COMMAND_PORT = 1998
DISCOVERY_PORT = 60010
_PROMPT = b"\n> "
_REPLY = re.compile(r'\("(.*?)" "(.*?)" "(.*?)" "(.*?)" "(.*?)" "(.*?)" (\d+) "(.*?)" (\d+) (\d+)\)')


class DecopError(Exception):
    """The device answered with an error."""


def encode(value) -> str:
    if isinstance(value, bool):
        return "#t" if value else "#f"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.6f}"
    if isinstance(value, str):
        return '"' + value.replace('"', "") + '"'
    raise TypeError(f"cannot encode {type(value).__name__}")


class DLCPro:
    """Blocking DeCoP command-line client. Reconnects on the next command after an error."""

    def __init__(self, host: str, port: int = COMMAND_PORT, timeout: float = 2.0):
        self.host, self.port, self.timeout = host, port, timeout
        self._sock = None
        self._buf = b""
        self._lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def open(self):
        self.close()
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock, self._buf = s, b""
        try:
            self._read_reply()  # welcome banner
        except BaseException:
            self.close()
            raise

    def close(self):
        s, self._sock = self._sock, None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass

    def _read_reply(self) -> str:
        while True:
            i = self._buf.find(_PROMPT)
            if i >= 0:
                data, self._buf = self._buf[:i], self._buf[i + len(_PROMPT):]
                return data.decode("utf-8", "replace").rstrip("\r")
            if len(self._buf) > 1 << 20:
                raise ConnectionError("reply too long")
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("connection closed by device")
            self._buf += chunk

    def command(self, cmd: str) -> str:
        with self._lock:
            if self._sock is None:
                self.open()
            try:
                self._sock.sendall(cmd.encode() + b"\n")
                reply = self._read_reply()
            except BaseException:  # timeout or reset: the stream is out of sync now
                self.close()
                raise
        lines = reply.splitlines()
        if lines and lines[0].strip() == cmd:  # 'echo' is enabled on the device
            lines = lines[1:]
        return "\n".join(lines).strip()

    def get(self, param: str) -> str:
        r = self.command(f"(param-ref '{param})")
        if r[:5].lower() == "error":
            raise DecopError(f"{param}: {r}")
        return r

    def get_float(self, param: str) -> float:
        r = self.get(param)
        try:
            return float(r)
        except ValueError:
            raise DecopError(f"{param}: unexpected reply {r!r}") from None

    def get_str(self, param: str) -> str:
        r = self.get(param)
        return r[1:-1] if len(r) >= 2 and r[0] == r[-1] == '"' else r

    def get_bool(self, param: str) -> bool:
        r = self.get(param)
        if r not in ("#t", "#f"):
            raise DecopError(f"{param}: unexpected reply {r!r}")
        return r == "#t"

    def get_int(self, param: str) -> int:
        r = self.get(param)
        try:
            return int(r)
        except ValueError:
            raise DecopError(f"{param}: unexpected reply {r!r}") from None

    def get_bytes(self, param: str) -> bytes:
        """A binary parameter (scope data, lock candidates)."""
        r = self.get(param)
        text = r[1:] if r.startswith("&") else r[1:-1] if len(r) >= 2 and r[0] == r[-1] == '"' else None
        try:
            if text is None:
                raise ValueError
            return base64.b64decode(text, validate=True)
        except (ValueError, binascii.Error):
            raise DecopError(f"{param}: not binary data {r[:40]!r}") from None

    def get_tuple(self, param: str) -> tuple:
        """A tuple such as a lockpoint's position, '(70.1 0.82)'."""
        r = self.get(param)
        try:
            value = decode(r)
        except ValueError:
            value = None
        if not isinstance(value, tuple):
            raise DecopError(f"{param}: unexpected reply {r!r}")
        return value

    def get_value(self, param: str):
        """Any parameter, its type inferred from the reply; binary parameters as bytes."""
        if param.endswith(BINARY):
            return self.get_bytes(param)
        r = self.get(param)
        try:
            return decode(r)
        except ValueError:
            raise DecopError(f"{param}: unexpected reply {r!r}") from None

    def set(self, param: str, value) -> int:
        r = self.command(f"(param-set! '{param} {encode(value)})")
        last = r.splitlines()[-1].strip() if r else ""
        try:
            status = int(last)
        except ValueError:
            raise DecopError(f"set {param}: {r or 'no reply'}") from None
        if status < 0:
            raise DecopError(f"set {param} = {value}: error {status}")
        return status

    def exec(self, name: str, *args) -> str:
        """Run a command; returns what it printed (its return value on the last line)."""
        cmd = f"(exec '{name}" + "".join(" " + encode(a) for a in args) + ")"
        r = self.command(cmd)
        lines = r.splitlines()
        if lines and (lines[0].lower().startswith("error") or lines[-1].lower().startswith("error")):
            bad = lines[0] if lines[0].lower().startswith("error") else lines[-1]
            raise DecopError(f"{name}: {bad.strip()}")
        return r

    def exec_bytes(self, name: str, *args) -> bytes:
        """Run a command that returns binary data (the recorder's get-data): its last line, base64."""
        r = self.exec(name, *args)
        last = r.splitlines()[-1].strip() if r else ""
        quoted = len(last) >= 2 and last[0] == last[-1] == '"'
        text = last[1:] if last.startswith("&") else last[1:-1] if quoted else None
        try:
            if text is None:
                raise ValueError
            return base64.b64decode(text, validate=True)
        except (ValueError, binascii.Error):
            raise DecopError(f"{name}: not binary data {last[:40]!r}") from None


BINARY = (":data", ":candidates", ":background-trace")  # parameters that are binary data


def decode(text: str):
    """A reply to (param-ref ...), its type inferred: bool, int, float, str, bytes or a tuple."""
    text = text.strip()
    if text.lower().startswith("error"):
        raise DecopError(text)
    if text in ("#t", "#T", "#f", "#F"):
        return text in ("#t", "#T")
    if text.startswith("&"):
        try:
            return base64.b64decode(text[1:], validate=True)
        except binascii.Error:
            raise ValueError(text) from None
    if len(text) >= 2 and text[0] == text[-1] == '"':
        return text[1:-1]
    if text.startswith("'("):
        text = text[1:]
    if text.startswith("(") and text.endswith(")"):
        return tuple(decode(t) for t in text[1:-1].split())
    try:
        return int(text)
    except ValueError:
        return float(text)  # ValueError if it is none of these


# ------------------------------------------------------------- binary data
def data_blocks(data: bytes) -> list:
    """[(id, payload)] of the DLC pro's 'Scope, Lock, and Recorder Binary Data' format."""
    out, i = [], 0
    while i < len(data):
        bid = chr(data[i])
        try:
            end = data.index(0, i + 1)
            n = int(data[i + 1:end])
        except ValueError:
            raise DecopError(f"binary data: bad header of block {bid!r}") from None
        if end + 1 + n > len(data):
            raise DecopError(f"binary data: block {bid!r} is cut short")
        out.append((bid, bytes(data[end + 1:end + 1 + n])))
        i = end + 1 + n
    return out


def float_arrays(data: bytes) -> dict:
    """Scope data: {'x': [...], 'y': [...], 'Y': [...]} (whichever blocks are there)."""
    out = {}
    for bid, payload in data_blocks(data):
        if bid in "aAbBxyY":
            if len(payload) % 4:
                raise DecopError(f"binary data: block {bid!r} is not float32")
            values = array("f")
            values.frombytes(payload)
            if byteorder != "little":
                values.byteswap()
            out[bid] = values.tolist()
    return out


_POINT = struct.Struct("<2fc")


def lock_points(data: bytes) -> dict:
    """Lock data: {'c': [(x, y, type char)], 'l': [...], 't': [...]} and 's': state (int)."""
    out = {}
    for bid, payload in data_blocks(data):
        if bid in "clt":
            if len(payload) % _POINT.size:
                raise DecopError(f"binary data: block {bid!r} is not a list of points")
            out[bid] = [(x, y, t.decode("latin-1")) for x, y, t in _POINT.iter_unpack(payload)]
        elif bid == "s" and len(payload) == 1:
            out["s"] = payload[0]
    return out


def encode_blocks(blocks) -> bytes:
    """The inverse of data_blocks, for the simulation: blocks is [(id, payload bytes)]."""
    return b"".join(bid.encode() + str(len(p)).encode() + b"\0" + p for bid, p in blocks)


def float_block(bid: str, values) -> tuple:
    a = array("f", values)
    if byteorder != "little":
        a.byteswap()
    return bid, a.tobytes()


def points_block(bid: str, points) -> tuple:
    return bid, b"".join(_POINT.pack(x, y, t.encode("latin-1")) for x, y, t in points)


# DLC pro signal ids (scope channels, lock inputs), from TOPTICA's SDK
SIGNALS = {
    -3: "none", -2: "time", -1: "frequency",
    0: "Fine In 1", 1: "Fine In 2", 2: "Fast In 3", 3: "Fast In 3B", 4: "Fast In 4", 5: "Fast In 4B",
    20: "Output A", 21: "Output B", 30: "Lock-in output", 31: "PID 1 output", 32: "PID 2 output",
    34: "Scan output", 35: "Scan aux output",
    40: "PDH 1 error 1", 41: "PDH 1 in 1", 42: "PDH 1 error 2", 43: "PDH 1 in 2",
    44: "PDH 2 error 1", 45: "PDH 2 in 1", 46: "PDH 2 error 2", 47: "PDH 2 in 2",
    50: "Piezo voltage", 51: "Current", 54: "DL photodiode",
    80: "SHG cavity error signal", 81: "SHG cavity rejection signal", 82: "SHG intra-cavity signal",
    83: "SHG power", 87: "SHG input power", 90: "SHG cavity piezo (slow)", 91: "SHG cavity piezo (fast)",
    100: "Lock input", 101: "Scan output (alias)",
}
FINE_IN1, SCAN_OUTPUT, PIEZO, SHG_INTRA, LOCK_INPUT, SCAN_ALIAS, NONE = 0, 34, 50, 82, 100, 101, -3
IO_VALUE = {0: "io:fine-1:value-act", 1: "io:fine-2:value-act", 2: "io:fast-3:value-act",
            4: "io:fast-4:value-act"}  # the present value of an input, by signal id


def signal_name(sig) -> str:
    return SIGNALS.get(sig, f"signal {sig}") if isinstance(sig, int) else str(sig)


# What a laser's scan generator drives (laserN:scan:output-channel), in TOPTICA's numbers (its FAQ): the scan's offset
# is the DC level of that output.
SCAN_OUT = "scan:output-channel"
SCAN_OFFSET = "scan:offset"
PIEZO_SET = "dl:pc:voltage-set"
CURRENT_SET = "dl:cc:current-set"
OUT_PC, OUT_CC = 50, 51
OUTPUTS = {20: "Out A", 21: "Out B", OUT_PC: "the piezo", OUT_CC: "the current", 90: "the SHG cavity's piezo",
           101: "the SC output"}


def drives(out) -> tuple:
    """(piezo, current): the parameters, below laserN:, that set its piezo voltage and its diode current, by what its
    scan generator drives (out: scan:output-channel; None: not known, taken to be the piezo, as on most): the scan's
    offset sets that one, its set value the other (a TA-SHG pro's scan on its current: 160 mA there, its piezo at
    41.3 V on voltage-set)."""
    if out == OUT_CC:
        return PIEZO_SET, SCAN_OFFSET
    if out is None or out == OUT_PC:
        return SCAN_OFFSET, CURRENT_SET
    return PIEZO_SET, CURRENT_SET


def output_name(out) -> str:
    return OUTPUTS.get(out, f"output channel {out}") if out is not None else "an output it does not say"


def scan_output(client, laser: int):
    """What laserN's scan generator drives (scan:output-channel), or None if the controller does not say."""
    try:
        out = client.get_value(f"laser{laser}:{SCAN_OUT}")
    except DecopError:
        return None
    return int(out) if isinstance(out, (int, float)) and not isinstance(out, bool) else None


# --------------------------------------------------------------------- discovery
@dataclass
class Device:
    ip: str
    port: int = COMMAND_PORT
    ids: tuple = ()          # names from the discovery reply (serial number, system label)
    system_type: str = ""
    serial: str = ""
    label: str = ""
    lasers: dict = field(default_factory=dict)  # n -> description, for detected laserN
    error: str = ""          # last probe error, '' if reachable
    probed: bool = False

    @property
    def key(self) -> str:
        return self.ip if self.port == COMMAND_PORT else f"{self.ip}:{self.port}"

    @classmethod
    def from_key(cls, key: str) -> "Device":
        host, _, port = key.strip().partition(":")
        return cls(host, int(port) if port else COMMAND_PORT)

    @property
    def name(self) -> str:
        """Short human name: system label, else serial number, else address."""
        ids = [i for i in self.ids if i]
        return self.label or self.serial or (ids[-1] if ids else self.key)

    def title(self) -> str:
        if self.error:
            return f"{self.key}  (unreachable)"
        parts = [self.system_type or "DLC pro"]
        if self.label:
            parts.append(f"“{self.label}”")
        if self.serial:
            parts.append(self.serial)
        parts.append(self.key)
        return "  ·  ".join(parts)


def broadcast_addresses() -> list:
    """Directed broadcast address of every IPv4 subnet we are on, plus 255.255.255.255."""
    out = []
    try:
        import ifaddr  # optional; without it only the limited broadcast is used

        for adapter in ifaddr.get_adapters():
            for ip in adapter.ips:
                if not isinstance(ip.ip, str):  # IPv6 entries are tuples
                    continue
                net = ipaddress.IPv4Network(f"{ip.ip}/{ip.network_prefix}", strict=False)
                if not (net.is_loopback or net.is_link_local) and net.prefixlen < 32:
                    out.append(str(net.broadcast_address))
    except Exception:
        pass
    out.append("255.255.255.255")
    return list(dict.fromkeys(out))


def _is_ip(text: str) -> bool:
    try:
        ipaddress.ip_address(text)
        return text not in ("0.0.0.0", "")
    except ValueError:
        return False


def discover(timeout: float = 1.0, targets=None, port: int = DISCOVERY_PORT) -> dict:
    """Broadcast 'whoareyou?' and collect replies for `timeout` s -> {key: Device}."""
    targets = broadcast_addresses() if targets is None else list(targets)
    found = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind(("", 0))
        for target in targets:
            try:
                s.sendto(b"whoareyou?", (target, port))
            except OSError:
                pass
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            s.settimeout(left)
            try:
                data, addr = s.recvfrom(4096)
            except socket.timeout:
                break
            except OSError:  # e.g. ICMP port unreachable reported on Windows
                continue
            m = _REPLY.search(data.decode("utf-8", "replace"))
            if not m:
                continue
            g = m.groups()
            dev = Device(g[7] if _is_ip(g[7]) else addr[0], int(g[8]) or COMMAND_PORT, ids=(g[0], g[5]))
            found[dev.key] = dev
    return found


def probe(dev: Device, timeout: float = 1.0) -> Device:
    """Fill in identity and lasers of `dev` (blocking, about ten round trips)."""

    def opt(param):
        try:
            return client.get_str(param)
        except DecopError:
            return None

    client = DLCPro(dev.ip, dev.port, timeout)
    try:
        client.open()
        dev.system_type = opt("system-type") or ""
        dev.serial = opt("serial-number") or ""
        dev.label = opt("system-label") or ""
        lasers = {}
        for n in range(1, 5):
            product = opt(f"laser{n}:product-name")
            if product is None:
                continue
            product = product or opt(f"laser{n}:type") or ""
            if product:
                label = opt(f"laser{n}:label") or ""
                lasers[n] = f"{product}  ·  {label}" if label else product
        dev.lasers, dev.error = lasers, ""
    except (OSError, DecopError) as e:
        dev.error = str(e) or type(e).__name__
    finally:
        client.close()
        dev.probed = True
    return dev
