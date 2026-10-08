"""Minimal TOPTICA DLC pro client and LAN discovery (no dependencies).

Protocol, as implemented by TOPTICA's own SDK (toptica.lasersdk):
  * command line on TCP 1998; every reply, the welcome banner included, ends
    with the prompt '\\n> '
  * (param-ref 'laser1:scan:offset)        -> '70.5', '"text"', '#t' or 'Error: ...'
  * (param-set! 'laser1:scan:offset 70.5)  -> status: 0 ok, >0 warning, <0 error
  * discovery: UDP 'whoareyou?' to port 60010 on each subnet broadcast address;
    every device answers ("sn" "..." "..." "..." "..." "label" n "ip" cmd_port mon_port)
"""
from __future__ import annotations

import ipaddress
import re
import socket
import threading
import time
from dataclasses import dataclass, field

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
