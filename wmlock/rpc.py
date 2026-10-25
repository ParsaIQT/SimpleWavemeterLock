"""ARTIQ's calls (sipyco's pc_rpc): every copy of this program answers them, for every laser it shows (its own PC's
and the other PCs': share.py), on a TCP port (PORT). No server of its own is needed, and no sipyco on this PC.

In ARTIQ's device_db, any PC that runs the program (best, one that is always on):

    device_db["wavemeter"] = {"type": "controller", "host": "lab-pc-1", "port": 3280}

and in an experiment (its host code: not a kernel's):

    self.setattr_device("wavemeter")
    self.wavemeter.set_setpoint(398.91121)                # the laser near 398.91121 nm, to exactly that
    self.wavemeter.wait("398.91", mhz=1.0, timeout=10.0)  # ...until it is locked within 1 MHz of it

Which laser (laser, in each call that takes one):
  a name, as its row has it: 'Yb 399' (any case), or part of a name only one has: 'cool'
  a wavelength: a number (nm) is the laser whose setpoint (its reading, without one) is nearest it, within
    NEAR_GHZ; digits as text, the laser whose setpoint begins with them: '398.91' or '398.91XXX'
  a wavemeter channel: '#5' (on the one wavemeter with a channel 5 shown), 'lab-pc-2#5' or 'WS7 1234#5' (on that
    wavemeter, by its name or its PC's)
More than one that fits is refused, saying which: choose by name or channel then.

A laser of this PC's is adjusted here, as if clicked (Engine.remote); another PC's by that PC, asked through share.py
(it must let other PCs adjust its lasers: Network…), its answer waited for. Its row says so: '⇄ setpoint
398.91121 nm from ARTIQ (10.0.0.5)'. A setpoint moved further than NEAR_GHZ (a typo?) is refused, unless forced.

The protocol: sipyco's pc_rpc, as every sipyco since 2019 speaks it, in either of its encodings (pyon v1, Python's
syntax, read here without eval; pyon v2, JSON, offered as sipyco offers it), with one target (TARGET), a thread for
each client. Errors reach the caller as the builtin exceptions they are (LookupError, ValueError, RuntimeError,
PermissionError, TimeoutError).
"""
from __future__ import annotations

import ast
import base64
import builtins
import inspect
import json
import logging
import math
import numbers
import os
import re
import socket
import threading
import time
import traceback
from collections import OrderedDict
from fractions import Fraction

from . import view
from .dlcpro import lan_addresses
from .wavemeter import C_NM_THZ

log = logging.getLogger(__name__)

PORT = 3280         # TCP: ARTIQ's calls
SIM_PORT = 3281     # ...the demo's (--sim), from this PC only
TARGET = "wmlock"   # the one target
INIT = b"ARTIQ pc_rpc\n"
NEAR_GHZ = 30.0     # a laser is near a wavelength within this; a setpoint moved further is refused unless forced
STALE_S = 3.0       # s: a reading older than this is no reading
WATCH_S = 30.0      # s: another PC's readings come at once (fast) for this long after a call that wants them
FRESH_S = 2.0       # s: a reading of another PC's laser is waited for this long, if the one there is old
SHOWN_S = 6.0       # s: another PC's laser just adjusted, its row here waited for this long at most to show it
LINE_MAX = 1 << 20  # bytes of a request at most
CLIENTS_MAX = 64    # clients at once
WAVELENGTHS = (100.0, 20000.0)  # nm: a wavelength outside is a mistake (THz given for nm, say)
OK_MHZ = 20.0       # is_ok: further than this from its setpoint is not okay, by default
OBSERVING_MAX = 256  # observations going on at once at most (observe; the oldest forgotten)
OBSERVED_PART = 20  # [began, s] of another PC's observation fetched at a time (Engine.remote's at most)


# --------------------------------------------------------------------------------------------- pyon
_NAMES = {"null": None, "true": True, "false": False, "inf": math.inf, "nan": math.nan,
          "None": None, "True": True, "False": False}


def _npscalar(dtype, data):
    import numpy

    return numpy.frombuffer(base64.b64decode(data), dtype=dtype)[0]


def _nparray(shape, dtype, data):
    import numpy

    return numpy.frombuffer(base64.b64decode(data), dtype=dtype).copy().reshape(shape)


_CALLS = {"npscalar": _npscalar, "nparray": _nparray, "Fraction": Fraction, "OrderedDict": OrderedDict,
          "slice": slice}  # (pyon v1's)
_TYPES = {"tuple": tuple, "dict": dict, "set": set, "complex": complex, "bytes": base64.b64decode, "slice": slice,
          "Fraction": Fraction, "OrderedDict": OrderedDict, "nparray": _nparray, "npscalar": _npscalar}  # (v2's)


def _number(x) -> bool:
    return isinstance(x, (int, float, complex)) and not isinstance(x, bool)


def _literal(node):
    """A value written in pyon v1 (Python's syntax): literals and pyon's few calls; nothing is evaluated."""
    if isinstance(node, ast.Constant) and node.value is not Ellipsis:
        return node.value
    if isinstance(node, ast.Name) and node.id in _NAMES:
        return _NAMES[node.id]
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        v = _literal(node.operand)
        if _number(v):
            return -v if isinstance(node.op, ast.USub) else v
    elif isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub)):  # (a complex number: 1+2j)
        a, b = _literal(node.left), _literal(node.right)
        if _number(a) and isinstance(b, complex):
            return a + b if isinstance(node.op, ast.Add) else a - b
    elif isinstance(node, ast.Tuple):
        return tuple(_literal(x) for x in node.elts)
    elif isinstance(node, ast.List):
        return [_literal(x) for x in node.elts]
    elif isinstance(node, ast.Set):
        return {_literal(x) for x in node.elts}
    elif isinstance(node, ast.Dict) and None not in node.keys:
        return {_literal(k): _literal(v) for k, v in zip(node.keys, node.values)}
    elif (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in _CALLS
          and not node.keywords):
        return _CALLS[node.func.id](*(_literal(x) for x in node.args))
    raise ValueError(f"not PYON: {ast.dump(node)[:80]}")


def _hook(d: dict):
    """A value of pyon v2 (JSON) of a type JSON has not."""
    if "__jsonclass__" not in d:
        return d
    name, args = d["__jsonclass__"]
    if name not in _TYPES:
        raise ValueError(f"not a PYON type: {name!r}")
    return _TYPES[name](*args)


def decode(line: str, v2: bool):
    """What a line of pyon (v2: JSON; else v1, Python's syntax) says."""
    if v2:
        return json.loads(line, object_hook=_hook)
    return _literal(ast.parse(line.strip(), mode="eval").body)


def _nparray_args(x) -> tuple:
    """A numpy array as pyon has it: its shape, its dtype, its bytes in base64."""
    import numpy

    if x.dtype.kind == "O":
        raise TypeError("a numpy array of objects cannot be sent")
    return tuple(int(k) for k in x.shape), x.dtype.str, base64.b64encode(numpy.ascontiguousarray(x).data).decode()


def _is_nparray(x) -> bool:
    return type(x).__name__ == "ndarray" and type(x).__module__ == "numpy"


def _wrap(x):
    """x for json: its tuples, sets and dicts with keys other than text marked as pyon v2 marks them."""
    if _is_nparray(x):
        shape, dtype, data = _nparray_args(x)
        return {"__jsonclass__": ["nparray", [list(shape), dtype, data]]}
    if isinstance(x, dict):
        if all(isinstance(k, str) for k in x):
            return {k: _wrap(v) for k, v in x.items()}
        return {"__jsonclass__": ["dict", [[[_wrap(k), _wrap(v)] for k, v in x.items()]]]}
    if isinstance(x, list):
        return [_wrap(v) for v in x]
    if isinstance(x, tuple):
        return {"__jsonclass__": ["tuple", [[_wrap(v) for v in x]]]}
    if isinstance(x, (set, frozenset)):
        return {"__jsonclass__": ["set", [[_wrap(v) for v in x]]]}
    if x is None or isinstance(x, (bool, str)):
        return x
    if isinstance(x, numbers.Integral):
        return int(x)
    if isinstance(x, numbers.Real):
        return float(x)
    raise TypeError(f"{x!r} ({type(x).__name__}) cannot be sent")


def _v1(x) -> str:
    """x in pyon v1 (as sipyco's pyon_v1 writes it)."""
    if _is_nparray(x):
        shape, dtype, data = _nparray_args(x)
        return f'nparray({_v1(shape)}, {_v1(dtype)}, b"{data}")'
    if x is None:
        return "null"
    if isinstance(x, bool):
        return "true" if x else "false"
    if isinstance(x, numbers.Integral):
        return repr(int(x))
    if isinstance(x, numbers.Real):
        return repr(float(x))
    if isinstance(x, str):
        return json.dumps(x)
    if isinstance(x, tuple):
        return f"({_v1(x[0])}, )" if len(x) == 1 else "(" + ", ".join(map(_v1, x)) + ")"
    if isinstance(x, list):
        return "[" + ", ".join(map(_v1, x)) + "]"
    if isinstance(x, (set, frozenset)):
        return "{" + ", ".join(map(_v1, x)) + "}"
    if isinstance(x, dict):
        return "{" + ", ".join(f"{_v1(k)}: {_v1(v)}" for k, v in x.items()) + "}"
    raise TypeError(f"{x!r} ({type(x).__name__}) cannot be sent")


def encode(x, v2: bool) -> str:
    """x as a line of pyon (v2: JSON; else v1), without its newline."""
    return json.dumps(_wrap(x), separators=(",", ":")) if v2 else _v1(x)


_BUILTIN = {v: k for k, v in vars(builtins).items() if isinstance(v, type) and issubclass(v, BaseException)}


def packed(exc: BaseException) -> dict:
    """An exception as sipyco sends it (raised again by the client: a builtin one as itself)."""
    tb = traceback.format_tb(exc.__traceback__)
    cls = type(exc)
    if cls in _BUILTIN:
        return {"class": _BUILTIN[cls], "message": str(exc), "traceback": tb}
    return {"class": "GenericRemoteException", "message": traceback.format_exception_only(cls, exc)[0].rstrip(),
            "traceback": tb}


# --------------------------------------------------------------------------------------------- the server
_caller = threading.local()  # the address of the client whose call this thread is answering


def caller() -> str:
    """Who is calling, as rows and the log say it."""
    addr = getattr(_caller, "addr", "")
    return f"ARTIQ ({addr})" if addr else "ARTIQ"


class Server:
    """Answers ARTIQ's calls to target (pc_rpc) on a TCP port, a thread for each client.

    settings: the network's ('share' in the settings file, which this PC's wavemeters share): artiq (answer them; on
    unless False), artiq_port (default_port unless set), artiq_from (only from these PCs: names or addresses; none:
    from any; this PC always), lan (False: from this PC alone)."""

    def __init__(self, target, settings: dict, default_port: int = PORT):
        self.target = target
        self.settings = settings
        self.default_port = default_port
        self.lock = threading.Lock()
        self.error = ""      # why it does not answer ('' if it does, or is off)
        self.listening = ""  # where it listens ('' nowhere)
        self.clients = 0     # connected now
        self.calls = 0       # answered since it started
        self.last = ""       # the last call, and from where...
        self.last_t = 0.0    # ...when (monotonic)
        self._methods = sorted(n for n, v in inspect.getmembers(type(target))
                               if not n.startswith("_") and callable(v))
        self._sock = None
        self._stop = threading.Event()
        self._thread = None
        self._conns = set()
        self._allowed = set()  # the addresses of artiq_from, looked up...
        self._allowed_at = -1e9  # ...then (monotonic)

    @property
    def on(self) -> bool:
        return self.settings.get("artiq", True) is not False

    @property
    def port(self) -> int:
        return int(self.settings.get("artiq_port") or self.default_port)

    def start(self):
        """Listen (if on), in a thread of its own; again, with its settings as they are now, if it was."""
        self.stop()
        with self.lock:
            self.error = self.listening = ""
            self.calls, self.last = 0, ""
        if not self.on:
            return
        host = "" if self.settings.get("lan", True) else "127.0.0.1"
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            if os.name != "nt":  # (a restart is not kept off it by the connections it just closed; Windows does
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # that as it is, and there SO_REUSEADDR
                #                                                          would let another listen on it too)
            s.bind((host, self.port))
            s.listen(16)
            s.settimeout(0.25)
        except OSError as e:
            s.close()
            with self.lock:
                self.error = f"TCP port {self.port} cannot be had ({e.strerror or e}): ARTIQ cannot call this PC"
            log.warning("%s", self.error)
            return
        self._stop = threading.Event()
        self._sock = s
        with self.lock:
            here = ", ".join(lan_addresses()) if not host else "this PC alone"
            self.listening = f"{socket.gethostname()}" + (f" ({here})" if here else "") + f", TCP port {self.port}"
        self._thread = threading.Thread(target=self._accept, args=(s, self._stop), daemon=True, name="artiq")
        self._thread.start()

    def stop(self):
        """Stop listening, and close the clients' connections."""
        self._stop.set()
        s, self._sock = self._sock, None
        if s is not None:
            s.close()
        with self.lock:
            conns = list(self._conns)
            self.listening = ""
        for c in conns:
            try:
                c.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(2.0)
            self._thread = None

    def summary(self) -> str:
        """For the window: where it listens, or why not ('' if off)."""
        if not self.on:
            return ""
        with self.lock:
            if self.error:
                return self.error
            text = f"ARTIQ :{self.port}"
            if self.clients:
                text += f" · {self.clients} client{'s' if self.clients != 1 else ''}"
            return text

    def detail(self) -> str:
        """For the network settings: its state, and the last call."""
        if not self.on:
            return "ARTIQ's calls: not answered"
        with self.lock:
            if self.error:
                return self.error
            text = (f"ARTIQ's calls: answered on {self.listening or 'nothing yet'}; {self.clients} client"
                    f"{'s' if self.clients != 1 else ''} now, {self.calls} call{'s' if self.calls != 1 else ''}")
            if self.last:
                text += f"; the last: {self.last}, {view.fmt_duration(time.monotonic() - self.last_t)} ago"
            return text

    # --------------------------------------------------------------- clients
    def _accept(self, s, stop: threading.Event):
        while not stop.is_set():
            try:
                conn, addr = s.accept()
            except socket.timeout:
                continue
            except OSError:
                return  # (closed: stopped)
            with self.lock:
                full = len(self._conns) >= CLIENTS_MAX
            if full or not self._allows(addr[0]):
                log.warning("ARTIQ call from %s refused: %s", addr[0], "too many clients" if full else
                            "not among the PCs allowed (Network…)")
                conn.close()
                continue
            threading.Thread(target=self._serve, args=(conn, addr), daemon=True, name=f"artiq {addr[0]}").start()

    def _allows(self, ip: str) -> bool:
        hosts = [h.strip() for h in self.settings.get("artiq_from") or [] if isinstance(h, str) and h.strip()]
        if not hosts or ip.startswith("127.") or ip in hosts:
            return True
        now = time.monotonic()
        if now - self._allowed_at > 60.0:  # (names looked up again a minute on)
            found = set()
            for h in hosts:
                try:
                    found.update(socket.gethostbyname_ex(h)[2])
                except OSError:
                    pass
            self._allowed, self._allowed_at = found, now
        return ip in self._allowed

    def _serve(self, conn: socket.socket, addr):
        with self.lock:
            self._conns.add(conn)
            self.clients += 1
        f = None
        try:
            conn.settimeout(None)
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)  # (a client gone without a word: let go)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            f = conn.makefile("rb")
            if f.readline(len(INIT) + 1) != INIT:
                return
            ident = {"targets": [TARGET], "description": f"SimpleWavemeterLock on {socket.gethostname()}: the "
                     f"lasers shown there, and the other PCs'", "features": ["pyon_v2"]}
            conn.sendall((json.dumps(ident, separators=(",", ":")) + "\n").encode())  # (JSON: both read it)
            line = f.readline(256)
            if not line.endswith(b"\n"):
                return
            name, *features = line[:-1].decode().split(" ")
            if name != TARGET or any(x != "pyon_v2" for x in features):
                return
            v2 = "pyon_v2" in features
            conn.sendall((encode(set(self._methods), v2) + "\n").encode())
            _caller.addr = addr[0]
            while not self._stop.is_set():
                line = f.readline(LINE_MAX + 1)
                if not line.endswith(b"\n"):
                    return  # (closed; or a line longer than any call)
                conn.sendall((self._reply(line.decode(errors="replace"), v2, addr[0]) + "\n").encode())
        except OSError:
            pass
        finally:
            if f is not None:
                f.close()  # (the connection stays open while its reader does)
            conn.close()
            with self.lock:
                self._conns.discard(conn)
                self.clients -= 1

    def _reply(self, text: str, v2: bool, ip: str) -> str:
        try:
            try:
                obj = decode(text, v2)
            except (ValueError, SyntaxError, TypeError, RecursionError, MemoryError) as e:
                raise ValueError(f"not a call: {e}") from None
            if not isinstance(obj, dict):
                raise ValueError("not a call: not a dict")
            action = obj.get("action")
            if action == "call":
                name, args, kwargs = obj.get("name"), obj.get("args", ()), obj.get("kwargs", {})
                if name not in self._methods:
                    raise AttributeError(f"no method {name!r}: there are {', '.join(self._methods)}")
                if not isinstance(args, (list, tuple)) or not isinstance(kwargs, dict) or not all(
                        isinstance(k, str) for k in kwargs):
                    raise TypeError("not a call")
                with self.lock:
                    self.calls += 1
                    self.last, self.last_t = f"{name} from {ip}", time.monotonic()
                ret = getattr(self.target, name)(*args, **kwargs)
            elif action == "get_rpc_method_list":
                ret = self._method_list()
            else:
                raise ValueError(f"Unknown action: {action}")
            return encode({"status": "ok", "ret": ret}, v2)
        except Exception as exc:  # (any: the caller gets it, raised again there; never the end of this thread)
            p = packed(exc)
            return encode({"status": "failed", "exception": p, "message": p["message"]}, v2)

    def _method_list(self) -> dict:
        """What sipyco_rpctool's list-methods shows: each method's arguments and what it does."""
        methods = {}
        for name in self._methods:
            fn = getattr(self.target, name)
            spec = inspect.getfullargspec(fn)._asdict()
            spec["annotations"] = {k: v if isinstance(v, str) else getattr(v, "__name__", str(v))
                                   for k, v in spec["annotations"].items()}
            methods[name] = (spec, inspect.getdoc(fn))
        return {"docstring": inspect.getdoc(self.target), "methods": methods}


# --------------------------------------------------------------------------------------------- the calls
def _real(x, what: str) -> float:
    if isinstance(x, bool) or not isinstance(x, (numbers.Real, str)):
        raise TypeError(f"{what}: not a number: {x!r}")
    try:
        v = float(x)
    except ValueError:
        raise ValueError(f"{what}: not a number: {x!r}") from None
    if not math.isfinite(v):
        raise ValueError(f"{what}: not a number: {x!r}")
    return v


def _laser_first(value, laser) -> tuple:
    """(value, laser) as meant, whichever order they came in: the laser first, set_setpoint('Anansi', 398.91121), is
    taken as set_setpoint(398.91121, 'Anansi') (a name is text, a value a number)."""
    if isinstance(value, str) and isinstance(laser, numbers.Real) and not isinstance(laser, bool):
        return laser, value
    return value, laser


def _wavelength(x) -> float:
    nm = _real(x, "wavelength (nm)")
    if not WAVELENGTHS[0] < nm < WAVELENGTHS[1]:
        raise ValueError(f"{nm} nm is not a wavelength a wavemeter reads (nm, vacuum)")
    return nm


class _Laser:
    """A laser shown here: of this PC's (engine), or another PC's (peer)."""

    __slots__ = ("name", "n", "wavemeter", "pc", "engine", "peer", "v")

    def __init__(self, name, n, wavemeter, pc, engine, peer, v):
        self.name, self.n, self.wavemeter, self.pc = name, n, wavemeter, pc
        self.engine, self.peer, self.v = engine, peer, v  # (v: what its row shows, view.py)

    @property
    def who(self) -> str:
        return f"‘{self.name}’" if self.name else f"channel {self.n} of {self.wavemeter}"

    @property
    def key(self) -> tuple:
        """The same laser, whatever its name: (its engine or its PC's instance, its channel)."""
        return (id(self.engine) if self.engine is not None else self.peer.id), self.n

    @property
    def nm(self):
        """Its setpoint, or its reading (nm), or None."""
        if self.v.get("sp"):
            return self.v["sp"]
        f = self.v.get("f")
        return C_NM_THZ / f if f else None

    def said(self) -> str:
        nm = self.nm
        return f"{self.who} ({self.wavemeter}#{self.n}" + (f", {nm:.6f} nm)" if nm else ")")


class Api:
    """SimpleWavemeterLock: the lasers this PC shows, its own and the other PCs'. A laser is chosen by its name
    ('Yb 399', or part of one), its wavelength (a number: the one nearest, within 30 GHz; digits as text: '398.91' or
    '398.91XXX', the one whose setpoint begins with them) or its wavemeter channel ('#5', 'lab-pc-2#5', 'WS7 1234#5').
    Wavelengths are in vacuum, nm."""

    def __init__(self, meters, share=None):
        """meters: () -> this PC's wavemeters (meters.Meter: engine, label); share: this program's instance on the
        network (share.Share: the other PCs' lasers), None: this PC's alone."""
        self._meters, self._share = meters, share
        self._lock = threading.Lock()
        self._observing = {}  # number -> (the laser, its tag), of the observations going on (observe)
        self._number = 0

    # ---------------------------------------------------------------- reading
    def ping(self) -> bool:
        """True: it answers."""
        return True

    def lasers(self) -> list:
        """Every laser shown here, this PC's and the other PCs': a dict each, as status() gives it."""
        now = time.monotonic()
        return [self._status(laser, now) for laser in self._all(now)]

    def status(self, laser) -> dict:
        """All about a laser: name, wavemeter, pc, channel, here (this PC's), setpoint_nm, nm and thz (its reading),
        dev_mhz (the reading less the setpoint), age_s (of the reading), state (off, error, on, engaging, recovering,
        asking, mapping; ref, ref-hold, ref-moved for a reference laser), locked, reference, adjustable, info (the
        line under its digits)."""
        now = time.monotonic()
        return self._status(self._find(laser, now), now)

    def get_wavelength(self, laser) -> float:
        """Its latest reading (nm, vacuum; corrected by the reference laser, if there is one). RuntimeError if it has
        none of the last 3 s."""
        return C_NM_THZ / self._reading(laser)

    def get_frequency(self, laser) -> float:
        """Its latest reading (THz), as get_wavelength."""
        return self._reading(laser)

    def get_setpoint(self, laser):
        """Its setpoint (nm, vacuum), None if it has none."""
        laser = self._find(laser, time.monotonic())
        return laser.v.get("sp") or None

    def get_deviation(self, laser) -> float:
        """Its latest reading less its setpoint (MHz). RuntimeError if it has no setpoint, or no reading of the last
        3 s."""
        laser = self._fresh(self._find(laser, time.monotonic()))
        dev = laser.v.get("dev")
        if dev is None:
            raise RuntimeError(f"{laser.who}: " + ("no setpoint" if not laser.v.get("sp") else self._why_none(laser)))
        self._recent(laser)
        return float(dev)

    def is_locked(self, laser) -> bool:
        """It is locked to its setpoint (a laser its DLC pro locks to a cavity: watched, Relock on)."""
        return self._find(laser, time.monotonic()).v.get("state") == "on"

    def wait(self, laser, mhz=1.0, timeout=10.0, hold=0.0) -> float:
        """Wait until it is locked within mhz (MHz) of its setpoint, and has been for hold s; its deviation then
        (MHz). TimeoutError if not within timeout s; RuntimeError at once if it is not locked (nor locking, nor
        being recovered)."""
        mhz, timeout, hold = (_real(x, what) for x, what in ((mhz, "mhz"), (timeout, "timeout"), (hold, "hold")))
        laser = self._find(laser, time.monotonic())
        end = time.monotonic() + timeout
        since, dev = None, None
        while True:
            now = time.monotonic()
            laser = self._again(laser, now)
            v = laser.v
            state = v.get("state")
            if state not in ("on", "engaging", "recovering", "asking"):
                why = {"mapping": "it is being mapped", "off": "it is not locked"}.get(state) or (
                    "the reference laser has no setpoint" if v.get("anchor") is not None else v.get("info", ""))
                raise RuntimeError(f"{laser.who}: {why or state}")
            dev, age = v.get("dev"), v.get("age")
            if state == "on" and dev is not None and age is not None and age < STALE_S and abs(dev) <= mhz:
                since = now if since is None else since
                if now - since >= hold:
                    return float(dev)
            else:
                since = None
            if now >= end:
                where = f"{dev:+.1f} MHz from its setpoint" if dev is not None else "no reading"
                raise TimeoutError(f"{laser.who}: {where} after {timeout:g} s, not within ±{mhz:g} MHz"
                                   + ("" if state == "on" else f" ({state})"))
            time.sleep(0.02 if laser.engine is not None else 0.05)

    def is_ok(self, *lasers, mhz=OK_MHZ) -> tuple:
        """(True, '') if every laser named is fine to run with: locked (a laser its DLC pro locks: watched, its cavity
        lock holding), its reading of the last 3 s within mhz (MHz) of its setpoint, lasing, neither being recovered
        nor mapped; else (False, what is wrong, laser by laser). Before an oven run, say: is_ok('399', '935')."""
        names = []
        for x in lasers:
            names += list(x) if isinstance(x, (list, tuple, set)) else [x]
        if not names:
            raise TypeError("which lasers: is_ok('399', '935')")
        mhz = _real(mhz, "mhz")
        wrong = []
        for which in names:
            try:
                laser = self._fresh(self._find(which, time.monotonic()))
            except (LookupError, ConnectionError) as e:
                wrong.append(str(e).split(":")[0] if isinstance(e, LookupError) else str(e))
                continue
            why = self._not_ok(laser, mhz)
            if why:
                wrong.append(f"{laser.who}: {why}")
        return not wrong, "; ".join(wrong)

    def observe(self, laser, mhz=None) -> int:
        """Begin observing a laser's lock: its lock dropouts (a laser its DLC pro locks to a cavity) and the times it is
        not locked, from now until end_observe (given mhz, also the times its readings are further than that from its
        setpoint). Returns the observation's number, for end_observe."""
        if mhz is not None:
            mhz = _real(mhz, "mhz")
            if mhz <= 0:
                raise ValueError(f"mhz: {mhz}, not above 0")
        target = self._find(laser, time.monotonic())
        tag = os.urandom(8).hex()
        if target.engine is not None:
            why = target.engine.observe(target.n, tag, mhz)
            if why:
                raise RuntimeError(f"{target.who}: {why}")
        else:
            self._look(target, "observe", tag=tag, mhz=mhz)
        with self._lock:
            self._number += 1
            self._observing[self._number] = (target, tag)
            while len(self._observing) > OBSERVING_MAX:  # (never ended: forgotten, the oldest first)
                del self._observing[next(iter(self._observing))]
            return self._number

    def end_observe(self, observation):
        """End an observation (its number, from observe; or its laser, if only one of its is going on): the times its
        laser was not locked, as a numpy array of [unix time it began, s] rows (n × 2), in order, overlapping ones made
        one. One still going on lasts until now; one that began before observe, from when it began. Times are by the
        clock of the PC whose laser it is."""
        import numpy

        with self._lock:
            number = self._which_observation(observation)
            target, tag = self._observing.pop(number)
        if target.engine is not None:
            e = target.engine
            if not e.end_observe(tag):
                raise LookupError(f"observation {number} of {target.who} is over (ended long ago?)")
            end = time.monotonic() + e.END_S + 1.0
            got = e.observed(tag)
            while got is not None and not got["ready"] and time.monotonic() < end:
                time.sleep(0.05)
                got = e.observed(tag)
            if got is None or not got["ready"]:
                raise RuntimeError(f"observation {number} of {target.who}: not over")
            times = got["times"]
        else:
            times = self._fetch(target, tag, number)
        return numpy.array(times, dtype=float).reshape(-1, 2)

    # ---------------------------------------------------------------- adjusting
    def set_setpoint(self, nm, laser=None, force=False) -> dict:
        """Its setpoint to exactly nm (vacuum). laser: which, by its name ('Anansi'), wavelength or channel (None: the
        one whose setpoint is nearest nm, within 30 GHz); named first, set_setpoint('Anansi', 398.91121), does the same.
        A setpoint moved more than 30 GHz is refused unless force. Returns the laser and its setpoint before
        (was_nm)."""
        nm, laser = _laser_first(nm, laser)
        nm = _wavelength(nm)
        target = self._find(nm if laser is None else laser, time.monotonic())
        return self._setpoint(target, nm, force)

    def set_frequency(self, thz, laser=None, force=False) -> dict:
        """Its setpoint to thz (THz), as set_setpoint."""
        thz, laser = _laser_first(thz, laser)
        thz = _real(thz, "frequency (THz)")
        if thz <= 0:
            raise ValueError(f"{thz} THz is not a frequency")
        return self.set_setpoint(C_NM_THZ / thz, laser, force)

    def shift_setpoint(self, mhz, laser, force=False) -> dict:
        """Its setpoint moved by mhz (MHz, in frequency: + is bluer), as set_setpoint."""
        mhz, laser = _laser_first(mhz, laser)
        mhz = _real(mhz, "mhz")
        target = self._find(laser, time.monotonic())
        sp = target.v.get("sp")
        if not sp:
            raise RuntimeError(f"{target.who} has no setpoint to move")
        return self._setpoint(target, C_NM_THZ / (C_NM_THZ / sp + mhz * 1e-6), force)

    def lock(self, laser) -> None:
        """Lock it to its setpoint (as Lock clicked; a laser its DLC pro locks to a cavity: Relock, watched)."""
        self._adjust(self._find(laser, time.monotonic()), "lock")

    def unlock(self, laser) -> None:
        """Unlock it (as Locked clicked; a laser its DLC pro locks: watched no more)."""
        self._adjust(self._find(laser, time.monotonic()), "unlock")

    # ---------------------------------------------------------------- inside
    def _all(self, now: float) -> list:
        out = []
        share = self._share
        host = share.host if share is not None else socket.gethostname()
        for m in self._meters():
            e = m.engine
            with e.lock:
                rows = [(n, view.row(e, n, now)) for n in e.rows()]
            out += [_Laser(v.get("name", ""), n, m.label, host, e, None, v) for n, v in rows]
        if share is not None and share.on:
            for p in share.snapshot():
                if p.status(now) not in ("live", "late"):
                    continue
                with share.lock:
                    rows = [(n, p.views.get(n), p.got.get(n)) for n in p.rows]
                out += [_Laser(v.get("name", ""), n, p.label, p.host, None, p, self._aged(v, got, now))
                        for n, v, got in rows if v is not None]
        return out

    @staticmethod
    def _aged(v: dict, got, now: float) -> dict:
        """Another PC's row as it is now: the age of its reading, from when it said it."""
        v = dict(v)
        age = v.get("age")
        v["age"] = age + (now - got) if age is not None and got is not None else None
        return v

    def _again(self, laser: _Laser, now: float) -> _Laser:
        """The laser with what its row shows now."""
        if laser.engine is not None:
            e = laser.engine
            with e.lock:
                if laser.n not in e.rows():
                    raise LookupError(f"{laser.who} is not shown any more")
                laser.v = view.row(e, laser.n, now)
            return laser
        share, p = self._share, laser.peer
        if p.status(now) not in ("live", "late"):
            raise ConnectionError(f"{laser.wavemeter} is not heard from")
        share.watch(p.id, WATCH_S)
        with share.lock:
            v, got = p.views.get(laser.n), p.got.get(laser.n)
        if v is None:
            raise LookupError(f"{laser.who} is not shown any more")
        laser.v = self._aged(v, got, now)
        return laser

    def _find(self, which, now: float) -> _Laser:
        lasers = self._all(now)
        if isinstance(which, bool) or which is None or not isinstance(which, (numbers.Real, str)):
            raise TypeError(f"which laser: its name, its wavelength or its channel, not {which!r}")
        if not lasers:
            raise LookupError("no laser is shown here")
        if isinstance(which, numbers.Real):  # a wavelength: the laser nearest it
            nm = _wavelength(which)
            near = [x for x in lasers if x.nm and abs(C_NM_THZ / x.nm - C_NM_THZ / nm) * 1e3 <= NEAR_GHZ]
            return self._one(near, f"near {nm} nm", lasers)
        text = which.strip()
        m = re.fullmatch(r"(.*?)\s*#\s*(\d+)", text)
        if m:  # a wavemeter channel
            where, n = m.group(1).lower(), int(m.group(2))
            found = [x for x in lasers if x.n == n and (where in x.wavemeter.lower() or where in x.pc.lower())]
            return self._one(found, f"{text!r}", lasers)
        low = text.lower()
        exact = [x for x in lasers if x.name.strip().lower() == low]
        if exact:
            return self._one(exact, f"{text!r}", lasers)
        digits = re.fullmatch(r"(\d+(?:\.\d*)?)[xX]*", text)
        if digits:  # a wavelength's first digits
            lo = float(digits.group(1))
            hi = lo + 10.0 ** -len((digits.group(1).split(".") + [""])[1])
            found = [x for x in lasers if x.nm and lo <= x.nm < hi]
            if found:
                return self._one(found, f"{text!r}", lasers)
        return self._one([x for x in lasers if low and low in x.name.lower()], f"{text!r}", lasers)

    @staticmethod
    def _one(found: list, what: str, lasers: list) -> _Laser:
        if len(found) == 1:
            return found[0]
        listed = "; ".join(x.said() for x in (found or lasers)[:12])
        if not found:
            raise LookupError(f"no laser {what}: the lasers shown are {listed}")
        raise LookupError(f"{len(found)} lasers {what}: {listed}. Say which by name or channel "
                          f"('{found[0].wavemeter}#{found[0].n}')")

    def _status(self, laser: _Laser, now: float) -> dict:
        v = laser.v
        f, age = v.get("f"), v.get("age")
        state = v.get("state", "off")
        return {"name": laser.name, "wavemeter": laser.wavemeter, "pc": laser.pc, "channel": laser.n,
                "here": laser.engine is not None, "setpoint_nm": v.get("sp") or None,
                "nm": C_NM_THZ / f if f else None, "thz": f, "dev_mhz": v.get("dev"),
                "age_s": age if age is not None and math.isfinite(age) else None, "state": state,
                "locked": state == "on", "reference": v.get("anchor") is not None,
                "adjustable": laser.engine is not None or bool(laser.peer.control), "info": v.get("info", "")}

    def _fresh(self, laser: _Laser) -> _Laser:
        """The laser, with a reading of the last STALE_S if one comes soon (another PC's: asked for at once)."""
        now = time.monotonic()
        laser = self._again(laser, now)
        end = now + (FRESH_S if laser.engine is None else 0.0)
        while laser.v.get("f") and not (laser.v.get("age") is not None and laser.v["age"] < STALE_S):
            if time.monotonic() >= end:
                break
            time.sleep(0.05)
            laser = self._again(laser, time.monotonic())
        return laser

    def _reading(self, which) -> float:
        laser = self._fresh(self._find(which, time.monotonic()))
        if not laser.v.get("f"):
            raise RuntimeError(f"{laser.who}: {self._why_none(laser)}")
        self._recent(laser)
        return float(laser.v["f"])

    @staticmethod
    def _why_none(laser: _Laser) -> str:
        return f"no reading ({laser.v.get('bad') or 'none'})"

    @staticmethod
    def _recent(laser: _Laser):
        age = laser.v.get("age")
        if age is None or not age < STALE_S:
            raise RuntimeError(f"{laser.who}: no reading for " + (f"{age:.0f} s" if age is not None else "a while"))

    def _setpoint(self, laser: _Laser, nm: float, force) -> dict:
        v = laser.v
        if v.get("anchor") is not None:
            raise RuntimeError(f"{laser.who} is the reference laser: it has no setpoint")
        was = v.get("sp") or None
        old = was or laser.nm
        if old and not force and abs(C_NM_THZ / old - C_NM_THZ / nm) * 1e3 > NEAR_GHZ:
            raise ValueError(f"{nm} nm is {abs(C_NM_THZ / old - C_NM_THZ / nm) * 1e3:.0f} GHz from "
                             f"{laser.who}'s {'setpoint' if was else 'reading'}, {old:.6f} nm: not set (a typo?); "
                             f"force=True sets it")
        found = self._adjust(laser, "setpoint", nm=nm)
        if isinstance(found, dict) and "was" in found:  # (as its PC had it: its row here may be a moment behind)
            was = found["was"]
        return {"laser": laser.name, "wavemeter": laser.wavemeter, "channel": laser.n, "setpoint_nm": nm,
                "was_nm": was}

    def _adjust(self, laser: _Laser, do: str, **args):
        """Done here (this PC's laser) or asked of its PC, its answer waited for; refused: raised, saying why. What it
        says besides (None: nothing)."""
        who = caller()
        if laser.engine is not None:
            ok, why, *found = laser.engine.remote(laser.n, do, args, who)
            found = found[0] if found else None
        else:
            share, p = self._share, laser.peer
            if not p.control:
                raise PermissionError(f"{laser.wavemeter} does not let other PCs adjust its lasers (its Network "
                                      f"settings)")
            ok, why, found = share.ask(p.id, laser.n, do, by=f"{who} via {share.who}", **args)
            if ok:
                self._shown(laser, do, args)
        if not ok:
            raise self._refused(laser, why)
        what = do if not args else f"{do} " + ", ".join(f"{k}={x}" for k, x in args.items())
        log.info("%s on %s: %s, from %s", laser.who, laser.wavemeter, what, who)
        return found

    def _shown(self, laser: _Laser, do: str, args: dict):
        """Another PC's laser, just adjusted there: until its row here shows it (a moment, as a rule), so that what
        is asked next goes by the laser as it is now (a wait() just after a setpoint, by the new one)."""
        if do == "setpoint":
            def done(v):
                return v.get("sp") == args["nm"]
        elif do in ("lock", "unlock"):
            def done(v):
                return (v.get("state") in ("off", "error")) == (do == "unlock")
        else:
            return
        end = time.monotonic() + SHOWN_S
        while True:
            try:
                if done(self._again(laser, time.monotonic()).v):
                    return
            except (LookupError, ConnectionError):
                return
            if time.monotonic() >= end:
                log.warning("%s on %s: %s done there, but not shown here after %g s", laser.who, laser.wavemeter, do,
                            SHOWN_S)
                return
            time.sleep(0.02)

    @staticmethod
    def _refused(laser: _Laser, why: str) -> Exception:
        if why.startswith("adjusting from other PCs is off"):
            return PermissionError(f"{laser.who}: {why}")
        if why.startswith("no answer"):
            return TimeoutError(f"{laser.who}: {why}")
        return RuntimeError(f"{laser.who}: {why}")

    def _look(self, laser: _Laser, do: str, **args):
        """A look at another PC's laser (its lock observed), asked of its PC: what it found."""
        share = self._share
        ok, why, found = share.ask(laser.peer.id, laser.n, do, by=f"{caller()} via {share.who}", **args)
        if not ok:
            raise self._refused(laser, why)
        return found

    def _fetch(self, laser: _Laser, tag: str, number: int) -> list:
        """Another PC's observation, ended there and fetched, a part at a time: its times."""
        self._look(laser, "observe_end", tag=tag)
        end = time.monotonic() + 10.0
        while True:
            got = self._look(laser, "observed", tag=tag, start=0, count=OBSERVED_PART)
            if not isinstance(got, dict):
                raise RuntimeError(f"observation {number} of {laser.who}: {got!r} from its PC")
            if got.get("ready"):
                break
            if time.monotonic() > end:
                raise TimeoutError(f"observation {number} of {laser.who}: its PC did not have it ready")
            time.sleep(0.2)
        times, total = list(got.get("times") or []), int(got.get("total") or 0)
        while len(times) < total:
            more = self._look(laser, "observed", tag=tag, start=len(times), count=OBSERVED_PART)
            if not isinstance(more, dict) or not more.get("times"):
                raise RuntimeError(f"observation {number} of {laser.who}: {len(times)} of its {total} times came")
            times += more["times"]
        return times

    def _which_observation(self, which) -> int:
        """An observation going on (under _lock): by its number, or by its laser if it has only one going on."""
        if isinstance(which, int) and not isinstance(which, bool) and which in self._observing:
            return which
        try:
            laser = self._find(which, time.monotonic())
        except (LookupError, TypeError, ValueError):
            laser = None
        found = [k for k, (x, _) in self._observing.items() if laser is not None and x.key == laser.key]
        if len(found) == 1:
            return found[0]
        if found:
            raise LookupError(f"{len(found)} observations of {laser.who} go on: say which (observe's numbers: "
                              f"{', '.join(map(str, found))})")
        going = ", ".join(f"{k} ({x.who})" for k, (x, _) in self._observing.items()) or "none"
        raise LookupError(f"no observation {which!r} goes on (going on: {going})")

    @staticmethod
    def _not_ok(laser: _Laser, mhz: float) -> str:
        """What is wrong with the laser ('': nothing)."""
        v = laser.v
        state, info = v.get("state"), v.get("info", "")
        if v.get("anchor") is not None:  # (the reference laser)
            if state != "ref":
                return {"ref-hold": "the reference laser holds: it does not track",
                        "ref-moved": "the reference laser has moved"}.get(state, f"the reference laser: {state}")
        elif state != "on":
            return {"off": "not locked", "engaging": "locking", "recovering": "being recovered (a mode hop?)",
                    "asking": "being recovered, step by step", "mapping": "being mapped",
                    "error": info or "not locked"}.get(state, state)
        for flag in ("NOT LASING", "CAVITY LOCK SLIPPING", "SHG SIGNAL GONE"):
            if flag in info:
                return flag.lower()
        if v.get("bad") == "off" and v.get("relock"):
            return ""  # (its wavemeter channel switched off: watched by what its DLC pro says)
        f, age, dev = v.get("f"), v.get("age"), v.get("dev")
        if not f:
            return f"no reading ({v.get('bad') or 'none'})"
        if age is None or not age < STALE_S:
            return "no reading for " + (f"{age:.0f} s" if age is not None else "a while")
        if v.get("anchor") is not None:
            return ""
        if dev is None:
            return "no setpoint"
        if abs(dev) > mhz:
            return f"{dev:+.1f} MHz from its setpoint"
        return ""
