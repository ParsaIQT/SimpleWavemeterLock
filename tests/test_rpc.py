"""ARTIQ's calls (rpc.py): sipyco's pc_rpc as its client speaks it (written here from sipyco's source: pc_rpc.py,
pyon_v1.py, pyon.py; both encodings), and the calls themselves, on two instances (share.py) talking over the loopback
interface, the client calling one of them."""
import asyncio
import base64
import importlib.util
import json
import math
import socket
import sys
import time
import types
from collections import OrderedDict
from fractions import Fraction
from pathlib import Path

import numpy as np
import pytest

from conftest import wait_for
from test_engine import hop_engine, quiet_demo  # noqa: F401 (a fixture)
from test_share import PC
from wmlock import rpc
from wmlock.meters import Meter
from wmlock.wavemeter import C_NM_THZ


# ---------------------------------------------------------------- sipyco's encodings, as its source has them
def _npscalar(dtype, data):
    return np.frombuffer(base64.b64decode(data), dtype=dtype)[0]


def _nparray(shape, dtype, data):
    return np.frombuffer(base64.b64decode(data), dtype=dtype).copy().reshape(shape)


V1_EVAL = {"__builtins__": {}, "null": None, "false": False, "true": True, "inf": np.inf, "slice": slice,
           "nan": np.nan, "Fraction": Fraction, "OrderedDict": OrderedDict, "nparray": _nparray,
           "npscalar": _npscalar}  # (pyon_v1._eval_dict)


def v1_encode(x) -> str:
    """pyon_v1.encode, for what a call carries."""
    if x is None:
        return "null"
    if isinstance(x, bool):
        return "true" if x else "false"
    if isinstance(x, np.ndarray):
        return (f"nparray({v1_encode(x.shape)}, {v1_encode(x.dtype.str)}, "
                f"b\"{base64.b64encode(np.ascontiguousarray(x).data).decode()}\")")
    if isinstance(x, np.generic):
        return f"npscalar({v1_encode(x.dtype.str)}, b\"{base64.b64encode(x.data).decode()}\")"
    if isinstance(x, (int, float)):
        return repr(x)
    if isinstance(x, str):
        return json.dumps(x)
    if isinstance(x, tuple):
        return f"({v1_encode(x[0])}, )" if len(x) == 1 else "(" + ", ".join(map(v1_encode, x)) + ")"
    if isinstance(x, list):
        return "[" + ", ".join(map(v1_encode, x)) + "]"
    if isinstance(x, set):
        return "{" + ", ".join(map(v1_encode, x)) + "}"
    if isinstance(x, dict):
        return "{" + ", ".join(f"{v1_encode(k)}: {v1_encode(v)}" for k, v in x.items()) + "}"
    raise TypeError(x)


def v1_decode(s: str):
    return eval(s, V1_EVAL, {})  # (as sipyco's own client does: our own answers, here)


def _wrap(o):
    """pyon.wrap: tuples, and dicts with keys other than text, kept from json's coercion."""
    if isinstance(o, dict):
        if not all(isinstance(k, str) for k in o):
            return {"__jsonclass__": ["dict", [[[_wrap(k), _wrap(v)] for k, v in o.items()]]]}
        return {k: _wrap(v) for k, v in o.items()}
    if isinstance(o, list):
        return [_wrap(v) for v in o]
    if isinstance(o, tuple):
        return {"__jsonclass__": ["tuple", [[_wrap(v) for v in o]]]}
    return o


def _default(o):
    if isinstance(o, set):
        return {"__jsonclass__": ["set", [[_wrap(v) for v in o]]]}
    if isinstance(o, np.ndarray):
        return {"__jsonclass__": ["nparray", [list(o.shape), o.dtype.str,
                                              base64.b64encode(np.ascontiguousarray(o).data).decode()]]}
    if isinstance(o, np.generic):
        return {"__jsonclass__": ["npscalar", [o.dtype.str, base64.b64encode(o.data).decode()]]}
    raise TypeError(o)


def v2_encode(x) -> str:
    return json.dumps(_wrap(x), default=_default, separators=(",", ":"))


_V2 = {"tuple": tuple, "dict": dict, "set": set, "complex": complex, "bytes": base64.b64decode, "slice": slice,
       "Fraction": Fraction, "OrderedDict": OrderedDict, "nparray": _nparray, "npscalar": _npscalar}


def v2_decode(s: str):
    def hook(d):
        if "__jsonclass__" not in d:
            return d
        name, args = d["__jsonclass__"]
        return _V2[name](*args)

    return json.loads(s, object_hook=hook)


class Remote(Exception):
    """GenericRemoteException."""


def raise_packed(pack):
    """packed_exceptions.raise_packed_exc."""
    import builtins

    cls = Remote if pack["class"] == "GenericRemoteException" else getattr(builtins, pack["class"])
    exc = cls(pack["message"])
    exc.parent_traceback = pack["traceback"]
    raise exc


class SipycoClient:
    """sipyco.pc_rpc.Client's side of the protocol, as its source has it: v2 if the server offers it and v2 is wanted
    (sipyco since July 2025), else v1 (all before)."""

    def __init__(self, port: int, v2: bool = True, target: str = "wmlock"):
        self.sock = socket.create_connection(("127.0.0.1", port), 10)
        self.f = self.sock.makefile("rb")
        self.sock.sendall(b"ARTIQ pc_rpc\n")
        line = self.f.readline().decode()
        self.ident = (v2_decode if v2 else v1_decode)(line)  # (a v1 client reads it with its eval)
        self.v2 = v2 and "pyon_v2" in self.ident.get("features", [])
        self.sock.sendall((target + (" pyon_v2" if self.v2 else "") + "\n").encode())
        self.methods = self._recv()

    def _recv(self):
        line = self.f.readline()
        if not line:
            raise EOFError("closed")
        return (v2_decode if self.v2 else v1_decode)(line.decode())

    def action(self, obj):
        self.sock.sendall(((v2_encode if self.v2 else v1_encode)(obj) + "\n").encode())
        obj = self._recv()
        if obj["status"] == "ok":
            return obj["ret"]
        raise_packed(obj["exception"])

    def call(self, name, *args, **kwargs):
        if name not in self.methods:
            raise AttributeError(name)  # (as sipyco's client: before it sends anything)
        return self.action({"action": "call", "name": name, "args": args, "kwargs": kwargs})

    def close(self):
        self.f.close()  # (else the socket stays open while its reader does)
        self.sock.close()

    close_rpc = close

    def __getattr__(self, name):  # (as sipyco's client has its methods: wm.lasers() for call("lasers"))
        if name.startswith("_") or name not in self.__dict__.get("methods", ()):
            raise AttributeError(name)
        return lambda *args, **kwargs: self.call(name, *args, **kwargs)


def free_tcp_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ---------------------------------------------------------------- pyon
def test_pyon_v1_is_read_without_eval_and_nothing_else_is(tmp_path):
    call = {"action": "call", "name": "set_setpoint", "args": (398.91121, "Yb 399", np.float64(0.5), None),
            "kwargs": {"force": True, "a": [1, -2, 3.5e-07], "t": (1, ), "s": {1, 2}, "inf": -np.inf}}
    got = rpc.decode(v1_encode(call), False)
    assert got["args"] == (398.91121, "Yb 399", 0.5, None) and isinstance(got["args"], tuple)
    assert got["kwargs"] == {"force": True, "a": [1, -2, 3.5e-07], "t": (1, ), "s": {1, 2}, "inf": -math.inf}
    a = rpc.decode(v1_encode(np.arange(6.0).reshape(3, 2)), False)
    assert a.shape == (3, 2) and a[2, 1] == 5.0
    assert math.isnan(rpc.decode("nan", False)) and rpc.decode("(1+2j)", False) == 1 + 2j
    assert rpc.decode('{"x": Fraction(1, 3), "o": OrderedDict([(1, 2)])}', False) == {"x": Fraction(1, 3),
                                                                                       "o": OrderedDict([(1, 2)])}
    marker = tmp_path / "pwned"
    for evil in (f"__import__('os').system('touch {marker}')", f"open('{marker}', 'w')", "[].__class__",
                 "(lambda: 1)()", "npscalar.__globals__", "{**{}}", "x", "1 if 1 else 2", "[x for x in (1,)]",
                 "npscalar('<f8', b'AAAAAAAA', extra=1)"):
        with pytest.raises((ValueError, TypeError)):
            rpc.decode(evil, False)
    assert not marker.exists()


def test_pyon_v2_and_what_our_answers_look_like_to_sipyco():
    call = {"action": "call", "name": "is_ok", "args": ("399", ["935", "Mulungu"], np.float32(2.5)), "kwargs": {}}
    got = rpc.decode(v2_encode(call), True)
    assert got["args"] == ("399", ["935", "Mulungu"], 2.5) and isinstance(got["args"], tuple)
    answer = {"status": "ok", "ret": (False, "‘399’: not locked; ‘Mulungu’: −12.0 MHz from its setpoint"),
              "x": {1: (2, 3)}, "drops": np.array([[1761400000.123456, 0.0031], [1761400001.5, 2.0]]),
              "m": {"set_setpoint", "lock"}, "n": [None, True, 398.91121, -math.inf, 7]}
    for v2, read in ((True, v2_decode), (False, v1_decode)):
        text = rpc.encode(answer, v2)
        assert "\n" not in text and text.isascii()
        back = read(text)
        assert back["ret"] == answer["ret"] and isinstance(back["ret"], tuple)
        assert back["x"] == {1: (2, 3)} and back["m"] == answer["m"] and back["n"] == answer["n"]
        assert isinstance(back["drops"], np.ndarray) and back["drops"].shape == (2, 2)
        assert np.array_equal(back["drops"], answer["drops"])  # (to the last bit)
    empty = v2_decode(rpc.encode(np.zeros((0, 2)), True))
    assert empty.shape == (0, 2) and v1_decode(rpc.encode(np.zeros((0, 2)), False)).shape == (0, 2)


# ---------------------------------------------------------------- the protocol
class Target:
    """A target, for the protocol alone."""

    def add(self, a, b=1):
        """a + b"""
        return a + b

    def fail(self):
        raise ValueError("no good")

    def oops(self):
        raise KeyErrorish("odd")

    def drops(self):
        return np.arange(4.0).reshape(2, 2)

    def _hidden(self):
        return "secret"


class KeyErrorish(Exception):
    pass


@pytest.fixture
def server():
    port = free_tcp_port()
    s = rpc.Server(Target(), {"artiq_port": port, "lan": False})
    s.start()
    assert s.listening and not s.error
    yield s
    s.stop()


@pytest.mark.parametrize("v2", [True, False])
def test_handshake_calls_and_errors_as_sipyco_has_them(server, v2):
    c = SipycoClient(server.port, v2=v2)
    try:
        assert c.ident["targets"] == ["wmlock"] and c.ident["features"] == ["pyon_v2"]
        assert "SimpleWavemeterLock" in c.ident["description"] and c.v2 == v2
        assert c.methods == {"add", "fail", "oops", "drops"}  # (a set; nothing private)
        assert c.call("add", 2, b=3) == 5 and c.call("add", 2.5) == 3.5
        with pytest.raises(ValueError, match="no good") as e:
            c.call("fail")
        assert any("fail" in line for line in e.value.parent_traceback)
        with pytest.raises(Remote, match="KeyErrorish: odd"):
            c.call("oops")
        with pytest.raises(AttributeError):
            c.action({"action": "call", "name": "_hidden", "args": (), "kwargs": {}})
        with pytest.raises(TypeError):
            c.call("add")
        assert np.array_equal(c.call("drops"), np.arange(4.0).reshape(2, 2))
        doc = c.action({"action": "get_rpc_method_list"})
        assert doc["docstring"] == "A target, for the protocol alone."
        spec, text = doc["methods"]["add"]
        assert text == "a + b" and spec["args"] == ["self", "a", "b"] and spec["defaults"] == (1, )
        assert set(spec) == {"args", "varargs", "varkw", "defaults", "kwonlyargs", "kwonlydefaults", "annotations"}
        assert server.calls == 6 and server.last == "drops from 127.0.0.1"  # (calls to its methods)
        assert server.clients == 1
    finally:
        c.close()
    assert wait_for(lambda: server.clients == 0, 2)


def test_anything_else_is_hung_up_on(server):
    for hello, target in ((b"HTTP/1.0 GET /\n", b""), (b"ARTIQ pc_rpc\n", b"other\n"),
                          (b"ARTIQ pc_rpc\n", b"wmlock pyon_v3\n")):
        with socket.create_connection(("127.0.0.1", server.port), 5) as s:
            f = s.makefile("rb")
            s.sendall(hello)
            if target:
                assert json.loads(f.readline())["targets"] == ["wmlock"]
                s.sendall(target)
            assert f.readline() == b""  # (closed)
    c = SipycoClient(server.port)
    try:
        c.sock.sendall(b"{not pyon\n")
        obj = v2_decode(c.f.readline().decode())
        assert obj["status"] == "failed" and obj["exception"]["class"] == "ValueError"
        assert obj["exception"]["message"].startswith("not a call: ")
        assert c.call("add", 1) == 2  # (still answering)
    finally:
        c.close()


def test_its_settings_who_may_call_and_a_port_taken():
    port = free_tcp_port()
    s = rpc.Server(Target(), {"artiq": False, "artiq_port": port, "lan": False})
    s.start()
    assert not s.listening and s.summary() == "" and "not answered" in s.detail()
    s.settings["artiq"] = True
    s.start()
    try:
        assert s.summary() == f"ARTIQ :{port}" and "127.0.0.1, TCP port" in s.detail()
        other = rpc.Server(Target(), {"artiq_port": port, "lan": False})
        other.start()
        assert other.error.startswith(f"TCP port {port} cannot be had") and not other.listening
        assert other.summary() == other.error
        s.settings["artiq_from"] = ["10.1.2.4", "no-such-host.invalid"]
        assert not s._allows("10.1.2.3") and s._allows("10.1.2.4") and s._allows("127.0.0.1")  # (this PC: always)
        s.settings["artiq_from"] = []
        assert s._allows("10.1.2.3")
    finally:
        s.stop()
    assert not s.listening


def test_sipyco_itself_calls_it():
    """sipyco's own client, where it is installed (ARTIQ's PCs): both encodings."""
    pc_rpc = pytest.importorskip("sipyco.pc_rpc")
    port = free_tcp_port()
    s = rpc.Server(Target(), {"artiq_port": port, "lan": False})
    s.start()
    try:
        c = pc_rpc.Client("127.0.0.1", port)  # (AutoTarget: its one target)
        try:
            assert c.add(2, b=3) == 5 and np.array_equal(c.drops(), np.arange(4.0).reshape(2, 2))
            with pytest.raises(ValueError, match="no good"):
                c.fail()
            assert "add" in c.get_rpc_method_list()["methods"]
        finally:
            c.close_rpc()
        serve = s._serve

        def v1_only(conn, addr):  # (a server that does not offer pyon v2: as a client of before July 2025 sees it)
            real = conn.sendall
            conn.sendall = lambda data: real(data.replace(b'"features":["pyon_v2"]', b'"features":[]'))
            serve(conn, addr)

        s._serve = v1_only
        c = pc_rpc.Client("127.0.0.1", port, "wmlock")
        try:
            assert c.add(1.5, 2) == 3.5
        finally:
            c.close_rpc()
    finally:
        s.stop()


# ---------------------------------------------------------------- the calls, on two PCs
@pytest.fixture
def lab():
    a = PC("Lab A").start()
    b = PC("Lab B", peers=[a]).start()
    api = rpc.Api(lambda: [Meter(a.engine, a.share)], a.share)
    port = free_tcp_port()
    server = rpc.Server(api, {"artiq_port": port, "lan": False})
    server.start()
    a.sees("Lab B")
    b.sees("Lab A")
    c = SipycoClient(port)
    yield a, b, c
    c.close()
    server.stop()
    b.close()
    a.close()


def test_every_laser_shown_here_and_which_is_which(lab):
    a, b, c = lab
    got = c.call("lasers")
    assert {("Lab A", n) for n in range(1, 8)} | {("Lab B", n) for n in range(1, 8)} <= {
        (x["wavemeter"], x["channel"]) for x in got}
    assert {x["wavemeter"] for x in got if x["here"]} == {"Lab A"}
    one = c.call("status", "Lab B#1")
    assert one["name"] == "397" and not one["here"] and one["state"] == "off" and not one["locked"]
    assert one["nm"] == pytest.approx(396.95912, abs=2e-5) and one["setpoint_nm"] is None and one["adjustable"]
    assert one["age_s"] is not None and one["age_s"] < 3.0
    with pytest.raises(LookupError, match="2 lasers '397': ‘397’ \\(Lab A#1, 396.959"):
        c.call("status", "397")  # (both PCs have one)
    with pytest.raises(LookupError, match="no laser 'nope': the lasers shown are"):
        c.call("status", "nope")
    assert c.call("status", "lab b#4")["name"] == "423" and c.call("status", "Lab A #2")["name"] == "866"
    b.engine.set_hidden(4, True)  # (B shows no 423: A's alone)
    assert wait_for(lambda: 4 not in a.peer("Lab B").rows, 3)
    assert c.call("status", "423")["wavemeter"] == "Lab A"
    assert c.call("status", "422.79XXX")["channel"] == 4 and c.call("status", "422.7")["channel"] == 4
    assert c.call("status", "#4")["wavemeter"] == "Lab A"  # (the one wavemeter with a channel 4 shown)
    assert c.call("get_setpoint", "423") is None
    assert c.call("get_wavelength", "423") == pytest.approx(422.7919, abs=2e-5)
    assert c.call("get_frequency", "Lab B#1") == pytest.approx(C_NM_THZ / 396.95912, abs=3e-5)
    with pytest.raises(RuntimeError, match="no setpoint"):
        c.call("get_deviation", "423")
    with pytest.raises(TypeError):
        c.call("status", None)


def test_a_setpoint_set_exactly_here_and_on_the_other_pc(lab):
    a, b, c = lab
    a.engine.set_hidden(3, True)  # (A shows no 854: B's alone; B no 423: A's alone)
    b.engine.set_hidden(4, True)
    assert wait_for(lambda: 4 not in a.peer("Lab B").rows, 3)
    r = c.call("set_setpoint", 422.79191)  # the laser nearest it: A's 423
    assert r == {"laser": "423", "wavemeter": "Lab A", "channel": 4, "setpoint_nm": 422.79191, "was_nm": None}
    ch = a.engine.channels[4]
    assert ch.setpoint_nm == 422.79191 and ch.remote_note == "setpoint 422.79191 nm from ARTIQ (127.0.0.1)"
    r = c.call("set_setpoint", 854.44396)  # B's 854, asked of B: done once the call returns
    ch3 = b.engine.channels[3]
    assert r["wavemeter"] == "Lab B" and r["channel"] == 3 and ch3.setpoint_nm == 854.44396
    assert ch3.remote_note.startswith("setpoint 854.44396 nm from ARTIQ (127.0.0.1) via Lab A")
    assert c.call("set_setpoint", 854.44399, "854")["was_nm"] == 854.44396 and ch3.setpoint_nm == 854.44399
    assert wait_for(lambda: c.call("get_setpoint", "Lab B#3") == 854.44399, 2)  # (its row, a moment later)
    with pytest.raises(ValueError, match="GHz from ‘854’'s setpoint, 854.443990 nm: not set"):
        c.call("set_setpoint", 854.7, "854")  # (105 GHz: a typo?)
    assert ch3.setpoint_nm == 854.44399
    assert c.call("set_setpoint", 854.7, "854", force=True)["setpoint_nm"] == 854.7 and ch3.setpoint_nm == 854.7
    c.call("set_setpoint", 422.79192, "422.79XXX")
    assert ch.setpoint_nm == 422.79192
    r = c.call("shift_setpoint", 10.0, "423")
    assert ch.setpoint_nm == r["setpoint_nm"] == pytest.approx(C_NM_THZ / (C_NM_THZ / 422.79192 + 10e-6), abs=1e-12)
    assert c.call("set_frequency", C_NM_THZ / 422.7919, "Lab A#4")["setpoint_nm"] == pytest.approx(422.7919, 1e-15)
    with pytest.raises(LookupError, match="no laser near 500.0 nm"):
        c.call("set_setpoint", 500.0)
    with pytest.raises(ValueError, match="not a wavelength a wavemeter reads"):
        c.call("set_setpoint", 751.2e3)
    with pytest.raises(ValueError, match="not a number"):
        c.call("set_setpoint", "abc", "423")
    with pytest.raises(TypeError, match="not a number"):
        c.call("set_setpoint", [1], "423")


def test_lock_wait_okay_and_unlock_here_and_there(lab):
    a, b, c = lab
    for pc in (a, b):
        pc.engine.link(1, pc.demo.dlcs[0].key, 1)
    c.call("set_setpoint", 396.95912, "Lab A#1")
    c.call("set_setpoint", 396.95912, "Lab B#1")
    ok, why = c.call("is_ok", "Lab A#1", "Lab B#1")
    assert not ok and why == "‘397’: not locked; ‘397’: not locked"
    c.call("lock", "Lab A#1")
    c.call("lock", "Lab B#1")
    assert abs(c.call("wait", "Lab A#1", 2.0, 15.0)) <= 2.0
    assert abs(c.call("wait", "Lab B#1", mhz=2.0, timeout=15.0, hold=0.3)) <= 2.0
    assert c.call("is_locked", "Lab B#1") and b.engine.channels[1].state == "on"
    assert c.call("is_ok", ["Lab A#1", "Lab B#1"]) == (True, "")  # (a tuple: True, '')
    assert abs(c.call("get_deviation", "Lab B#1")) < 3.0
    assert c.call("get_wavelength", "Lab B#1") == pytest.approx(396.95912, abs=1e-5)
    ok, why = c.call("is_ok", "Lab A#1", "nope", "397")
    assert not ok and why == "no laser 'nope'; 2 lasers '397'"
    ok, why = c.call("is_ok", "Lab A#1", mhz=0.0001)
    assert not ok and why.startswith("‘397’: ") and why.endswith("MHz from its setpoint")
    with pytest.raises(TimeoutError, match="not within ±0.0001 MHz"):
        c.call("wait", "Lab A#1", 0.0001, 0.3)
    c.call("unlock", "Lab B#1")
    assert b.engine.channels[1].state == "off"
    with pytest.raises(RuntimeError, match="it is not locked"):
        c.call("wait", "Lab B#1", 1.0, 2.0)
    c.call("unlock", "Lab A#1")
    assert a.engine.channels[1].state == "off"


def test_lock_drops_observed_here_and_on_the_other_pc(lab):
    a, b, c = lab
    for pc in (a, b):
        pc.engine.link(1, pc.demo.dlcs[0].key, 1)
        pc.engine.set_setpoint(1, 396.95912)
        pc.engine.engage(1)
    assert wait_for(lambda: a.engine.channels[1].state == b.engine.channels[1].state == "on", 10)
    na, nb = c.call("observe", "Lab A#1"), c.call("observe", "Lab B#1")
    c.call("observe", "Lab A#2")  # (not locked at all: one unlock, from before; ended by its laser, below)
    since = a.engine.channels[2].state_t
    time.sleep(0.3)
    t_off = time.time()
    c.call("unlock", "Lab A#1")
    c.call("unlock", "Lab B#1")
    time.sleep(0.5)
    t_lock = time.time()
    c.call("lock", "Lab A#1")
    assert wait_for(lambda: a.engine.channels[1].state == "on", 10)
    t_on = time.time()
    t_lock_b = time.time()
    c.call("lock", "Lab B#1")  # (it returns once its row here shows it locking: a moment after)
    assert wait_for(lambda: b.engine.channels[1].state == "on", 10)
    t_on_b = time.time()
    time.sleep(0.3)
    t_last = time.time()
    c.call("unlock", "Lab A#1")  # (still unlocked when the observation ends)
    time.sleep(0.3)
    da = c.call("end_observe", na)
    t_end = time.time()
    db = c.call("end_observe", nb)
    assert isinstance(da, np.ndarray) and da.shape == (2, 2) and db.shape == (1, 2)
    assert da[0, 0] == pytest.approx(t_off, abs=0.15) and t_lock <= da[0, 0] + da[0, 1] <= t_on
    assert da[1, 0] == pytest.approx(t_last, abs=0.15) and da[1, 0] + da[1, 1] == pytest.approx(t_end, abs=0.3)
    assert db[0, 0] == pytest.approx(t_off, abs=0.3) and t_lock_b <= db[0, 0] + db[0, 1] <= t_on_b
    d2 = c.call("end_observe", "Lab A#2")  # (by its laser: its one observation)
    assert d2.shape == (1, 2) and d2[0, 0] == pytest.approx(since, abs=1e-6)  # (from when it began)
    with pytest.raises(LookupError, match="no observation"):
        c.call("end_observe", na)  # (over)
    n = c.call("observe", "Lab B#3", mhz=50.0)
    assert c.call("end_observe", n).shape == (1, 2)  # (unlocked from before: off target too, merged into one)
    assert not a.engine.observations or all(o.t1 is not None for o in a.engine.observations.values())
    assert not a.engine.channels[1].observers and not b.engine.channels[1].observers


def test_another_pc_that_lets_no_one_adjust_its_lasers_can_be_observed(lab):
    a, b, c = lab
    b.share.settings["control"] = False
    assert wait_for(lambda: not a.peer("Lab B").control, 4)
    with pytest.raises(PermissionError, match="Lab B does not let other PCs adjust its lasers"):
        c.call("set_setpoint", 396.959, "Lab B#1")
    assert c.call("status", "Lab B#1")["adjustable"] is False
    n = c.call("observe", "Lab B#1")
    assert c.call("end_observe", n).shape == (1, 2)  # (not locked: one unlock)


def test_lock_dropouts_are_observed_too(quiet_demo):  # noqa: F811
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, drop_ms=0.25))
    e.DROP_SETTLE_S = 1.0
    api = rpc.Api(lambda: [Meter(e, None)])
    try:
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.state == "on" and ch.hop.trans_locked > 0
                        and e.monitors[demo.SHG].method == "recorder", 15)
        time.sleep(1.5)  # (settled)
        n = api.observe("#7")
        time.sleep(0.3)
        t = time.time()
        shg.lock.blip(0.004)
        time.sleep(0.3)
        shg.lock.blip(0.002)  # (just before the end: waited for, as its monitor reads it up to 2 s later)
        time.sleep(0.02)  # (the simulated one comes a moment after it is asked for)
        drops = api.end_observe(n)
        assert drops.shape == (2, 2), drops
        assert drops[0, 0] == pytest.approx(t, abs=0.1) and drops[0, 1] == pytest.approx(0.004, abs=0.0006)
        assert drops[1, 1] == pytest.approx(0.002, abs=0.0006)
        assert api.is_ok("#7") == (True, "")
    finally:
        e.stop()


# ---------------------------------------------------------------- examples/sipyco_client.py, as it says
def examples(monkeypatch):
    """examples/sipyco_client.py, with sipyco.pc_rpc this test's (sipyco's protocol, as its source has it)."""
    pc_rpc = types.ModuleType("sipyco.pc_rpc")
    pc_rpc.Client = lambda host, port, target_name=None, timeout=None: SipycoClient(port)

    class Async:
        async def connect_rpc(self, host, port, target_name):
            self.c = SipycoClient(port)

        def __getattr__(self, name):
            async def call(*args, **kwargs):
                return self.__dict__["c"].call(name, *args, **kwargs)

            return call

        async def close_rpc(self):
            self.c.close()

    pc_rpc.AsyncioClient, pc_rpc.AutoTarget = Async, object()
    pkg = types.ModuleType("sipyco")
    pkg.pc_rpc = pc_rpc
    monkeypatch.setitem(sys.modules, "sipyco", pkg)
    monkeypatch.setitem(sys.modules, "sipyco.pc_rpc", pc_rpc)
    spec = importlib.util.spec_from_file_location(
        "sipyco_client_example", Path(__file__).resolve().parents[1] / "examples" / "sipyco_client.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_sipyco_examples_do_what_they_say(lab, monkeypatch, capsys):
    a, b, c = lab
    ex = examples(monkeypatch)
    where = dict(host="127.0.0.1", port=c.sock.getpeername()[1])
    ex.take_point = lambda i: time.sleep(0.02)
    for pc in (a, b):
        pc.engine.link(1, pc.demo.dlcs[0].key, 1)
        pc.engine.set_setpoint(1, 396.95912)
        pc.engine.engage(1)
    assert wait_for(lambda: a.engine.channels[1].state == b.engine.channels[1].state == "on", 10)
    ex.show_lasers(**where)
    out = capsys.readouterr().out
    assert "Lab A#1   397          396.959" in out and "set 396.959120" in out and "locked" in out
    assert "Lab B#2   866" in out and "no setpoint" in out
    ex.read("Lab A#1", **where)
    assert capsys.readouterr().out.startswith("Lab A#1: 396.95")
    ex.go_to(396.95913, "Lab B#1", **where)
    assert b.engine.channels[1].setpoint_nm == 396.95913
    assert "397: 396.95912 -> 396.95913 nm, locked" in capsys.readouterr().out
    ex.scan("Lab A#1", 396.95910, 396.95914, 3, **where)
    assert a.engine.channels[1].setpoint_nm == 396.95912  # (put back...
    assert not c.call("is_ok", "Lab A#1")[0]  # ...38 MHz from where the scan left it: not okay until it is there)
    c.call("wait", "Lab A#1", 1.0, 10.0)
    assert ex.oven_run(["Lab A#1", "Lab B#1"], seconds=1.2, **where) is True
    assert ex.oven_run(["Lab A#1", "Lab A#2"], seconds=1.0, **where) is False
    assert "oven run cancelled: ‘866’: not locked" in capsys.readouterr().out
    dog = ex.Watchdog(["Lab A#1", "Lab B#1"], every=0.1, **where)
    dog.start()
    time.sleep(0.4)
    assert not dog.bad.is_set()
    b.engine.disengage(1)
    assert dog.bad.wait(3) and dog.why == "‘397’: not locked"
    dog.stop()
    b.engine.engage(1)
    assert wait_for(lambda: b.engine.channels[1].state == "on", 5)

    def point(i):  # (the laser unlocked for points 5 to 9)
        if i == 5:
            b.engine.disengage(1)
        elif i == 10:
            b.engine.engage(1)
            assert wait_for(lambda: b.engine.channels[1].state == "on", 5)
        time.sleep(0.05)

    ex.take_point = point
    drops, bad = ex.measure_with_dropouts("Lab B#1", points=15, **where)
    assert drops.shape == (1, 2) and bad[0] in (4, 5) and bad[-1] in (10, 11) and len(bad) >= 5
    assert "points marked bad" in capsys.readouterr().out
    ex.careful("Lab A#1", **where)
    out = capsys.readouterr().out.splitlines()
    assert out[0].startswith("no laser 'nope'") and "GHz from ‘397’'s setpoint" in out[1]
    asyncio.run(ex.in_asyncio(**where))
    assert capsys.readouterr().out.startswith("False ")  # (no laser '399' there: not okay, and why)
