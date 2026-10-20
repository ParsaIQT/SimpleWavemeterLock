"""Several PCs sharing their wavemeters (share.py): two or three instances in this process, each with its simulated
wavemeter and DLC pros, talking UDP over the loopback interface."""
import socket
import time
import zlib

import pytest

from conftest import wait_for
from wmlock.engine import Engine
from wmlock.lock import LockParams
from wmlock.modehop import HopParams
from wmlock.share import MAGIC, Share, address, clean, decode, encode, secret
from wmlock.sim import Demo, SimWavemeter


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("", 0))
        return s.getsockname()[1]


class PC:
    """An instance: its simulated wavemeter and DLC pros, its engine, its share."""

    def __init__(self, label, peers=(), **settings):
        self.demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
        self.engine = Engine({"channels": {str(c): {"name": n} for c, n in self.demo.names.items()}},
                             self.demo.make_source, discovery_targets=["127.0.0.1"],
                             extra_devices=[d.key for d in self.demo.dlcs])
        self.port = free_port()
        self.share = Share(self.engine, {"label": label, "port": self.port, "lan": False,
                                         "peers": [f"127.0.0.1:{p.port}" for p in peers], **settings})

    def start(self):
        self.engine.start()
        self.share.start()
        return self

    def close(self):
        self.share.stop()
        self.share.join(2.0)
        self.engine.stop()
        self.demo.close()

    def peer(self, label):
        return next((p for p in self.share.snapshot() if p.label == label), None)

    def sees(self, label, timeout=5.0):
        """Its view of that PC, once that PC's rows came."""
        assert wait_for(lambda: (p := self.peer(label)) is not None and p.rows and all(n in p.views for n in p.rows),
                        timeout), f"{self.share.label} does not see {label}"
        return self.peer(label)


@pytest.fixture
def two():
    a = PC("Lab A").start()
    b = PC("Lab B", peers=[a]).start()  # (only B knows where A is: A learns of B from its hello)
    yield a, b
    b.close()
    a.close()


def test_datagrams_signed_or_not_and_nothing_else_taken():
    msg = {"k": "state", "id": "abc", "g": "", "c": {"1": {"info": "x" * 500}}}
    data = encode(msg)
    assert data[:4] == MAGIC and data[4] & 1 and len(data) < 200  # (compressed)
    assert decode(data) == msg
    k1, k2 = secret("one"), secret("two")
    signed = encode(msg, k1)
    assert decode(signed, k1) == msg
    assert decode(signed, k2) is None and decode(signed) is None  # wrong key, or none here
    assert decode(data, k1) is None  # unsigned, with a key here
    tampered = bytearray(signed)
    tampered[-1] ^= 1
    assert decode(bytes(tampered), k1) is None
    assert decode(b"whoareyou?") is None and decode(MAGIC + b"\x00not json") is None
    assert decode(MAGIC + b"\x00[1, 2]") is None  # not a message
    bomb = MAGIC + b"\x01" + zlib.compress(b'{"id": "' + b"x" * (3 << 20) + b'"}')
    assert decode(bomb) is None
    deep = MAGIC + b"\x01" + zlib.compress(b"[" * 200000 + b"]" * 200000)
    assert decode(deep) is None


def test_what_another_pc_sends_is_cleaned_before_it_is_painted():
    v = clean({"f": "405", "dev": float("nan"), "age": 1.5, "state": "bogus", "info_c": "#123456", "info": 7,
               "name": "397", "relock": 1, "refch": True, "lp": {"ki": 0.5, "kp": "x"}, "evil": "x"})
    assert v == {"f": None, "dev": None, "age": 1.5, "state": "off", "info_c": "fg", "name": "397", "relock": True,
                 "refch": None, "lp": {"ki": 0.5}}
    assert clean("not a dict") == {}
    assert address("lab3-pc", 47474) == ("lab3-pc", 47474)
    assert address("10.0.0.7:5000", 47474) == ("10.0.0.7", 5000)


def test_odd_datagrams_change_nothing(two):
    """Datagrams that are not what they should be (another program's, a broken one) are ignored, never the end of
    the loop."""
    a, b = two
    pb = a.sees("Lab B")
    rows = list(pb.rows)
    me = b.share.id
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        for msg in ({"k": "fast", "id": me, "g": "", "r": [[1], "x", [float("inf"), 1, 2], None]},
                    {"k": "state", "id": me, "g": "", "rows": "abc", "c": [1, 2]},
                    {"k": "state", "id": me, "g": "", "rows": [1, "x", None], "c": {"1": "nope", "x": {}}},
                    {"k": "ack", "id": me, "g": "", "c": [1]},
                    {"k": "hello", "id": "zz", "g": "", "label": 5, "knows": [1, "a:b:c", "1.2.3.4:99999999"]},
                    {"k": "cmd", "id": "zz", "g": "", "c": "x", "ch": None, "do": 5},
                    {"k": "what", "id": me, "g": ""}):
            s.sendto(encode(msg), ("127.0.0.1", a.port))
        s.sendto(MAGIC + b"\x00" + b"[" * 60000, ("127.0.0.1", a.port))  # (nested too deep for JSON)
        s.sendto(b"whoareyou?", ("127.0.0.1", a.port))
    time.sleep(0.5)
    assert a.share.is_alive() and pb.status(time.monotonic()) == "live"
    assert wait_for(lambda: pb.rows == rows and all(isinstance(pb.views.get(n), dict) for n in rows), 6)
    assert address("10.0.0.7:99999999", 47474) == ("10.0.0.7:99999999", 47474)  # (no such port: a name)


def test_two_pcs_find_each_other_and_show_the_same_rows(two):
    a, b = two
    pa = b.sees("Lab A")
    pb = a.sees("Lab B")  # (it learned of B from B's hello)
    assert pa.rows == a.engine.rows() and pb.rows == b.engine.rows()
    assert pa.host == socket.gethostname() and pa.control and pa.status(time.monotonic()) == "live"
    assert wait_for(lambda: abs(pa.views[1]["f"] - a.engine.channels[1].freq) < 2e-6, 2)  # (2 MHz: the noise)
    v = pa.views[1]
    assert v["name"] == "397" and v["state"] == "off" and v["info"] == "right-click to link a laser"
    assert v["sp"] == 0.0 and v["dev"] is None and not v["relock"] and v["lp"]["ki"] == 0.5
    a.engine.link(1, a.demo.dlcs[0].key, 1)
    a.engine.setpoint_to_current(1)
    a.engine.engage(1)
    assert wait_for(lambda: pa.views[1].get("state") == "on", 3)
    assert pa.views[1]["info"].startswith("laser1 @ ") and pa.views[1]["dev_c"] == "ok"
    assert pa.views[1]["sp"] == a.engine.channels[1].setpoint_nm
    assert "mode-hop" not in pa.views[1]["notes"]
    a.engine.set_hidden(5, True)  # its rows change: so do the others'
    assert wait_for(lambda: 5 not in pa.rows and 5 not in pa.views, 3)


def test_adjustments_from_another_pc_are_done_there(two):
    a, b = two
    pa = b.sees("Lab A")
    ch = a.engine.channels[1]
    b.share.command(pa.id, 1, "setpoint", nm=396.95912)
    assert wait_for(lambda: ch.setpoint_nm == 396.95912, 2)
    assert ch.remote_note == f"setpoint 396.95912 nm from {b.share.who}"
    assert wait_for(lambda: pa.views[1]["sp"] == 396.95912 and "⇄ setpoint 396.95912 nm from Lab B" in
                    pa.views[1]["info"], 2)
    b.share.command(pa.id, 1, "lock")  # no laser linked there: refused, and said why
    assert wait_for(lambda: 1 in pa.feedback, 2)
    assert pa.feedback[1][0] == "not done on Lab A: right-click to link a laser first" and ch.state == "off"
    a.engine.link(1, a.demo.dlcs[0].key, 1)
    b.share.command(pa.id, 1, "lock")
    assert wait_for(lambda: ch.state == "on", 3) and 1 not in pa.feedback
    assert wait_for(lambda: pa.views[1].get("state") == "on", 2)
    b.share.command(pa.id, 1, "lock")  # (locked already: nothing happens)
    a.engine.set_params(1, LockParams(capture=1234.0))
    b.share.command(pa.id, 1, "lock_params", p={"ki": 0.25, "max_step": 0.1})
    assert wait_for(lambda: ch.lock.p.ki == 0.25 and ch.lock.p.max_step == 0.1, 2) and ch.state == "on"
    assert ch.lock.p.capture == 1234.0  # (those not said: as they were)
    b.share.command(pa.id, 1, "rename", name="397 blue")
    assert wait_for(lambda: ch.name == "397 blue", 2)
    b.share.command(pa.id, 1, "unlock")
    assert wait_for(lambda: ch.state == "off", 2) and ch.message == ""
    b.share.command(pa.id, 2, "setpoint_to_reading")
    assert wait_for(lambda: a.engine.channels[2].setpoint_nm is not None, 2)
    assert abs(a.engine.channels[2].setpoint_nm - 866.45205) < 5e-4  # (where its free-running laser is)

    a.share.settings["control"] = False  # watch only, there
    assert wait_for(lambda: not pa.control, 3)
    b.share.command(pa.id, 1, "lock")
    assert wait_for(lambda: 1 in pa.feedback, 2) and ch.state == "off"
    assert pa.feedback[1][0] == "not done on Lab A: adjusting from other PCs is off there"


def test_an_adjustment_sent_again_is_done_once_and_one_unanswered_says_so(two):
    a, b = two
    pa = b.sees("Lab A")
    done = []
    remote = a.engine.remote

    def counted(*args):
        done.append(args[:2])
        return remote(*args)

    a.engine.remote = counted
    lost = [0]
    on_cmd = a.share._on_cmd

    def lossy(msg, addr, now):  # the first two tries never arrive
        if lost[0] < 2:
            lost[0] += 1
            return
        on_cmd(msg, addr, now)

    a.share._on_cmd = lossy
    b.share.command(pa.id, 3, "rename", name="854 red")
    assert wait_for(lambda: a.engine.channels[3].name == "854 red", 3)
    time.sleep(1.0)
    assert done == [(3, "rename")] and lost[0] == 2 and not b.share._pending
    a.share._on_cmd = on_cmd

    b.share.TRIES = 3
    b.share._send_raw = lambda data, addr, send=b.share._send_raw: (
        False if decode(data).get("k") == "cmd" else send(data, addr))  # nothing gets through
    b.share.command(pa.id, 3, "rename", name="never")
    assert wait_for(lambda: 3 in pa.feedback, 3)
    assert pa.feedback[3][0] == "not done: no answer from Lab A" and a.engine.channels[3].name == "854 red"


def test_a_setpoint_overtaken_on_the_way_is_not_done():
    a = PC("Lab A").start()
    try:
        assert wait_for(lambda: 1 in a.engine.rows(), 5)
        ok, why = a.share._do("x", 5, 1, "setpoint", {"nm": 396.9592}, None, "Lab B")
        assert ok and why == ""
        ok, why = a.share._do("x", 4, 1, "setpoint", {"nm": 396.9}, None, "Lab B")  # sent before, came after
        assert ok and why == "superseded"
        assert a.engine.channels[1].setpoint_nm == 396.9592
        assert a.share._do("x", 6, 1, "setpoint", {"nm": "x"}, None, "Lab B") == (False, "not a wavelength: 'x'")
        assert a.share._do("x", 7, 1, "format", {}, None, "Lab B") == (False, "cannot 'format' from another PC")
        a.share._key = secret("k")  # with a key, an adjustment stamped long ago (replayed?) is refused
        assert not a.share._do("x", 8, 1, "setpoint", {"nm": 396.95}, time.time() - 600, "Lab B")[0]
        assert a.share._do("x", 9, 1, "setpoint", {"nm": 396.95}, time.time(), "Lab B") == (True, "")
        assert a.share._do("x", 10, 9, "lock", {}, time.time(), "Lab B") == (False, "no channel 9 shown there")
    finally:
        a.close()


def test_continue_only_allows_the_step_that_was_shown():
    """Step by step, another PC's Continue allows the step its row showed, not one that came meanwhile."""
    a = PC("Lab A").start()
    try:
        e = a.engine
        assert wait_for(lambda: e.channels[7].freq > 0, 5)
        e.link(7, a.demo.dlcs[2].key, 1)
        e.setpoint_to_current(7)
        e.set_hop_params(7, HopParams(enabled=True, cavity=True, shg=True, confirm=True))
        ch = e.channels[7]
        assert e.remote(7, "lock", {}, "Lab B") == (True, "")  # Relock: watched
        assert wait_for(lambda: ch.guard.armed, 5) and ch.remote_note == "Relock clicked from Lab B"
        a.demo.laser(7).lock.jump(1)
        assert wait_for(lambda: bool(ch.hop_prompt), 5)
        first = ch.hop_prompt
        ok, why = e.remote(7, "continue", {"prompt": "something else"}, "Lab B")
        assert not ok and why.startswith("the step it waits for is another one now")
        assert e.remote(7, "unlock", {}, "Lab B") == (False, "it is recovering: stop that instead")
        assert e.remote(7, "continue", {"prompt": first}, "Lab B") == (True, "")
        assert wait_for(lambda: ch.hop_prompt not in ("", first), 5)
        assert e.remote(7, "stop", {}, "Lab B") == (True, "")
        assert wait_for(lambda: ch.state == "off", 5)
        assert e.remote(7, "continue", {"prompt": first}, "Lab B") == (
            False, "it is not waiting for a step to be allowed any more")
    finally:
        a.close()


def test_a_key_or_a_group_keeps_others_out():
    a = PC("Lab A", key="sesame").start()
    b = PC("Lab B", peers=[a], key="other").start()
    c = PC("Lab C", peers=[a], key="sesame").start()
    d = PC("Lab D", peers=[a], key="sesame", group="cold atoms").start()
    try:
        c.sees("Lab A")
        a.sees("Lab C")
        time.sleep(1.0)
        assert [p.label for p in a.share.snapshot()] == ["Lab C"] and not b.share.snapshot()
        assert not d.share.snapshot()
    finally:
        for pc in (d, c, b, a):
            pc.close()


def test_they_all_find_each_other_through_one_they_know():
    """A and C each know only where B is: they learn of each other from B's hellos."""
    b = PC("Lab B").start()
    a = PC("Lab A", peers=[b]).start()
    c = PC("Lab C", peers=[b]).start()
    try:
        for x, y in ((a, "Lab C"), (c, "Lab A"), (b, "Lab A"), (b, "Lab C")):
            x.sees(y, timeout=8)
    finally:
        for pc in (c, a, b):
            pc.close()


def test_a_pc_that_closes_goes_quiet_or_comes_back():
    a = PC("Lab A").start()
    b = PC("Lab B", peers=[a]).start()
    try:
        a.share.LATE_S, a.share.LOST_S = 0.5, 1.0
        pb = a.sees("Lab B")
        send = b.share._send_raw
        b.share._send_raw = lambda data, addr: False  # (its network cable pulled)
        assert wait_for(lambda: pb.status(time.monotonic()) == "lost" and pb.lost, 3)  # (said in the log too)
        b.share._send_raw = send
        assert wait_for(lambda: pb.status(time.monotonic()) == "live" and not pb.lost, 4)
        b.share.stop()
        b.share.join(2.0)
        assert wait_for(lambda: pb.status(time.monotonic()) == "closed", 2)
        again = Share(b.engine, dict(b.share.settings))  # restarted: it takes its old place
        again.start()
        try:
            assert wait_for(lambda: [(p.label, p.id) for p in a.share.snapshot()] == [("Lab B", again.id)], 5)
        finally:
            again.stop()
            again.join(2.0)
    finally:
        b.close()
        a.close()


def test_the_traffic_is_light():
    """Seven channels, two locked: a few kB/s to a PC that shows them, much less to one that has them folded away,
    and nothing but hellos when no one wants them."""
    a = PC("Lab A").start()
    b = PC("Lab B", peers=[a]).start()
    try:
        e = a.engine
        assert wait_for(lambda: all(e.channels[n].freq > 0 for n in (1, 3)), 5)
        for n, (dlc, laser) in ((1, (0, 1)), (3, (1, 1))):
            e.link(n, a.demo.dlcs[dlc].key, laser)
            e.setpoint_to_current(n)
            e.engage(n)
        pa = b.sees("Lab A")
        time.sleep(3.0)
        sent = a.share.bytes_out
        time.sleep(4.0)
        fast = (a.share.bytes_out - sent) / 4.0
        b.share.want(pa.id, "slow")
        time.sleep(3.0)
        sent = a.share.bytes_out
        time.sleep(5.0)
        slow = (a.share.bytes_out - sent) / 5.0
        assert 300 < fast < 6000, fast
        assert slow < 1000, slow
        assert wait_for(lambda: abs(pa.views[1]["f"] - e.channels[1].freq) < 1e-5, 6)  # (still there, now and then)
    finally:
        b.close()
        a.close()
