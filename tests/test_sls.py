"""SLS lock electronics (sls.py): the HTTP client against a simulated unit, what a poll says, the guard of a slave a
PLL beat-locks (its steps, then the whole night failure in the simulation: the master's lock drops, the PLL's manual
lock runs the slave away and stops it lasing, its TA switches itself off; guarded, none of that)."""
import json
import math
import random
import re
import socket
import statistics
import threading
import time

import pytest

from conftest import wait_for
from wmlock import sls, view
from wmlock.engine import Engine, Track, step_fit
from wmlock.meters import Elsewhere, Meter, row_holds
from wmlock.rpc import Api
from wmlock.share import Share, clean
from wmlock.sim import Demo, SimWavemeter
from wmlock.simsls import FakeSls
from wmlock.sls import Guard, Look, SlsParams, View


# ------------------------------------------------------------------------------------------------- the client
@pytest.fixture
def unit():
    u = FakeSls("P1029T", {"AutoLockEnable": True, "LockFrequency": 8.0e7, "PLLLockState": "MANUAL",
                           "PLLLockStatus": lambda: "PLL Manual Lock", "PLLManualLockEnable": False, "Count": 3})
    yield u
    u.close()


def test_client_reads_writes_and_says_what_went_wrong(unit):
    c = sls.Sls(unit.host, unit.port)
    assert c.get("PLLLockState") == "MANUAL"
    assert c.get("PLLLockStatus") == "PLL Manual Lock"
    assert sls.Sls._value("pllmanuallockenable=1\n", "PLLManualLockEnable") == "1"  # (its reply's name, any case)
    assert sls.as_float(c.get("LockFrequency")) == 8.0e7 and c.get("Count") == "3"
    assert sls.as_bool(c.get("AutoLockEnable")) is True
    assert c.set("PLLManualLockEnable", True) == "true" and unit.params["PLLManualLockEnable"] is True
    assert unit.log == [("PLLManualLockEnable", True)]
    assert c.set("LockFrequency", 80.5e6) and unit.params["LockFrequency"] == 80.5e6
    got = c.values()
    assert got["PLLLockState"] == "MANUAL" and got["PLLManualLockEnable"] == "1"
    assert c.command("save") == "OK" and unit.log[-1] == ("appCommand", "save")
    with pytest.raises(sls.SlsError, match="Unknown parameter"):
        c.get("NoSuchThing")
    with pytest.raises(sls.SlsError, match="read only"):
        c.set("PLLLockStatus", "x")
    unit.down = "close"
    with pytest.raises(OSError):
        c.get("PLLLockState")
    unit.down = ""
    assert c.get("PLLLockState") == "MANUAL"  # (answers again: a new connection)
    c.close()


def test_values_parsed_and_told_apart():
    assert sls.parse("A=1\nB=x=y\n\nnot a value\n C = 2 \n") == {"A": "1", "B": "x=y", "C": "2"}
    assert sls._text(b"<html><body>A=1<br>B=2</body></html>").split() == ["A=1", "B=2"]
    for s, b in (("1", True), ("TRUE", True), ("on", True), ("0", False), ("false", False), ("maybe", None)):
        assert sls.as_bool(s) is b
    assert sls.as_float("1.010000000000000E+07") == 1.01e7 and sls.as_float("nan") is None
    assert sls.names("pll", 2)["manual"] == "Servo2ManualLockEnable"
    assert sls.names("pll", 0)["amp"] == "PLLAmplitude" and sls.names("pdh")["trans"] == "Transmission"


def test_what_a_master_servo_says():
    v = sls.pdh_view(0.0, {"AutoLockEnable": "1", "AutoLockState": "Locked: 1 (0d 01:00:21)", "Transmission": "3.5",
                           "TransmissionThreshold": "0.2", "LockCount": "1", "LockTime": "3621"})
    assert v.holds and v.mode == "auto" and v.count == 1 and v.held == 3621.0
    v = sls.pdh_view(0.0, {"AutoLockEnable": "1", "AutoLockState": "Acquiring", "Transmission": "0.02"})
    assert v.holds is False and "Acquiring" in v.why
    v = sls.pdh_view(0.0, {"AutoLockEnable": "1", "AutoLockState": "Locked: 2 (0d 00:00:01)", "Transmission": "0.1",
                           "TransmissionThreshold": "0.2", "LockDetectEnable": "1"})
    assert v.holds is False and "below its threshold" in v.why
    v = sls.pdh_view(0.0, {"AutoLockEnable": "0", "AutoLockState": "Idle"})
    assert v.holds is None and v.mode == "" and "autolock is off" in v.why


def test_what_a_pll_says():
    p = SlsParams(kind="pll", host="x")
    base = {"PLLManualLockEnable": "1", "AutoLockEnable": "0", "PLLLockState": "MANUAL", "PLLAmplitude": "0.88",
            "PLLAmpThreshold": "0.3", "PZTServoOutput": "0.65", "CurrentServoOutput": "0.001"}
    v = sls.pll_view(10.0, base, 0, p)
    assert v.mode == "manual" and v.holds and v.amp == 0.88 and v.slow == 0.65
    assert sls.pll_view(10.0, {**base, "PLLAmplitude": "0.05"}, 0, p).why.startswith("no beat: amplitude 0.050 V")
    assert "at its rail" in sls.pll_view(10.0, {**base, "PZTServoOutput": "-4.9"}, 0, p).why
    later = sls.pll_view(10.1, {**base, "PZTServoOutput": "0.95"}, 0, p, prev=v)
    assert later.holds is False and "running away (+3.0 V/s" in later.why
    auto = sls.pll_view(10.0, {**base, "PLLManualLockEnable": "0", "AutoLockEnable": "1",
                               "PLLLockState": "AUTO_SEARCHING"}, 0, p)
    assert auto.mode == "auto" and auto.holds is False and "AUTO_SEARCHING" in auto.why
    off = sls.pll_view(10.0, {**base, "PLLManualLockEnable": "0"}, 0, p)
    assert off.mode == "" and off.holds is None
    three = {"Servo2ManualLockEnable": "1", "Servo2AutoLockEnable": "0", "Servo2LockState": "MANUAL",
             "Servo2SlowServoOutput": "0.2", "Servo1ManualLockEnable": "0"}
    v3 = sls.pll_view(0.0, three, 2, p)
    assert v3.mode == "manual" and v3.holds and v3.amp is None  # (no amplitude on a 3-channel PLL)
    assert sls.pll_view(0.0, three, 1, p).mode == ""
    for stale in ("FAIL", "Aborting:STATE_FAIL", "IDLE"):  # (its autolock's state, from a failed try: not its manual
        assert sls.pll_view(0.0, {**three, "Servo2LockState": stale}, 2, p).holds  # lock's)
    tried = {**three, "Servo2ManualLockEnable": "0", "Servo2AutoLockEnable": "1", "Servo2LockState": "FAIL"}
    assert sls.pll_view(0.0, tried, 2, p).holds is False  # (its autolock's: it goes by it)
    # ...but where its fast output and its phase error sit holding it, learned: far from there, it has lost it
    banded = SlsParams(kind="pll", host="x", fast_mid=0.002, fast_dev=0.004, phase_mid=0.0, phase_dev=0.1)
    there = {**three, "Servo2FastServoOutput": "0.03", "Servo2PhaseError": "-0.6"}
    assert sls.pll_view(0.0, there, 2, banded).holds  # (within 0.05 V (its floor), and 8 × 0.1)
    far = sls.pll_view(0.0, {**there, "Servo2FastServoOutput": "-0.2"}, 2, banded)
    assert far.holds is False and far.band and far.why == ("its fast output -0.200 V, far from where it holds it "
                                                           "(+0.002 ± 0.004 V)")
    lost = sls.pll_view(0.0, {**there, "Servo2PhaseError": "2.5"}, 2, banded)
    assert lost.holds is False and lost.band and lost.why.startswith("its phase error +2.500, far from where")
    assert sls.pll_view(0.0, {**there, "Servo2FastServoOutput": "-0.2"}, 2, p).holds  # (not learned: not watched)


def test_a_unit_without_get_values_is_read_one_by_one():
    u = FakeSls("P1029X", {"AutoLockEnable": True, "AutoLockState": "Locked: 1 (0d 00:00:10)", "Transmission": 3.0,
                           "TransmissionThreshold": 0.2, "LockCount": 1, "LockTime": 10.0})
    real = u.answer
    u.answer = lambda q: "Unknown command\n" if q.startswith("getValues") else real(q)
    got = []
    w = sls.SlsWatch(SlsParams(kind="pdh", host=u.host, port=u.port), step=lambda w, v, fresh: got.append(v) or [])
    w.start()
    try:
        assert wait_for(lambda: any(v is not None for v in got), 3)
        v = next(v for v in got if v is not None)
        assert v.holds and v.trans == 3.0 and not w.whole and not w.error
    finally:
        w.stop()
        w.join(3)
        u.close()


def test_probe_says_what_the_unit_is():
    demo = Demo(discovery_port=0, beat=True)
    try:
        text = sls.probe(demo.pll.sls.host, demo.pll.sls.port, "pll", 0)
        assert "PLL, firmware Oct  9 2024" in text and "manual lock (MANUAL)" in text, text
        assert "which servo, 1 to 3" not in text
        assert "a PLL, not a Laser Servo" in sls.probe(demo.pll.sls.host, demo.pll.sls.port, "pdh")
        assert "not a 3-channel PLL" in sls.probe(demo.pll.sls.host, demo.pll.sls.port, "pll", 2)
        assert "Laser Servo" in sls.probe(demo.pdh.sls.host, demo.pdh.sls.port, "pdh")
        assert "127.0.0.1:1:" in sls.probe("127.0.0.1", 1, "pll", timeout=0.5)
    finally:
        demo.close()


def test_survey_says_what_each_unit_is():
    """Find units (the settings dialog): each unit in a line, by its name on the LAN; names not found said so."""
    demo = Demo(discovery_port=0, beat=True, beat_servo=1)
    try:
        said = sls.survey(demo.pll.sls.host, demo.pll.sls.port)
        assert said.startswith("3-channel PLL: servo 1 manual lock at 80 MHz; servo 2 off"), said
        assert sls.survey(demo.pdh.sls.host, demo.pdh.sls.port).startswith("Laser Servo (to the cavity): Locked: 1")
        assert sls.survey("p1029z-not-here") == "not found on the LAN"
        found = sls.units("P1029Z-not-here-", letters="AB", timeout=0.5)
        assert found == [("P1029Z-not-here-A", "not found on the LAN"), ("P1029Z-not-here-B", "not found on the LAN")]
    finally:
        demo.close()
    one = Demo(discovery_port=0, beat=True)
    try:
        said = sls.survey(one.pll.sls.host, one.pll.sls.port)
        assert said.startswith("one-channel PLL at 80 MHz: manual lock (MANUAL)"), said
    finally:
        one.close()


def test_test_says_why_a_unit_does_not_answer(monkeypatch):
    """Test, a unit not answering: its name not known here; nothing listening on its port; its port taking connections
    but answering nothing; its port dropped while VNC (or SSH) answers; nothing answering at all."""
    assert sls.diagnose("p1029z-not-here")[0] == "name"
    closed = socket.socket()
    closed.bind(("127.0.0.1", 0))
    port = closed.getsockname()[1]
    closed.close()  # (nothing listens there now)
    what, why = sls.diagnose("127.0.0.1", port, timeout=0.5)
    assert what == "refused" and f"nothing listens on its port {port}" in why
    mute = socket.socket()  # (takes connections, never answers)
    mute.bind(("127.0.0.1", 0))
    mute.listen(8)
    try:
        port = mute.getsockname()[1]
        assert sls.diagnose("127.0.0.1", port, timeout=0.5)[0] == "open"
        monkeypatch.setattr(sls, "SLOW_S", 0.3)
        said = sls.probe("127.0.0.1", port, "pll", 1, timeout=0.3)
        assert said.startswith(f"127.0.0.1:{port}: timed out") and "takes connections on its port" in said, said
    finally:
        mute.close()
    states = {sls.PORT: "silent", sls.VNC_PORT: "open", sls.SSH_PORT: "refused"}
    monkeypatch.setattr(sls, "lookup", lambda host: "192.168.1.113")
    monkeypatch.setattr(sls, "knock", lambda ip, p, timeout=1.5: states[p])
    what, why = sls.diagnose("192.168.1.113")
    assert what == "dropped" and why.startswith("192.168.1.113 is there (VNC answers), but its port 8080 does not "
                                                "answer at all: a firewall drops it"), why
    states[sls.VNC_PORT] = "silent"
    assert sls.diagnose("192.168.1.113")[0] == "absent"


def test_a_slow_whole_list_is_read_a_value_at_a_time():
    """A unit whose whole list (getValues) takes longer than a poll may: Test and its watch read its values one at a
    time instead."""
    u = FakeSls("P1029S", {"AutoLockEnable": True, "AutoLockState": "Locked: 1 (0d 00:00:10)", "Transmission": 3.0,
                           "TransmissionThreshold": 0.2, "LockCount": 1, "LockTime": 10.0, "AppVersion": "AppVer:X"})
    real = u.answer
    u.answer = lambda q: (time.sleep(1.6), real(q))[1] if q.startswith("getValues") else real(q)
    try:
        said = sls.probe(u.host, u.port, "pdh", timeout=0.5)
        assert said.startswith(f"{u.host}: Laser Servo, firmware X") and "Locked: 1" in said, said
        assert re.search(r"\(its whole list did not come in 0\.5 s: read a value at a time, as it is polled: \d+ ms "
                         r"each; the first \d+ ms, behind the whole list\)$", said), said
        got = []
        w = sls.SlsWatch(SlsParams(kind="pdh", host=u.host, port=u.port), step=lambda w, v, fresh: got.append(v) or [])
        w.start()
        try:
            assert wait_for(lambda: any(v is not None for v in got), 6)
            assert not w.whole and next(v for v in got if v is not None).holds
        finally:
            w.stop()
            w.join(3)
    finally:
        u.close()


def test_find_units_looks_on_this_pcs_private_networks():
    """Find units, names not known here: this PC's private networks (only those) asked on the units' port; a unit
    found so said by its address; one answering VNC only said so."""
    import ipaddress

    nets = sls.private_nets(["192.168.1.50", "139.184.52.46", "10.0.0.7", "127.0.0.1", "169.254.3.4", "x"])
    assert nets == [ipaddress.IPv4Network("192.168.1.0/24"), ipaddress.IPv4Network("10.0.0.0/24")]
    demo = Demo(discovery_port=0, beat=True, beat_servo=1)
    try:
        port = demo.pll.sls.port
        loop = [ipaddress.IPv4Network("127.0.0.0/30")]
        assert sls.scan(loop, (port,), timeout=0.3) == {"127.0.0.1": [port]}
        found = sls.units("P1029Z-not-here-", letters="A", port=port, nets=loop)
        assert found[0] == ("P1029Z-not-here-A", "not found on the LAN")
        assert found[1][0] == "127.0.0.1" and found[1][1].startswith("3-channel PLL: servo 1 manual lock"), found
    finally:
        demo.close()


def test_settings_from_before_and_what_is_learned_afresh():
    old = SlsParams.from_dict({"kind": "pll", "host": "P1029H", "master": 9, "beat": 80.0})  # (before beat_known)
    assert old.beat_known and old.beat == 80.0 and old.master_where == "channel 9"
    new = SlsParams.from_dict({"kind": "pll", "host": "P1029F", "servo": 1, "master": 3, "master_on": "Lab A",
                               "beat": 0.0, "beat_known": True})
    assert new.beat_known and new.beat == 0.0 and new.master_where == "channel 3 on Lab A"
    assert not SlsParams.from_dict({"kind": "pll", "host": "P1029F"}).beat_known  # (learned)
    assert SlsParams().master_where == ""
    banded = SlsParams(kind="pll", host="x", fast_mid=0.1, fast_dev=0.01, phase_dev=0.2)
    d = banded.to_dict()
    assert not set(SlsParams.FRESH) & set(d) and SlsParams.from_dict(d).fast_dev == 0.0  # (learned each time it holds)


def test_a_row_says_whether_its_lock_holds():
    """A master on another PC: whether its lock holds, as its own PC says on its row (or, a PC running an older
    version, by its row's words)."""
    assert row_holds({"holds": True, "state": "off"}) == (True, "")
    assert row_holds({"holds": False, "holds_why": "Acquiring"}) == (False, "Acquiring")
    assert row_holds({"holds": None, "state": "on"}) == (None, "")
    assert row_holds({"state": "on", "info": "locked by SLS Laser Servo   SLS LOCK LOST   SLS P1029D: …"}) == (
        False, "sls lock lost")
    assert row_holds({"state": "recovering", "info": ""}) == (False, "being recovered")
    assert row_holds({"state": "on", "info": "locked by SLS Laser Servo   SLS P1029D: Locked: 3"}) == (True, "")
    assert row_holds({"state": "off", "info": "…"}) == (None, "")
    assert clean({"holds": True, "holds_why": "x"}) == {"holds": True, "holds_why": "x"}
    assert clean({"holds": "yes"}) == {"holds": None}  # (only what it may be)


def test_a_master_followed():
    """Its master's readings, as they come (from its own PC: now and then, fed again): the median of its last
    three, if the last is fresh; steady since its last jump."""
    t = Track()
    assert t.point(0.0, 3.0) is None and t.steady_s(0.0, 3.0) is None
    for k, f in enumerate((400.0, 400.0 + 1e-6, 400.0 + 9e-6)):
        t.feed(f, 1.0 + 0.1 * k)
    assert t.point(1.5, 3.0) == pytest.approx(400.0 + 1e-6, abs=1e-9)  # (one odd reading: not it)
    t.feed(500.0, 1.2)  # (the same reading's time again: left out)
    assert list(t.f)[-1] == pytest.approx(400.0 + 9e-6) and t.point(5.0, 3.0) is None  # (too old: none)
    assert t.steady_s(1.5, 3.0) == pytest.approx(0.3)  # (8 MHz: a jump, steady since)
    t.feed(None, 2.0)  # (no reading)
    assert t.point(2.0, 3.0) is None and t.steady_s(2.0, 3.0) is None


# ------------------------------------------------------------------------------------------------- the guard
def pv(mode="manual", holds=True, why="", amp=None, amp_min=None, t=0.0):
    return View("pll", t, mode=mode, holds=holds, why=why, amp=amp, amp_min=amp_min)


def look(now, pll, **kw):
    kw.setdefault("armed", True)
    kw.setdefault("master", True)
    kw.setdefault("master_s", 100.0)
    return Look(now=now, pll=pll, **kw)


def test_guard_releases_when_the_master_is_lost_and_relocks_once_it_holds():
    g = Guard(SlsParams(kind="pll", host="x"))
    assert g.step(look(0.0, pv())) == [] and g.stage == "hold"
    assert g.step(look(1.0, pv(), fresh=False, master=False)) == [("release", "its master's lock was lost")]
    assert g.stage == "released" and g.relocking
    off = pv(mode="", holds=None)
    assert g.step(look(1.1, off, master=False)) == []        # (its master not back)
    assert g.step(look(2.0, off, master_s=1.0)) == []         # (back, not long enough)
    assert g.step(look(4.0, off, master_s=3.0, can_steer=True)) == [("steer", 0.0)] and g.stage == "steer"
    assert g.step(look(4.5, off, near_n=2, can_steer=True)) == []
    assert g.step(look(5.0, off, near_n=3, can_steer=True)) == [("unsteer",), ("engage",)]
    assert g.stage == "engage"
    assert g.step(look(5.1, pv())) == []                      # (settling, then holding...)
    assert g.step(look(5.0 + Guard.SETTLE_S + 0.5, pv())) == []
    assert g.step(look(5.0 + Guard.SETTLE_S + Guard.VERIFY_S, pv())) == [("held",)] and g.stage == "hold"
    assert not g.relocking and g.tries == 0


def test_guard_releases_after_two_polls_not_holding_and_a_search_by_amplitude():
    g = Guard(SlsParams(kind="pll", host="x", capture=0.6, amp_usual=0.88))
    assert g.step(look(0.0, pv(amp=0.88, amp_min=0.3))) == [] and g.stage == "hold"
    lost = pv(holds=False, why="no beat: amplitude 0.010 V", amp=0.01, amp_min=0.3)
    assert g.step(look(0.1, lost)) == []  # (one poll: not yet)
    assert g.step(look(0.2, lost)) == [("release", "no beat: amplitude 0.010 V")]
    off = pv(mode="", holds=None, amp=0.01, amp_min=0.3)
    assert g.step(look(4.0, off, master_s=10.0, can_steer=True)) == [("steer", 0.0)]
    # there, no beat in its band: the next point, half a capture up, then down...
    assert g.step(look(5.0, off, near_n=Guard.STEER_N + Guard.DWELL_N, can_steer=True)) == [("steer", 0.3)]
    assert g.step(look(6.0, off, near_n=Guard.STEER_N + Guard.DWELL_N, can_steer=True)) == [("steer", -0.3)]
    # its beat in its band (0.7 of its usual at least): engaged at once, wherever the wavemeter says it is
    weak = pv(mode="", holds=None, amp=0.5, amp_min=0.3)
    assert g.step(look(6.5, weak, near_n=1, can_steer=True)) == []  # (0.5 < 0.7 * 0.88)
    there = pv(mode="", holds=None, amp=0.7, amp_min=0.3)
    assert g.step(look(6.6, there, near_n=1, can_steer=True)) == [("unsteer",), ("engage",)]


def test_guard_tries_again_gives_up_and_tries_again_later():
    g = Guard(SlsParams(kind="pll", host="x", tries=2))
    off = pv(mode="", holds=None)
    g.step(look(0.0, off))  # (armed with its PLL off: relocked)
    assert g.stage == "released" and g.why == "its PLL was off"
    assert g.step(look(4.0, off)) == [("engage",)]  # (nothing to steer it with: engaged as it is)
    bad = pv(holds=False, why="its slow output running away")
    acts = g.step(look(4.0 + Guard.SETTLE_S + 0.05, bad))
    assert acts == [("release", "it did not hold: its slow output running away")] and g.stage == "released"
    assert g.tries == 1
    assert g.step(look(5.0, off)) == [("engage",)]
    acts = g.step(look(5.0 + Guard.SETTLE_S + 0.05, bad))
    assert acts[0][0] == "release" and acts[1][0] == "gave_up" and g.stage == "failed"
    assert g.why == "2 tries in a row failed, the last: its slow output running away" and g.relocking is False
    assert "gave up" in g.said(6.0) and "tries again in 289 s" in g.said(6.0 + 10.0)  # (RETRY_S: 300 s)
    assert g.step(look(6.0 + Guard.RETRY_S, off)) == [] and g.stage == "released" and g.tries == 0
    for k in range(Guard.GIVE_UPS - 1):  # (given up GIVE_UPS times in a row: no more tries)
        t = 1000.0 * (k + 1)
        g.step(look(t, off))
        g.step(look(t + Guard.SETTLE_S + 0.05, bad))
        g.step(look(t + 1.0, off))
        g.step(look(t + 1.0 + Guard.SETTLE_S + 0.05, bad))
        assert g.stage == "failed" and g.gave_up == k + 2
        g.step(look(t + 1.0 + Guard.RETRY_S + 1.0, off))
    assert g.stage == "failed" and "Relock to try again" in g.said(1e5)


def test_guard_has_a_slave_that_stopped_lasing_recovered_first():
    g = Guard(SlsParams(kind="pll", host="x"))
    g.step(look(0.0, pv()))
    g.step(look(1.0, pv(), master=False))
    off = pv(mode="", holds=None)
    acts = g.step(look(1.5, off, master=False, lasing=False, can_recover=True))
    assert acts == [("recover", "it does not lase", True)] and g.stage == "recover"  # (its master not back yet)
    assert g.step(look(2.0, off, lasing=False, recovering=True, can_recover=True)) == []
    assert g.step(look(9.0, off, recovered="", can_recover=True)) == [] and g.stage == "released"
    assert g.step(look(9.1, off, can_steer=True)) == [("steer", 0.0)]
    # an engagement that did not hold stopped it lasing again: recovered again
    g2 = Guard(SlsParams(kind="pll", host="x"))
    g2.step(look(0.0, off))
    g2.step(look(4.0, off))  # (engaged)
    g2.step(look(4.0 + Guard.SETTLE_S + 0.05, pv(holds=False, why="x")))
    assert g2.step(look(4.5, off, lasing=False, can_recover=True))[0][0] == "recover"
    # with nothing to recover it with: waited for, then given up
    g3 = Guard(SlsParams(kind="pll", host="x"))
    g3.step(look(0.0, off))
    assert g3.step(look(1.0, off, far="no reading", can_recover=False)) == []
    acts = g3.step(look(1.0 + Guard.WAIT_S + 1, off, far="no reading", can_recover=False))
    assert acts[0][0] == "gave_up" and "no mode-hop recovery set up" in acts[0][1]


def test_guard_leaves_alone_what_it_does_not_have():
    g = Guard(SlsParams(kind="pll", host="x"))
    assert g.step(look(0.0, pv(mode="auto"))) == [] and g.stage == "auto"  # (its autolock has it: watched)
    assert g.step(look(1.0, pv(mode="auto", holds=False, why="x"), master=False)) == []
    assert g.step(look(2.0, pv())) == [] and g.stage == "hold"
    assert g.step(look(3.0, pv(mode="", holds=None))) == [("disarm", "its PLL was switched off at its box")]
    assert g.stage == "idle"
    g.step(look(4.0, pv()))
    assert g.step(look(5.0, pv(), armed=False)) == [] and g.stage == "idle"
    assert g.step(look(6.0, None)) == []  # (armed while its PLL does not answer: nothing to go by)


# ------------------------------------------------------------------------------------------------- the night
@pytest.fixture
def lab():
    """The 739 SHG's sisters: a master an SLS Laser Servo locks to the cavity, a slave an SLS PLL beat-locks 80 MHz
    above it with its manual lock, a TA-SHG pro that stops lasing 1.3 mA below its current (and starts only 2.4 mA
    above), its TA switching itself off after 3 s without seed."""
    demo = Demo(discovery_port=0, beat=True, wavemeter=SimWavemeter(noise=0.05), exposure=0.004)
    yield demo
    demo.close()


def make_engine(demo, tmp_path, guard=True, hop=True, beat=80.0, hop_kw=None, **sls_kw):
    m, s = demo.MASTER, demo.SLAVE
    slave = {"name": "739 slave", "link": {"device": demo.sister.key, "laser": 1},
             "sls": {"kind": "pll", "host": "127.0.0.1", "port": demo.pll.sls.port, "master": m, "beat": beat,
                     "guard": guard, **sls_kw}}
    if hop:
        slave["hop"] = {"enabled": True, "i_nom": 150.0, **(hop_kw or {})}
    cfg = {"channels": {str(m): {"name": "739 master", "sls": {"kind": "pdh", "host": "127.0.0.1",
                                                               "port": demo.pdh.sls.port}},
                        str(s): slave}}
    e = Engine(cfg, demo.make_source, discovery_targets=["127.0.0.1"], drop_path=tmp_path / "drops.jsonl")
    e.start()
    assert wait_for(lambda: all(e.channels.get(n) and e.channels[n].freq > 0 and e.channels[n].sls_view is not None
                                for n in (m, s)), 5)
    return e


def ta_on(demo) -> bool:
    return bool(demo.sister.params["laser1:amp:cc:enabled"])


def test_the_night_failure_unguarded(lab, tmp_path):
    """Watched only: the master's lock drops for half a second; the PLL's manual lock runs the slave's current and
    piezo to their rails, the slave stops lasing, its TA switches itself off; the master back, it stays so. Both
    dropouts are logged, the slave's going on."""
    e = make_engine(lab, tmp_path, guard=False)
    m, s = e.channels[lab.MASTER], e.channels[lab.SLAVE]
    try:
        assert wait_for(lambda: s.sls_view.holds and m.sls_view.holds, 3)
        lab.pdh.drop(0.5)
        assert wait_for(lambda: not lab.laser(lab.SLAVE).ecdl.lasing, 3)
        assert wait_for(lambda: m.drops_total == 1, 3), m.drops
        assert wait_for(lambda: not ta_on(lab), 6)  # (its seed power read now and then: the TA's protection)
        assert lab.pll.manual and not lab.pll.locked and abs(lab.pll.slow) > 1.0  # (still engaged, run away)
        assert s.sls_drop is not None and s.drops_total == 0  # (the slave's dropout goes on)
        row = view.row(e, lab.SLAVE, time.monotonic())
        assert "NOT LASING" in row["info"] and row["info_c"] == "err" and view.sls_flag(s) == "BEAT LOCK LOST"
        assert "slow -5.000 V, fast -1.000 V" in row["info"]  # (its PLL's outputs at their rails)
        assert "SLS LOCK" not in view.row(e, lab.MASTER, time.monotonic())["info"]  # (the master held again)
        rec = json.loads((tmp_path / "drops.jsonl").read_text().splitlines()[0])
        assert rec["channel"] == lab.MASTER and rec["method"] == "sls-pdh" and 400 < rec["duration_ms"] < 900
    finally:
        e.stop()


def test_guarded_the_slave_keeps_lasing_and_is_relocked(lab, tmp_path):
    """Guarded (Relock): the master's lock lost, its PLL is released before it runs the slave away; once the master
    has held MASTER_S, the slave brought to its beat point by the wavemeter, its PLL engaged and holding again. Its TA
    never went off; its dropout logged, observed (ARTIQ's observe), and is_ok says so meanwhile."""
    e = make_engine(lab, tmp_path)
    m, s = e.channels[lab.MASTER], e.channels[lab.SLAVE]
    api = Api(lambda: [Meter(e)])
    try:
        e.engage(lab.MASTER)
        e.engage(lab.SLAVE)
        assert wait_for(lambda: s.sls_guard is not None and s.sls_guard.stage == "hold", 3), s.sls_guard.stage
        assert m.state == "on" and s.state == "on"
        tag = "obs"
        assert e.observe(lab.SLAVE, tag) == ""
        lab.pdh.drop(0.5)
        t0 = time.monotonic()
        assert wait_for(lambda: not lab.pll.manual, 1.0), "not released"
        released = time.monotonic() - t0
        assert released < 0.5, released
        assert wait_for(lambda: s.state == "recovering", 1)
        ok, why = api.is_ok("739 slave", "739 master")
        assert not ok and why.startswith("‘739 slave’") and "being recovered" in why, why
        assert wait_for(lambda: s.sls_guard.stage == "hold" and s.state == "on", 25), (s.sls_guard.stage, s.hop_stage)
        assert ta_on(lab) and lab.laser(lab.SLAVE).ecdl.lasing and lab.pll.locked
        assert wait_for(lambda: s.drops_total == 1, 1) and lab.pll.engages >= 1 and m.drops_total == 1
        took = s.drops[-1][1]
        assert Guard.MASTER_S < took < 25, took
        assert "relocked: its PLL holds it again (its master's lock was lost" in s.sls_note
        assert e.end_observe(tag)
        got = None
        assert wait_for(lambda: (e.observed(tag) or {}).get("ready"), 3)
        got = e.observed(tag)
        assert got["total"] >= 1 and abs(got["times"][0][1] - took) < 1.0, got
        assert abs(s.error_mhz) < 1.0  # (its setpoint is its beat point)
        assert view.row(e, lab.SLAVE, time.monotonic())["state"] == "on"
        assert wait_for(lambda: api.is_ok("739 slave", "739 master") == (True, ""), 3), api.is_ok("739 slave")
    finally:
        e.stop()


def test_a_slave_that_stopped_lasing_is_recovered_before_its_ta_switches_off(lab, tmp_path):
    """Its PLL runs it away faster than the guard can release it (the master's lock lost while the guard does not
    watch it, its own unit's polls the only sign): released, the slave dark (it starts again only 2.4 mA above its
    current), its mode-hop recovery jumps its current up at once, before its TA's 3 s are up; relocked after."""
    e = make_engine(lab, tmp_path)
    s = e.channels[lab.SLAVE]
    try:
        e.engage(lab.SLAVE)  # (the master not armed, and not watched for the slave: its unit's settings cleared)
        assert wait_for(lambda: s.sls_guard is not None and s.sls_guard.stage == "hold", 3)
        e.set_sls(lab.MASTER, SlsParams())  # (its master's lock no longer watched: the beat alone tells)
        lab.laser(lab.SLAVE).stop_lasing()  # (as a runaway would: at once)
        lab.pdh.drop(0.5)
        assert wait_for(lambda: not lab.pll.manual, 2), "not released"
        assert wait_for(lambda: s.runner is not None, 3), (s.sls_guard.stage, s.lasing)
        assert wait_for(lambda: lab.laser(lab.SLAVE).ecdl.lasing, 4)
        assert ta_on(lab)
        assert wait_for(lambda: s.sls_guard.stage == "hold", 60), (s.sls_guard.stage, s.hop_stage, s.message)
        assert ta_on(lab) and lab.pll.locked
    finally:
        e.stop()


def test_three_channel_pll_relocked_by_the_wavemeter_alone(tmp_path):
    """A 3-channel PLL's servo has no beat amplitude: lost, by its slow output running away (and the wavemeter);
    relocked where the wavemeter puts it on its beat point."""
    demo = Demo(discovery_port=0, beat=True, beat_servo=2, wavemeter=SimWavemeter(noise=0.05), exposure=0.004)
    try:
        e = make_engine(demo, tmp_path, servo=2)
        s = e.channels[demo.SLAVE]
        try:
            assert s.sls_view.amp is None and s.sls_view.mode == "manual"
            e.engage(demo.MASTER)
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.sls_guard.stage == "hold", 3)
            demo.pdh.drop(0.5)
            assert wait_for(lambda: not demo.pll.manual, 1.0)
            assert demo.pll.sls.written("Servo2ManualLockEnable")[-1] is False
            assert wait_for(lambda: s.sls_guard.stage == "hold" and demo.pll.locked, 25), s.sls_guard.stage
            assert ta_on(demo)
        finally:
            e.stop()
    finally:
        demo.close()


def test_settings_kept_and_the_unit_not_answering(lab, tmp_path):
    e = make_engine(lab, tmp_path)
    s = e.channels[lab.SLAVE]
    try:
        cfg = e.to_config()["channels"][str(lab.SLAVE)]["sls"]
        assert cfg["kind"] == "pll" and cfg["master"] == lab.MASTER and cfg["beat"] == 80.0 and cfg["beat_known"]
        assert SlsParams.from_dict(cfg).to_dict() == s.sls.to_dict()
        e.engage(lab.SLAVE)
        lab.pll.sls.down = "close"
        assert wait_for(lambda: s.sls_error, 3)
        row = view.row(e, lab.SLAVE, time.monotonic())
        assert "SLS UNIT NOT ANSWERING" in row["info"], row["info"]
        lab.pll.sls.down = ""
        assert wait_for(lambda: not s.sls_error, 5)
        e.set_sls(lab.SLAVE, SlsParams())  # (no SLS lock any more: unarmed, unwatched)
        assert s.state == "off" and lab.SLAVE not in e.sls_watches and "sls" not in s.to_config()
    finally:
        e.stop()


# ------------------------------------------------------------------------------------------- its master elsewhere
def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("", 0))
        return s.getsockname()[1]


@pytest.fixture
def sirius(tmp_path):
    """Sirius, as in the lab: a slave a 3-channel PLL's servo 1 (P1029F) beat-locks with its manual lock, at its
    master's frequency (an AOM making up the PLL's 80 MHz), on another PC (Lab B) than its master (the other SHG, an
    SLS Laser Servo locking it to the cavity, on Lab A); each PC with its engine and its instance on the network,
    the slave's master as the network names it, its beat learned."""
    demo = Demo(discovery_port=0, beat=True, beat_servo=1, beat_offset=0.0, wavemeter=SimWavemeter(noise=0.05),
                exposure=0.004)
    m, s = demo.MASTER, demo.SLAVE
    master = {"name": "739 SHG", "sls": {"kind": "pdh", "host": "127.0.0.1", "port": demo.pdh.sls.port}}
    a = Engine({"channels": {str(m): master}}, demo.make_source, discovery_targets=["127.0.0.1"],
               drop_path=tmp_path / "a.jsonl")
    b = Engine({"channels": {str(s): {"name": "Sirius", "link": {"device": demo.sister.key, "laser": 1},
                                      "hop": {"enabled": True, "i_nom": 150.0},
                                      "sls": {"kind": "pll", "host": "127.0.0.1", "port": demo.pll.sls.port,
                                              "servo": 1, "master": m, "master_on": "Lab A"}}}},
               demo.make_source, discovery_targets=["127.0.0.1"], drop_path=tmp_path / "b.jsonl")
    b.BEAT_LEARN_S = 2.0
    port = free_port()
    share_a = Share(a, {"label": "Lab A", "port": port, "lan": False})
    share_b = Share(b, {"label": "Lab B", "port": free_port(), "lan": False, "peers": [f"127.0.0.1:{port}"]})
    b.elsewhere = Elsewhere(lambda: [Meter(b, share_b)], share_b)
    for x in (a, share_a, b, share_b):
        x.start()
    try:
        yield demo, a, b, share_a
    finally:
        for x in (share_b, share_a):
            x.stop()
            x.join(2.0)
        b.stop()
        a.stop()
        demo.close()


def test_sirius_and_its_master_on_another_pc(sirius):
    """Its master picked as the network shows it, followed from the other PC: its readings, and its lock as its own
    PC watches it. Its beat learned, near 0. The master's lock lost over there: its PLL released at once, and not
    engaged again while the master is lost; relocked once it holds, its TA on all along. Its master's PC gone, it
    is left holding."""
    demo, a, b, share_a = sirius
    s = b.channels[demo.SLAVE]
    assert wait_for(lambda: ("Lab A", demo.MASTER, "739 SHG") in b.elsewhere.lasers(), 10)
    assert wait_for(lambda: s.sls_view is not None and (s.master_row or {}).get("holds") is True, 10), s.master_row
    assert s.master_row["label"] == "Lab A" and s.master_row["name"] == "739 SHG"
    assert not s.sls.beat_known and s.sls_view.amp is None and s.sls_view.mode == "manual"
    b.engage(demo.SLAVE)
    assert wait_for(lambda: s.sls_guard is not None and s.sls_guard.stage == "hold", 3)
    assert wait_for(lambda: s.sls.beat_known, 15), (len(s.beat_seen), s.sls_since)
    assert abs(s.sls.beat) < 0.5, s.sls.beat  # (on its master's frequency, the same wavemeter reading both)
    info = view.row(b, demo.SLAVE, time.monotonic())["info"]
    assert re.search(r"master 739 SHG \(channel 9 on Lab A\), beat [+-]0\.\d\d MHz", info), info
    assert wait_for(lambda: s.sls.fast_dev > 0, 10)  # (where its outputs sit, holding it: learned)
    demo.pdh.drop(4.0)
    t0 = time.monotonic()
    assert wait_for(lambda: not demo.pll.manual, 1.0), "not released"
    assert wait_for(lambda: (s.master_row or {}).get("holds") is False, 1.5), s.master_row  # (as Lab A says)
    assert "739 SHG (channel 9 on Lab A), its lock lost" in view.master_said(s)
    while time.monotonic() - t0 < 3.8:  # (its master lost: not engaged meanwhile)
        assert not demo.pll.manual, s.sls_guard.stage
        time.sleep(0.05)
    assert wait_for(lambda: s.sls_guard.stage == "hold" and demo.pll.locked, 30), (s.sls_guard.stage, s.sls_guard.why)
    assert ta_on(demo) and demo.laser(demo.SLAVE).ecdl.lasing
    assert wait_for(lambda: s.drops_total == 1, 2) and "relocked: its PLL holds it again" in s.sls_note
    share_a.stop()  # (its master's PC closes)
    assert wait_for(lambda: s.master_row is None, 5)
    assert "not seen (its PC not heard from)" in view.row(b, demo.SLAVE, time.monotonic())["info"]
    time.sleep(1.0)
    assert demo.pll.manual and demo.pll.locked and s.sls_guard.stage == "hold"  # (left holding)


def test_its_masters_servo_polled_from_the_slaves_pc(lab, tmp_path):
    """Its master's SLS Laser Servo polled by the slave's own PC (its master's lock not watched on its row): lost,
    the slave's PLL released at once; relocked once it holds again."""
    e = make_engine(lab, tmp_path, master_host="127.0.0.1", master_port=lab.pdh.sls.port)
    s = e.channels[lab.SLAVE]
    try:
        e.set_sls(lab.MASTER, SlsParams())  # (its own row: not watched)
        assert wait_for(lambda: s.mview is not None and s.mview.holds and s.m_since is not None, 3)
        assert "its SLS servo polled from here too, 127.0.0.1:" in view.notes(e, lab.SLAVE)
        e.engage(lab.SLAVE)
        assert wait_for(lambda: s.sls_guard is not None and s.sls_guard.stage == "hold", 3)
        lab.pdh.drop(2.0)
        assert wait_for(lambda: s.mview.holds is False, 0.5)
        assert wait_for(lambda: not lab.pll.manual, 1.0), "not released"
        assert view.master_said(s) == "739 master (channel 9), its lock lost (“Acquiring”)", view.master_said(s)
        assert wait_for(lambda: s.sls_guard.stage == "hold" and lab.pll.locked, 30), s.sls_guard.stage
        assert ta_on(lab)
        e.set_sls(lab.SLAVE, SlsParams.from_dict({**s.sls.to_dict(), "master_host": ""}))
        assert lab.SLAVE not in e.master_watches and s.mview is None  # (no longer polled)
    finally:
        e.stop()


def test_where_its_outputs_sit_learned_afresh_when_engaged_again(tmp_path):
    """A 3-channel PLL holding it: where its fast output sits learned; far from there, it has lost the beat
    (released); engaged again, where it sits now learned afresh, not held against it."""
    demo = Demo(discovery_port=0, beat=True, beat_servo=1, wavemeter=SimWavemeter(noise=0.05), exposure=0.004)
    try:
        e = make_engine(demo, tmp_path, servo=1)
        e.BEAT_LEARN_S = 1.0
        s = e.channels[demo.SLAVE]
        try:
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.sls_guard.stage == "hold" and s.sls.fast_dev > 0, 8), s.sls_guard.stage
            assert abs(s.sls.fast_mid) < 0.01 and s.sls.fast_dev < 0.02
            demo.pll.sls.params["Servo1FastServoOutput"] = lambda: 0.4  # (it sits elsewhere: as lost)
            assert wait_for(lambda: not demo.pll.manual, 1.0), "not released"
            assert "its fast output +0.400 V, far from where it holds it" in s.sls_guard.why, s.sls_guard.why
            assert wait_for(lambda: s.sls_guard.stage == "hold" and demo.pll.locked, 30), s.sls_guard.stage
            assert wait_for(lambda: s.sls.fast_dev > 0, 8) and s.sls.fast_mid == pytest.approx(0.4)
            time.sleep(1.0)
            assert s.sls_guard.stage == "hold" and demo.pll.manual  # (held, not released again)
        finally:
            e.stop()
    finally:
        demo.close()


def test_a_beat_typed_wrong_is_learned_again_not_held_against_it(tmp_path):
    """Its beat set to 80 MHz (the PLL's lock frequency, typed in by mistake: with an AOM making that up, it sits on
    its master's frequency, a beat of 0): Relock does not release it for reading 80 MHz off its beat point; once its
    PLL has held it, its beat is learned again (0) and gone by from then on."""
    demo = Demo(discovery_port=0, beat=True, beat_servo=1, beat_offset=0.0, wavemeter=SimWavemeter(noise=0.05),
                exposure=0.004)
    try:
        e = make_engine(demo, tmp_path, servo=1, beat=80.0)
        e.BEAT_LEARN_S = 2.0
        s = e.channels[demo.SLAVE]
        try:
            assert s.sls.beat_known and s.sls.beat == 80.0 and not s.beat_ok  # (as saved: not gone by yet)
            assert "(as last learned: checked once its PLL has held it 10 s)" in view.row(e, demo.SLAVE,
                                                                                          time.monotonic())["info"]
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.sls_guard is not None and s.sls_guard.stage == "hold", 3)
            assert wait_for(lambda: s.beat_e is not None and abs(s.beat_e + 80.0) < 2.0, 3), s.beat_e
            assert s.beat_off_n == 0 and demo.pll.manual  # (80 MHz off what it was told: not held against it)
            assert wait_for(lambda: s.beat_ok, 10), (len(s.beat_seen), s.sls_since)
            assert abs(s.sls.beat) < 0.5 and s.sls_guard.stage == "hold" and demo.pll.manual and demo.pll.locked
            assert wait_for(lambda: s.beat_e is not None and abs(s.beat_e) < 1.0, 2)
            assert e.to_config()["channels"][str(demo.SLAVE)]["sls"]["beat"] == s.sls.beat  # (kept)
        finally:
            e.stop()
    finally:
        demo.close()


def slow_unit(unit, whole_s: float, one_s: float):
    """A unit as slow as P1029F: one request at a time, its whole list taking whole_s, a value one_s."""
    import threading

    real, gate = unit.answer, threading.Lock()

    def answer(q):
        with gate:
            time.sleep(whole_s if q.startswith("getValues") else one_s)
            return real(q)

    unit.answer = answer


def test_a_slow_unit_is_polled_a_value_at_a_time_and_its_slave_still_guarded(tmp_path):
    """Its PLL on a unit whose whole list takes 2 s and a value 0.1 s, one request at a time: read a value at a time
    (only what its guard needs, but every tenth poll); its master's lock lost, the poll under way is cut short and its
    PLL released at once; relocked once its master holds again."""
    demo = Demo(discovery_port=0, beat=True, beat_servo=1, wavemeter=SimWavemeter(noise=0.05), exposure=0.004)
    slow_unit(demo.pll.sls, 2.0, 0.1)
    try:
        m, s = demo.MASTER, demo.SLAVE
        cfg = {"channels": {
            str(m): {"name": "739 master", "sls": {"kind": "pdh", "host": "127.0.0.1", "port": demo.pdh.sls.port}},
            str(s): {"name": "739 slave", "link": {"device": demo.sister.key, "laser": 1},
                     "hop": {"enabled": True, "i_nom": 150.0},
                     "sls": {"kind": "pll", "host": "127.0.0.1", "port": demo.pll.sls.port, "servo": 1, "master": m,
                             "beat": 80.0}}}}
        e = Engine(cfg, demo.make_source, discovery_targets=["127.0.0.1"], drop_path=tmp_path / "drops.jsonl")
        e.start()
        ch = e.channels[s]
        try:
            assert wait_for(lambda: ch.sls_view is not None, 12), ch.sls_error
            w = e.sls_watches[s]
            assert not w.whole and w.client.timeout == sls.SlsWatch.ONE_S and not ch.sls_error
            assert ch.sls_view.mode == "manual" and ch.sls_view.holds
            polls = w.polls
            time.sleep(2.0)
            assert 2 <= w.polls - polls <= 6  # (brief: 4 values, 0.1 s each)
            e.engage(s)
            assert wait_for(lambda: ch.sls_guard is not None and ch.sls_guard.stage == "hold", 5)
            demo.pdh.drop(3.0)
            t0 = time.monotonic()
            assert wait_for(lambda: not demo.pll.manual, 2.0), "not released"
            assert time.monotonic() - t0 < 0.8  # (the poll under way cut short; the write at once)
            assert wait_for(lambda: ch.sls_guard.stage == "hold" and demo.pll.locked, 40), ch.sls_guard.stage
            assert ta_on(demo)
        finally:
            e.stop()
    finally:
        demo.close()


def test_a_wavemeter_recalibrating_moves_the_beat_and_is_no_lost_lock(tmp_path):
    """Its PLL holding it, its master's wavemeter recalibrated (its readings of the master 30 MHz higher from now on,
    the lasers where they were): the slave read 30 MHz off its beat point, steady there, its PLL holding it by all it
    shows: its beat learned again, its PLL never released, no dropout."""
    demo = Demo(discovery_port=0, beat=True, beat_servo=1, beat_offset=0.0, wavemeter=SimWavemeter(noise=0.05),
                exposure=0.004)
    try:
        e = make_engine(demo, tmp_path, servo=1, beat=0.0)
        e.BEAT_LEARN_S = 1.0
        s = e.channels[demo.SLAVE]
        try:
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.sls_guard.stage == "hold" and s.beat_ok, 10), (s.sls_guard.stage, s.beat_ok)
            assert abs(s.sls.beat) < 0.5
            demo.laser(demo.MASTER).misread = 30.0
            t0 = time.monotonic()
            while time.monotonic() - t0 < 7.0 and abs(s.sls.beat + 30.0) > 1.5:
                assert demo.pll.manual and s.sls_guard.stage == "hold", s.sls_guard.why
                time.sleep(0.02)
            assert abs(s.sls.beat + 30.0) < 1.5, s.sls.beat
            assert re.match(r"its beat moved from [+-]0\.\d\d to -(29|30)\.\d\d MHz, its PLL holding it all along "
                            r"\(one of the wavemeters recalibrated\?\)", s.sls_note), s.sls_note
            time.sleep(1.0)
            assert demo.pll.manual and demo.pll.locked and s.sls_guard.stage == "hold" and s.beat_off_n == 0
            assert s.drops_total == 0 and s.sls_drop is None  # (no dropout: it held all along)
        finally:
            e.stop()
    finally:
        demo.close()


def test_a_slave_read_moving_off_its_beat_point_is_still_lost(tmp_path):
    """Read off its beat point and still moving there (no step of its beat: something wrong that its PLL does not
    show), after BEAT_STEP_S it counts: its PLL released."""
    import threading

    demo = Demo(discovery_port=0, beat=True, beat_servo=1, beat_offset=0.0, wavemeter=SimWavemeter(noise=0.05),
                exposure=0.004)
    stop = threading.Event()
    try:
        e = make_engine(demo, tmp_path, servo=1, beat=0.0)
        e.BEAT_LEARN_S, e.BEAT_STEP_S = 1.0, 3.0
        s = e.channels[demo.SLAVE]
        try:
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.sls_guard.stage == "hold" and s.beat_ok, 10)
            master = demo.laser(demo.MASTER)

            def drift():  # (the master read further and further off: 40 MHz/s)
                while not stop.wait(0.05):
                    master.misread += 2.0

            master.misread = 15.0
            threading.Thread(target=drift, daemon=True).start()
            assert wait_for(lambda: not demo.pll.manual, 6), "not released"
            assert "MHz from its beat point" in s.sls_guard.why or "the wavemeter reads it" in (s.sls_note or ""), \
                s.sls_guard.why
        finally:
            stop.set()
            e.stop()
    finally:
        demo.close()


def test_its_optimum_taken_as_on_a_laser_its_dlc_pro_locks(tmp_path):
    """Mode-hop recovery set up on a slave a PLL holds: Relock takes its optimum the first time, as on a laser its DLC
    pro locks; one taken with an SHG stage ticked (in the dialog, not yet OK'd) says why its SHG level is not known."""
    demo = Demo(discovery_port=0, beat=True, wavemeter=SimWavemeter(noise=0.05), exposure=0.004)
    try:
        e = make_engine(demo, tmp_path)
        s = e.channels[demo.SLAVE]
        try:
            assert s.hop.enabled and not s.hop.optimum
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.hop.optimum, 3), s.note
            assert s.hop.opt_i == pytest.approx(150.0, abs=0.5) and s.hop.opt_shg == 0.0
            assert s.opt_why == "no SHG stage ticked" and "SHG" not in s.note  # (none set up: not said on its row)
            e.take_optimum(demo.SLAVE, shg=True)  # (the simulated slave has no SHG stage)
            assert wait_for(lambda: s.opt_why != "no SHG stage ticked", 3)
            assert s.opt_why == "its SHG cavity says “?”, not locked" and "SHG level not known: its SHG" in s.note
        finally:
            e.stop()
    finally:
        demo.close()


# --------------------------------------------------------------------- its piezo trimmed (Sirius: no slow servo)
def test_a_step_told_from_a_drift():
    """What a step did by itself, from the readings either side of it: a drift through both taken off."""
    rng = random.Random(1)
    before = [(0.1 * k, 0.5 + 0.02 * 0.1 * k + rng.gauss(0, 0.002)) for k in range(25)]
    after = [(3.0 + 0.1 * k, 0.49 + 0.02 * (3.0 + 0.1 * k) + rng.gauss(0, 0.002)) for k in range(25)]
    step, sigma, slope = step_fit(before, after)
    assert step == pytest.approx(-0.01, abs=4 * sigma) and sigma < 0.002 and slope == pytest.approx(0.02, abs=0.003)
    naive = statistics.fmean(y for _, y in after) - statistics.fmean(y for _, y in before)
    assert naive == pytest.approx(0.05, abs=0.005)  # (the drift alone: a step five times as big, the other way)


def no_slow_servo(shg=False, below=None) -> Demo:
    """Sirius by itself: its PLL's slow servo unused (as in the lab: it unlocks the laser), its fast output alone
    holding it (and running out of range as it drifts, untrimmed); with an SHG stage (shg); below: GHz below where a
    second mode begins to lase (as near the top of its mode), else in its middle."""
    demo = Demo(discovery_port=0, beat=True, beat_servo=1, beat_offset=0.0, beat_slow=False, beat_shg=shg,
                wavemeter=SimWavemeter(noise=0.05), exposure=0.004)
    if below is not None:
        slave = demo.laser(demo.SLAVE)
        e = slave.ecdl
        with slave._mutex:
            e.nudge(e.misalignment(e.m, *slave._vi()) - (e.zone(1.0)[0] - below))
    return demo


def misalignment(demo) -> float:
    slave = demo.laser(demo.SLAVE)
    with slave._mutex:
        return slave.ecdl.misalignment(slave.ecdl.m, *slave._vi())


def test_its_piezo_trimmed_keeps_its_fast_output_near_0(tmp_path):
    """Sirius's PLL with its slow servo unused: its piezo trimmed (SlsParams.trim). How its fast output follows the
    piezo is learned by small steps (what each did by itself, the slave drifting meanwhile), and the piezo then brings
    it near 0 and keeps it there as the slave drifts on, a small step every few s; its PLL never released (its guard's
    band moving along with each step)."""
    demo = no_slow_servo()
    slave = demo.laser(demo.SLAVE)
    with slave._mutex:
        slave.ecdl.f0 += 120e-6  # (120 MHz off: its fast output at +0.15 V to begin with)
    stop = threading.Event()
    try:
        e = make_engine(demo, tmp_path, beat=0.0, servo=1, trim=True)
        e.TRIM_S = 1.5
        s = e.channels[demo.SLAVE]
        try:
            assert wait_for(lambda: s.sls_view.fast > 0.12, 3)
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.sls.trim_gain, 40), s.trim_said
            assert 0.3 < s.sls.trim_gain < 0.5, s.trim_said  # (0.4 V for a V: 0.4 GHz/V of piezo, 1 GHz/V of fast)
            assert wait_for(lambda: abs(s.trim_level) <= 0.03, 20), (s.trim_level, s.trim_said)

            def drift():  # (4 MHz/s: its fast output up 0.05 V in 12 s, untrimmed)
                while not stop.wait(0.1):
                    with slave._mutex:
                        slave.ecdl.f0 += 0.4e-6

            v0 = demo.sister.offset(1)
            threading.Thread(target=drift, daemon=True).start()
            time.sleep(12.0)
            stop.set()
            assert abs(s.trim_level) < 0.04, (s.trim_level, s.trim_said)
            assert demo.sister.offset(1) - v0 == pytest.approx(-0.12, abs=0.07)  # (48 MHz at 0.4 GHz/V)
            assert demo.pll.losses == 0 and s.sls_guard.stage == "hold" and s.state == "on"
            text = view.notes(e, demo.SLAVE)
            assert "its piezo trimmed to keep its PLL's fast output near 0" in text and "V for a V of piezo" in text
        finally:
            e.stop()
    finally:
        stop.set()
        demo.close()


def test_an_upkeep_of_a_slave_its_pll_holds(tmp_path):
    """Mode-hop recovery's upkeep on Sirius, its piezo trimmed: its PLL holds it on its frequency all along, so its
    current moves with its piezo (by what was learned of how its fast output follows each: its current's by small
    steps, for the upkeep), up to where its SHG signal dips (the top of its mode) and back off it, as on a laser its
    DLC pro locks to a cavity."""
    demo = no_slow_servo(shg=True, below=0.3)
    try:
        e = make_engine(demo, tmp_path, beat=0.0, servo=1, trim=True, trim_gain=0.4,
                        hop_kw={"shg": True, "upkeep": 0.05})
        e.TRIM_S = 1.5
        s = e.channels[demo.SLAVE]
        try:
            assert misalignment(demo) == pytest.approx(2.4, abs=0.15) and demo.slave_shg.main() == 1.0
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.sls.trim_cur, 40), s.trim_said
            assert -0.35 < s.sls.trim_cur < -0.18, s.trim_said  # (-0.25 V for a mA: 4 mA for a V of fast output)
            assert wait_for(lambda: s.note.startswith("upkeep:"), 30), s.note
            assert "the top of its mode at" in s.note and "its piezo with it" in s.note, s.note
            assert wait_for(lambda: s.state == "on", 5)
            a = misalignment(demo)
            assert 2.5 < a < 2.72, a  # (a second mode begins at 2.7 GHz: just below, as by hand)
            assert demo.slave_shg.main() > 0.99 and demo.pll.losses == 0 and s.sls_guard.stage == "hold"
            assert s.last["outcome"] == s.note and s.last["upkeep"]["top"] > s.last["upkeep"]["from"]
        finally:
            e.stop()
    finally:
        demo.close()


def test_an_upkeep_stops_when_its_pll_is_released(tmp_path):
    """An upkeep under way when its master's lock drops: its PLL's guard releases it at once, and the upkeep stops
    (its guard has it from there: recovered if its PLL ran it dark meanwhile, relocked once the master holds again);
    untrimmed meanwhile."""
    demo = no_slow_servo(shg=True)
    try:
        e = make_engine(demo, tmp_path, beat=0.0, servo=1, trim=True, trim_gain=0.4, trim_cur=-0.25,
                        hop_kw={"shg": True, "upkeep": 0.05})
        e.TRIM_S, e.BEAT_LEARN_S = 1.5, 2.0
        s = e.channels[demo.SLAVE]
        try:
            e.engage(demo.SLAVE)
            assert wait_for(lambda: s.upkeeping, 30), s.note
            assert s.beat_ok  # (not before its beat is learned: released meanwhile, it is steered back by it)
            demo.pdh.drop(1.0)
            assert wait_for(lambda: s.note.startswith("upkeep stopped"), 5), s.note
            assert "its master's lock was lost" in s.note and not s.upkeeping
            assert wait_for(lambda: s.trim_level == math.inf, 2)
            assert wait_for(lambda: s.sls_guard.stage == "hold" and s.state == "on" and demo.pll.locked, 30), (
                s.sls_guard.stage, s.state, s.sls_note)
            assert "relocked: its PLL holds it again" in s.sls_note and s.runner is None
        finally:
            e.stop()
    finally:
        demo.close()
