"""SLS lock electronics (sls.py): the HTTP client against a simulated unit, what a poll says, the guard of a slave a
PLL beat-locks (its steps, then the whole night failure in the simulation: the master's lock drops, the PLL's manual
lock runs the slave away and stops it lasing, its TA switches itself off; guarded, none of that)."""
import json
import time

import pytest

from conftest import wait_for
from wmlock import sls, view
from wmlock.engine import Engine
from wmlock.meters import Meter
from wmlock.rpc import Api
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


def make_engine(demo, tmp_path, guard=True, hop=True, beat=80.0, **sls_kw):
    m, s = demo.MASTER, demo.SLAVE
    slave = {"name": "739 slave", "link": {"device": demo.sister.key, "laser": 1},
             "sls": {"kind": "pll", "host": "127.0.0.1", "port": demo.pll.sls.port, "master": m, "beat": beat,
                     "guard": guard, **sls_kw}}
    if hop:
        slave["hop"] = {"enabled": True, "i_nom": 150.0}
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
        assert cfg["kind"] == "pll" and cfg["master"] == lab.MASTER and cfg["beat"] == 80.0
        assert SlsParams.from_dict(cfg) == s.sls
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
