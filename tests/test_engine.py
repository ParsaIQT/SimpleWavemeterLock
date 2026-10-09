import math

import pytest

from conftest import wait_for
from wmlock.engine import Engine, round_sig
from wmlock.lock import LockParams
from wmlock.sim import FakeDiscovery
from wmlock.wavemeter import C_NM_THZ


@pytest.fixture
def engine(demo):
    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    e.start()
    assert wait_for(lambda: all(e.channels.get(n) and e.channels[n].freq > 0 for n in (1, 2, 3, 4)))
    yield e
    e.stop()


def detune(ch, mhz):
    """Setpoint `mhz` above the present frequency."""
    return C_NM_THZ / (ch.freq + mhz * 1e-6)


def test_lock_pulls_in_and_holds(engine, demo):
    engine.link(1, demo.dlcs[0].key, 1)
    ch = engine.channels[1]
    engine.set_setpoint(1, detune(ch, 80))
    engine.engage(1)
    assert wait_for(lambda: ch.state == "on", 2)
    assert wait_for(lambda: abs(ch.error_mhz) < 5, 4)
    assert demo.dlcs[0].offset(1) > 70.1  # tuning is positive: offset went up
    engine.disengage(1)
    assert ch.state == "off" and not ch.lock.active
    # once unlocked, the displayed offset is the device's own value (read back)
    assert wait_for(lambda: abs(ch.offset - demo.dlcs[0].offset(1)) < 1e-6, 2)


def test_wrong_sign_unlocks_itself(engine, demo):
    engine.link(2, demo.dlcs[0].key, 2)  # the 866 laser tunes the other way
    engine.set_params(2, LockParams(ki=1.0, max_step=0.5, excursion=1.0))
    ch = engine.channels[2]
    engine.set_setpoint(2, detune(ch, 30))
    engine.engage(2)
    assert wait_for(lambda: ch.state == "on", 2)
    assert wait_for(lambda: ch.state == "off", 5)
    assert ch.message.startswith("unlocked")
    assert 69.0 <= demo.dlcs[0].offset(2) <= 71.0  # never left the excursion window


def test_lost_controller_unlocks(engine, demo):
    engine.link(3, demo.dlcs[1].key, 1)
    ch = engine.channels[3]
    engine.setpoint_to_current(3)
    engine.engage(3)
    assert wait_for(lambda: ch.state == "on", 2)
    demo.dlcs[1].close()
    assert wait_for(lambda: ch.state == "off", 4)
    assert ch.message.startswith("unlocked")
    assert engine.controllers[demo.dlcs[1].key].online is False


def test_engage_needs_a_link_and_uses_reading_as_setpoint(engine, demo):
    engine.engage(4)
    assert engine.channels[4].state == "off" and "link" in engine.channels[4].message
    engine.link(4, demo.dlcs[1].key, 2)
    assert engine.channels[4].message == ""
    engine.engage(4)
    ch = engine.channels[4]
    assert ch.setpoint_nm == pytest.approx(C_NM_THZ / ch.freq, abs=1e-5)
    assert wait_for(lambda: ch.state == "on", 2)
    engine.set_setpoint(4, None)
    assert ch.state == "off" and "setpoint" in ch.message


def test_engage_refuses_offset_outside_limits(engine, demo):
    engine.link(1, demo.dlcs[0].key, 1)
    engine.set_params(1, LockParams(v_min=0, v_max=50))
    engine.setpoint_to_current(1)
    engine.engage(1)
    ch = engine.channels[1]
    assert wait_for(lambda: ch.state == "off" and ch.message, 2)
    assert "outside" in ch.message


def test_a_laser_is_linked_to_one_channel_only(engine, demo):
    key = demo.dlcs[0].key
    engine.link(1, key, 1)
    engine.link(2, key, 1)
    assert engine.channels[1].device is None and "moved" in engine.channels[1].message
    assert engine.channels[2].device == key
    assert wait_for(lambda: engine.channels[2].offset == 70.0, 2)  # read back for display
    assert engine.channels[2].unit == "V"


def test_config_roundtrip(engine, demo):
    engine.link(1, demo.dlcs[0].key, 2)
    engine.set_setpoint(1, 396.959125)
    engine.rename(1, "397 cooling")
    engine.set_params(1, LockParams(ki=0.7))
    engine.set_hidden(5, True)
    cfg = engine.to_config()
    assert not engine.dirty
    again = Engine(cfg, demo.make_source)
    ch = again.channels[1]
    assert (ch.device, ch.laser, ch.name, ch.lock.p.ki) == (demo.dlcs[0].key, 2, "397 cooling", 0.7)
    assert ch.setpoint_nm == 396.959125 and again.channels[5].hidden
    assert demo.dlcs[0].key in again.known and demo.dlcs[0].key in again.devices


def test_discovery_lists_lasers(demo, monkeypatch):
    import wmlock.engine as eng_mod

    responder = FakeDiscovery(demo.dlcs, port=0)
    real = eng_mod.discover
    monkeypatch.setattr(eng_mod, "discover", lambda timeout, targets: real(0.3, targets, port=responder.port))
    try:
        e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
        e.start_discovery()
        assert e.scanning
        assert wait_for(lambda: not e.scanning, 3)
        devs = e.device_list()
        assert {d.serial: sorted(d.lasers) for d in devs} == {"SIM-000123": [1, 2], "SIM-000456": [1, 2]}
        assert all(d.probed and not d.error for d in devs)
    finally:
        responder.close()


def test_round_sig():
    assert round_sig(396.95912534) == 396.959125
    assert round_sig(1550.123456789) == 1550.12346
    assert round_sig(0.0) == 0.0 and math.isnan(round_sig(float("nan")))


def test_channel_dropped_by_the_switch_unlocks(engine, demo):
    engine.link(1, demo.dlcs[0].key, 1)
    engine.setpoint_to_current(1)
    engine.engage(1)
    assert wait_for(lambda: engine.channels[1].state == "on", 2)
    engine.on_active([2, 3, 4, 5])  # as if 'Use' was unticked in the WLM software
    assert engine.channels[1].state == "off" and "no longer measured" in engine.channels[1].message


def test_reference_holds_locked_lasers_true_frequency():
    """End to end: the locked 397 laser's TRUE frequency (known to the simulator) stays put while
    the wavemeter drifts and the reference laser mode-hops; without the reference it would not."""
    import time

    from wmlock.reference import TRACKING, RefParams
    from wmlock.sim import Demo, SimWavemeter

    eps = [0.0]
    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.05, drift=lambda now: eps[0]), exposure=0.005)
    laser = demo.laser(1)
    laser.drift = 0.0  # no free-running walk: what moves it is the lock alone
    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    e.start()

    def true_mhz():
        return laser.true_freq(time.monotonic()) * 1e6

    try:
        assert wait_for(lambda: all(n in e.channels and e.channels[n].freq > 0 for n in (1, 6)))
        e.set_reference(6)
        e.set_reference_params(RefParams(tau=3, max_drift=600, max_jump=50))  # a quick test wavemeter
        assert wait_for(lambda: e.reference.state == TRACKING, 15)
        e.link(1, demo.dlcs[0].key, 1)
        e.setpoint_to_current(1)
        e.engage(1)
        ch = e.channels[1]
        assert wait_for(lambda: ch.state == "on" and abs(ch.error_mhz) < 1, 5)
        start = true_mhz()

        raw0 = ch.raw
        for i in range(1, 41):  # the wavemeter starts reading 4e-8 high: +30 MHz at 755 THz
            eps[0] = 1e-9 * i
            time.sleep(0.1)
        time.sleep(3)
        assert (ch.raw - raw0) * 1e6 == pytest.approx(30, abs=4)  # it reads differently...
        assert abs(true_mhz() - start) < 3  # ...but the laser has not moved

        demo.laser(6).mode_hop(1500, 3)  # the reference hops to another cavity mode for 3 s
        time.sleep(2)
        assert e.reference.state != TRACKING
        assert abs(true_mhz() - start) < 3  # a naive correction would have moved it by 2.8 GHz
        assert wait_for(lambda: e.reference.state == TRACKING, 15)
        assert "recovered" in e.reference.message
        assert abs(true_mhz() - start) < 3

        e.clear_reference()  # control: now the lock follows the wavemeter's error
        time.sleep(2)
        assert true_mhz() - start == pytest.approx(-30, abs=4)
    finally:
        e.stop()
        demo.close()


def test_reference_survives_restart():
    from wmlock.reference import HOLD, TRACKING
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.05), exposure=0.005)
    try:
        e = Engine({"reference": {"channel": 6}}, demo.make_source, discovery_targets=["127.0.0.1"])
        e.start()
        try:
            assert wait_for(lambda: e.reference.state == TRACKING, 20)
            cfg = e.to_config()
        finally:
            e.stop()
        assert cfg["reference"]["channel"] == 6 and "anchor_thz" in cfg["reference"]
        again = Engine(cfg, demo.make_source, discovery_targets=["127.0.0.1"])
        assert again.reference.state == HOLD and again.reference.anchor == cfg["reference"]["anchor_thz"]
        assert again.reference.active  # the saved correction applies straight away
        again.start()
        try:
            assert wait_for(lambda: again.reference.state == TRACKING, 20)
            assert "recovered" in again.reference.message
        finally:
            again.stop()
    finally:
        demo.close()


def test_reference_channel_cannot_be_wavemeter_locked(engine, demo):
    engine.link(6, demo.dlcs[0].key, 1)
    engine.set_reference(6)
    engine.engage(6)
    assert engine.channels[6].state == "off"
    engine.link(1, demo.dlcs[0].key, 2)
    engine.setpoint_to_current(1)
    engine.engage(1)
    assert wait_for(lambda: engine.channels[1].state == "on", 2)
    engine.set_reference(1)  # taking a locked channel as the reference unlocks it
    assert engine.channels[1].state == "off" and engine.reference.channel == 1
