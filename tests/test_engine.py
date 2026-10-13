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
        assert {d.serial: sorted(d.lasers) for d in devs} == {"SIM-000123": [1, 2], "SIM-000456": [1, 2],
                                                              "SIM-000789": [1]}
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


# ----------------------------------------------------------- mode-hop recovery
@pytest.fixture
def quiet_demo():
    from wmlock.sim import Demo, SimWavemeter

    d = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    for laser in d.lasers:
        laser.drift = 0.0
    yield d
    d.close()


def hop_engine(demo, n, key, laser, params, log_path=None):
    from wmlock.modehop import HopParams

    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"], log_path=log_path)
    e.start()
    assert wait_for(lambda: n in e.channels and e.channels[n].freq > 0, 5)
    e.link(n, key, laser)
    e.setpoint_to_current(n)
    e.set_hop_params(n, HopParams(**params))
    return e, e.channels[n]


def currents_written(dlc, laser):
    return [150.0] + dlc.written(f"laser{laser}:dl:cc:current-set")


def assert_gentle(currents, lo, hi):
    assert all(lo - 1e-9 <= c <= hi + 1e-9 for c in currents)
    assert all(abs(a - b) <= 0.2 + 1e-9 for a, b in zip(currents, currents[1:]))


def test_wavemeter_locked_laser_recovers_from_a_mode_hop(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[0]
    e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, step=0.2))
    try:
        e.engage(1)
        assert wait_for(lambda: ch.state == "on" and ch.guard.armed and ch.hop.i_nom == 150.0, 5)
        demo.laser(1).hop_mode(+1)  # +6 GHz: a naive lock would chase it with the piezo
        assert wait_for(lambda: ch.state == "recovering", 3)
        assert wait_for(lambda: ch.state == "on", 30), ch.message
        assert wait_for(lambda: abs(ch.error_mhz) < 3, 5)
        assert demo.laser(1).ecdl.m == 0 and dlc.current(1) > 151.0  # back on its mode, by the current
        assert "recentered" in ch.note and ch.message == ""
        assert ch.hop.kv == pytest.approx(350, rel=0.1) and ch.hop.spacing == pytest.approx(6000, rel=0.05)
        assert_gentle(currents_written(dlc, 1), 148.0, 152.0)
        cfg = e.to_config()["channels"]["1"]["hop"]
        assert cfg["enabled"] and cfg["kv"] == ch.hop.kv  # what it learned is kept
    finally:
        e.stop()


def test_shg_laser_put_back_on_the_right_peak(quiet_demo, tmp_path):
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, step=0.2),
                       log_path=tmp_path / "log.jsonl")
    try:
        assert wait_for(lambda: ch.cavity_on is True and ch.guard.armed and ch.shg_txt == "locked", 5)
        shg.lock.jump(1)  # 'locked', one free spectral range off
        assert wait_for(lambda: ch.state == "recovering", 5)
        assert wait_for(lambda: ch.state == "off" and shg.lock.locked(), 20), ch.message
        assert abs(ch.error_mhz) < 3 and "laser itself was fine" in ch.note and not ch.message
        assert shg.lock.peak == pytest.approx(shg.lock.f_res, abs=1e-12) and shg.lock.shg_state()[0] == 5
        assert dlc.written("laser1:dl:lock:lock-enabled") == [False, True]
        assert ch.hop.lp_code == ord("t") and ch.hop.falc_sign == 1  # learned
        assert abs(shg.lock.dc()) < 0.002  # the FALC's input offset nulled the error signal's DC level

        assert wait_for(lambda: ch.guard.armed, 5)
        shg.hop_mode(-1)  # the master hops: its lock relocks 6 GHz away and calls that locked
        assert wait_for(lambda: ch.state == "recovering", 5)
        assert wait_for(lambda: ch.state == "off" and not ch.message and "recentered" in ch.note, 60), ch.message
        assert shg.lock.locked() and abs(ch.error_mhz) < 3 and shg.ecdl.m == 0
        assert shg.lock.shg_state()[0] == 5 and "single mode" in ch.note
        assert_gentle(currents_written(dlc, 1), 148.0, 152.0)
        assert ch.last["ok"] and ch.last["maps"] and ch.last["scan"]["x"]  # kept for a look...
        lines = (tmp_path / "log.jsonl").read_text().splitlines()
        assert len(lines) == 2 and "relocked, recentered" in lines[1]  # ...and logged
    finally:
        e.stop()


def test_step_by_step_through_the_engine(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, confirm=True))
    try:
        assert wait_for(lambda: ch.cavity_on is True and ch.guard.armed, 5)
        shg.lock.jump(-1)
        assert wait_for(lambda: ch.hop_prompt.startswith("Take the laser over"), 5)
        assert dlc.written("laser1:dl:lock:lock-enabled") == []  # nothing done before you say so
        for _ in range(12):
            if not wait_for(lambda: ch.hop_prompt or ch.state == "off", 10) or ch.state == "off":
                break
            e.confirm_step(demo.SHG)
            assert wait_for(lambda: not ch.hop_prompt or ch.state == "off", 2)
        assert wait_for(lambda: ch.state == "off", 10) and shg.lock.locked() and not ch.message, ch.message
    finally:
        e.stop()


def test_setup_check_reads_only(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    try:
        before = len(dlc.log)
        e.check_setup(demo.SHG)
        assert wait_for(lambda: ch.setup is not None, 5)
        text = "\n".join(t for _, t in ch.setup)
        assert "relock by lockpoint" in text and "falc1" in text and "SHG cavity lock on" in text
        assert not [lvl for lvl, _ in ch.setup if lvl == "error"], ch.setup
        assert len(dlc.log) == before  # nothing written
    finally:
        e.stop()


def test_stopping_a_recovery_puts_current_and_offset_back(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[0]
    e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, step=0.1))
    try:
        e.engage(1)
        assert wait_for(lambda: ch.state == "on" and ch.guard.armed, 5)
        offset = dlc.offset(1)
        demo.laser(1).hop_mode(+1)
        assert wait_for(lambda: "mapping" in ch.hop_stage and dlc.current(1) < 149.5, 10)
        e.toggle(1)  # the button, while recentering: stop
        assert wait_for(lambda: ch.state == "off", 5)
        assert ch.message == "recovery stopped (current and offset put back)"
        assert dlc.current(1) == 150.0 and dlc.offset(1) == pytest.approx(offset, abs=1e-6)
        assert_gentle(currents_written(dlc, 1), 148.0, 152.0)
    finally:
        e.stop()


def test_recoveries_are_rate_limited(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[0]
    e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, max_per_hour=1, step=0.2))
    try:
        e.engage(1)
        assert wait_for(lambda: ch.state == "on" and ch.guard.armed, 5)
        demo.laser(1).hop_mode(+1)
        assert wait_for(lambda: ch.state == "recovering", 3)
        assert wait_for(lambda: ch.state == "on" and ch.guard.armed, 30), ch.message
        demo.laser(1).hop_mode(+1)
        assert wait_for(lambda: ch.state == "off", 10)
        assert "1 recoveries within the hour" in ch.message and dlc.current(1) > 151.0  # left alone
    finally:
        e.stop()


def test_recovery_refuses_a_current_outside_its_window(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[0]
    e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, i_nom=145.0, i_window=1.0))
    try:
        e.recenter(1)
        assert wait_for(lambda: ch.state == "off" and ch.message, 5)
        assert "outside 144.00 … 146.00 mA" in ch.message and dlc.written("laser1:dl:cc:current-set") == []
    finally:
        e.stop()
