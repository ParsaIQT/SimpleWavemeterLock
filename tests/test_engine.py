import math
import re
import time

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


def test_a_lock_that_runs_away_reverses_its_gains_and_locks(engine, demo):
    """The 866 laser tunes the other way: with positive gains, its lock pushes it away from the setpoint, faster and
    faster. Caught at it: back where it was nearest, its gains reversed (and kept), locked; the row says so."""
    engine.link(2, demo.dlcs[0].key, 2)
    engine.set_params(2, LockParams(ki=1.0, max_step=0.5, excursion=1.0))
    ch = engine.channels[2]
    engine.set_setpoint(2, detune(ch, 30))
    engine.dirty = False
    engine.engage(2)
    assert wait_for(lambda: ch.state == "on", 2)
    assert wait_for(lambda: ch.lock.reversed and abs(ch.error_mhz) < 5, 8), (ch.state, ch.message, ch.error_mhz)
    assert ch.state == "on" and ch.lock.p.ki == -1.0 and engine.dirty
    assert ch.to_config()["lock"]["ki"] == -1.0
    assert ch.lock_note.startswith("it ran away (") and "its gains are reversed (P 0, I -1)" in ch.lock_note
    assert 69.0 <= demo.dlcs[0].offset(2) <= 71.0  # never left the excursion window
    from wmlock import view
    assert "gains reversed by the lock: it ran away" in view.row(engine, 2, time.monotonic())["info"]
    assert "its lock, at " in view.tip(engine, 2)


def test_a_lock_that_runs_away_either_way_is_unlocked_and_put_back(engine, demo):
    """Linked to the wrong laser (the 397's piezo for the 866's channel), the lock moves something that does not move
    it, while it drifts away faster and faster: reversed once, it runs away all the same; unlocked, the offset it moved
    put back where the error was smallest, its gains as they were."""
    engine.link(2, demo.dlcs[0].key, 1)
    engine.set_params(2, LockParams(ki=1.0, max_step=0.5, excursion=2.0))
    ch = engine.channels[2]
    laser, t0 = demo.lasers[1], time.monotonic()
    real = laser.true_freq
    laser.true_freq = lambda now: real(now) + 3e-6 * math.exp((now - t0) / 0.7)  # (THz: away, faster and faster)
    engine.setpoint_to_current(2)
    v0 = demo.dlcs[0].offset(1)
    engine.engage(2)
    assert wait_for(lambda: ch.state == "off", 10), (ch.state, ch.error_mhz)
    assert ch.message.startswith("unlocked: it ran away with its gains either way"), ch.message
    assert ch.lock.p.ki == 1.0 and "the right laser, the right channel?" in ch.message
    assert wait_for(lambda: abs(demo.dlcs[0].offset(1) - v0) < 0.05, 2), demo.dlcs[0].offset(1)


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


def test_a_ta_shg_pro_whose_scan_drives_its_current_is_locked_by_its_piezo(engine, demo):
    """As the lab's second TA-SHG pro: its scan generator drives its current, so its scan offset is its current, 160 mA,
    and its piezo, at 41.3 V, is set by dl:pc:voltage-set. Taken for the piezo, 160 was outside 0 … 140 V and nothing
    would lock ("the offset, 160.00 V, is outside 0 … 140 V"): its piezo is locked instead, its current left alone."""
    dlc, ch = demo.dlcs[1], engine.channels[4]  # (the demo's 423 nm laser: its scan drives its current)
    dlc.params["laser2:scan:offset"], dlc.params["laser2:dl:pc:voltage-set"] = 160.0, 41.3
    t = time.monotonic()
    assert wait_for(lambda: ch.t > t, 2)  # (read as it is now)
    engine.link(4, dlc.key, 2)
    assert wait_for(lambda: ch.scan_out == 51 and ch.offset == pytest.approx(41.3), 2)  # shown: its piezo
    assert (ch.piezo, ch.cc, ch.unit) == ("dl:pc:voltage-set", "scan:offset", "V")
    engine.set_setpoint(4, detune(ch, 80))
    engine.engage(4)
    assert wait_for(lambda: ch.state == "on", 2), ch.message
    assert wait_for(lambda: abs(ch.error_mhz) < 5, 8)
    assert dlc.piezo(2) > 41.5 and dlc.written("laser2:dl:pc:voltage-set")  # tuning is positive: up
    assert dlc.written("laser2:scan:offset") == [] and dlc.current(2) == 160.0  # its current: never touched
    engine.disengage(4)
    engine.set_params(4, LockParams(v_min=0, v_max=40))
    engine.engage(4)
    assert wait_for(lambda: ch.state == "off" and ch.message, 2)
    assert ch.message.startswith("piezo voltage (dl:pc:voltage-set) 41.") and ch.message.endswith("outside 0 … 40")


def test_a_scan_offset_not_in_volts_is_never_taken_for_the_piezo(engine, demo):
    """A controller that does not say what its scan drives (scan:output-channel refused), its scan offset in mA: not
    locked (the lock would write piezo volts into its current), and it says why."""
    dlc, ch = demo.dlcs[1], engine.channels[4]
    dlc.busy_checks.append(lambda kind, name: name == "laser2:scan:output-channel")
    engine.link(4, dlc.key, 2)
    engine.setpoint_to_current(4)
    engine.engage(4)
    assert wait_for(lambda: ch.state == "off" and ch.message, 2)
    assert ch.message == ("cannot lock: its scan offset is 150.000 mA, not the piezo (see Scan › Output channel at the "
                          "DLC pro)")
    assert ch.scan_out is None and ch.piezo == "scan:offset" and dlc.written("laser2:scan:offset") == []


def test_its_scan_switched_to_its_current_while_locked_unlocks_it(engine, demo):
    """Switched at the DLC pro, by hand, while it is locked: what the lock writes, the scan offset, would set the
    current from then on. It is unlocked within a poll, and its piezo is dl:pc:voltage-set from then on."""
    dlc = demo.dlcs[0]
    engine.link(1, dlc.key, 1)
    ch = engine.channels[1]
    engine.setpoint_to_current(1)
    engine.engage(1)
    assert wait_for(lambda: ch.state == "on" and dlc.written("laser1:scan:offset"), 2)
    n = len(dlc.log)
    dlc.execute("(param-set! 'laser1:scan:output-channel 51)")
    assert wait_for(lambda: ch.state == "off", 2)
    assert ch.message == "unlocked: its scan now drives the current: the piezo is dl:pc:voltage-set"
    assert (ch.piezo, ch.cc) == ("dl:pc:voltage-set", "scan:offset")
    late = [v for name, v in dlc.log[n:] if name == "laser1:scan:offset"]
    assert len(late) <= 8  # (the lock's writes until the next poll: none after it)
    dlc.execute("(param-set! 'laser1:scan:offset 150.0)")  # (its current set back by hand)
    t = time.monotonic()
    assert wait_for(lambda: ch.t > t, 2)  # (read as it is now)
    engine.set_setpoint(1, detune(ch, 50))
    engine.engage(1)
    assert wait_for(lambda: ch.state == "on", 2), ch.message
    assert wait_for(lambda: abs(ch.error_mhz) < 5, 8)
    assert dlc.written("laser1:dl:pc:voltage-set") and dlc.current(1) == 150.0


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


def hop_engine(demo, n, key, laser, params, log_path=None, drop_path=None):
    from wmlock.modehop import HopParams

    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"], log_path=log_path, drop_path=drop_path)
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


def test_a_laser_whose_scan_drives_its_current_recovers_by_its_scan_offset(quiet_demo):
    """The demo's 423 nm laser, its scan on its current (as on the lab's TA-SHG pro): a recovery sets its current by
    the scan's offset (its current-set drives nothing there), its piezo by dl:pc:voltage-set; the setup check says
    which is which."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[1]
    e, ch = hop_engine(demo, 4, dlc.key, 2, dict(enabled=True, step=0.2))
    try:
        e.engage(4)
        assert wait_for(lambda: ch.state == "on" and ch.guard.armed and ch.hop.i_nom == 150.0, 5)
        demo.laser(4).hop_mode(+1)
        assert wait_for(lambda: ch.state == "recovering", 3)
        assert wait_for(lambda: ch.state == "on", 30), ch.message
        assert wait_for(lambda: abs(ch.error_mhz) < 3, 5)
        assert demo.laser(4).ecdl.m == 0 and dlc.current(2) > 151.0  # back on its mode, by the current
        assert_gentle([150.0] + dlc.written("laser2:scan:offset"), 148.0, 152.0)
        assert dlc.written("laser2:dl:cc:current-set") == [] and dlc.written("laser2:dl:pc:voltage-set")
        e.check_setup(4)
        assert wait_for(lambda: ch.setup is not None, 5)
        text = "\n".join(t for _, t in ch.setup)
        assert ("its scan drives the current (scan:output-channel 51), not the piezo: the lock moves the piezo's own "
                "set value, dl:pc:voltage-set, and recoveries the current by the scan's offset, now 15") in text
        assert "set 15" in text and "(the scan's offset: its scan drives the current)" in text
        assert not [lvl for lvl, _ in ch.setup if lvl == "error"], ch.setup
    finally:
        e.stop()


@pytest.mark.parametrize("reset", [3.0, 0.0], ids=["reset", "goes"])
def test_shg_laser_put_back_on_the_right_peak(quiet_demo, tmp_path, reset):
    """Its lock on the next peak: relocked, the current parked high on its mode first. Then the master hops: back on
    its right mode by a reset (as by hand: the current 3 mA up and down to the optimum's), or without it, caught by
    the goes up and down within its window."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, step=0.2,
                                                        reset_up=reset), log_path=tmp_path / "log.jsonl")
    try:
        e.engage(demo.SHG)  # Relock: watch it
        assert ch.state == "on" and not ch.lock.active  # watched, not locked to the wavemeter
        assert wait_for(lambda: ch.cavity_on is True and ch.guard.armed and ch.shg_txt == "locked", 5)
        shg.lock.jump(1)  # 'locked', one free spectral range off
        assert wait_for(lambda: ch.state == "recovering", 5)
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked() and "top end" in ch.note, 20), ch.message
        assert abs(ch.error_mhz) < 3 and not ch.message  # on its mode: its current parked high in it all the same
        assert "on the right mode: the current to the top end of where it is at its best" in ch.note, ch.note
        assert shg.lock.peak == pytest.approx(shg.lock.f_res, abs=1e-12) and shg.lock.shg_state()[0] == 5
        assert dlc.written("laser1:dl:lock:lock-enabled") == [False, True]
        assert ch.hop.lp_code == ord("t") and ch.hop.falc_sign == 1  # learned
        assert abs(shg.lock.dc()) < 0.002  # the FALC's input offset nulled the error signal's DC level

        assert wait_for(lambda: ch.guard.armed, 5)  # watching it again
        shg.hop_mode(-1)  # the master hops: its lock relocks 6 GHz away and calls that locked
        assert wait_for(lambda: ch.state == "recovering", 5)
        how = "back on its right mode by a reset" if reset else "caught"
        assert wait_for(lambda: ch.state == "on" and not ch.message and how in ch.note, 60), ch.message
        assert shg.lock.locked() and abs(ch.error_mhz) < 3 and shg.ecdl.m == 0  # back on its own mode
        assert shg.lock.shg_state()[0] == 5 and "of the optimum's" in ch.note
        (lo, hi), off = ch.last["i_window"], ch.last["act_offset"]  # measured: 148.58 ... 151.58 mA
        assert lo == pytest.approx(148.58) and hi == pytest.approx(151.58) and ch.last["i_top"] == pytest.approx(154.58)
        top = 149.58 + reset if reset else hi  # (the reset: up to 3 mA above the optimum's, 149.58 mA)
        assert_gentle(currents_written(dlc, 1), lo - off - 1e-3, top - off + 1e-3)  # set (off: as the piezo was)
        assert max(currents_written(dlc, 1)) == pytest.approx(top - off, abs=0.01)
        assert ch.last["ok"] and ch.last["reset" if reset else "catch"] and ch.last["scan"]["x"]  # kept for a look...
        assert ch.last["ref"][0] == pytest.approx(70.0, abs=0.5)  # (where it was on target: its right mode there)
        lines = (tmp_path / "log.jsonl").read_text().splitlines()
        logged = "relocked, back on its right mode by a reset" if reset else "relocked, caught the right mode at"
        assert len(lines) == 2 and logged in lines[1]  # ...and logged
    finally:
        e.stop()


def test_step_by_step_through_the_engine(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, confirm=True))
    try:
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.cavity_on is True and ch.guard.armed, 5)
        shg.lock.jump(-1)
        assert wait_for(lambda: ch.hop_prompt.startswith("Take the laser over"), 5)
        assert dlc.written("laser1:dl:lock:lock-enabled") == []  # nothing done before you say so
        for _ in range(12):
            if not wait_for(lambda: ch.hop_prompt or ch.state != "recovering", 10) or ch.state != "recovering":
                break
            asked = ch.hop_prompt
            e.confirm_step(demo.SHG)
            assert wait_for(lambda a=asked: ch.hop_prompt != a or ch.state != "recovering", 2)  # (or the next)
        assert wait_for(lambda: ch.state == "on", 10) and shg.lock.locked() and not ch.message, ch.message
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
        assert "watched with the DLC pro's recorder" in text
        assert [x for x in dlc.log[before:] if ":recorder:" not in x[0]] == []  # nothing written (by the check: its
        #                                                                       dropout monitor uses the recorder)
        assert "after a hop, first a reset, as by hand" in text and "current goes down 0.001 mA at a time" in text
        assert "upkeep:" not in text
        from wmlock.modehop import HopParams

        e.set_hop_params(demo.SHG, HopParams.from_dict({**ch.hop.to_dict(), "upkeep": 30.0}))
        ch.setup = None
        e.check_setup(demo.SHG)
        assert wait_for(lambda: ch.setup is not None, 5)
        text = "\n".join(t for _, t in ch.setup)
        assert "upkeep: every 30 min, locked and steady (both locks left on), the current up 0.005 mA at a time" in text
        assert "The upkeep changes nothing but the current, 0.005 mA a step" in text
    finally:
        e.stop()


def test_now_and_then_the_upkeep_keeps_a_locked_shg_laser_just_below_the_top_of_its_mode(quiet_demo):
    """HopParams.upkeep: every so many minutes a laser its DLC pro locks, with an SHG stage, watched and locked, has
    the top of its mode found (the current up until its SHG signal locked dips, both locks left on) and is kept
    just below it. It stays locked and watched all along, and says so on its row and in its last recovery."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, upkeep=0.02))
    try:
        las = shg.ecdl

        def to_second_mode():  # mA up to where a second mode begins
            v, i = shg._vi()
            return (las.zone(1.0)[0] - las.misalignment(las.m, v + shg.lock.dv, i)) / (las.ki - las.ksi)

        las.nudge(-(to_second_mode() - 0.25) * (las.ki - las.ksi))  # (0.25 mA below it)
        shg.lock.frequency()
        assert shg.lock.locked() and to_second_mode() == pytest.approx(0.25, abs=0.01)
        before, i_set = len(dlc.log), dlc.params["laser1:dl:cc:current-set"]
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.note.startswith("upkeep"), 40), (ch.note, ch.message)
        assert re.match(r"upkeep: the top of its mode at 149\.8\d\d mA, the current 149\.580 → 149\.\d\d\d mA "
                        r"\(\+0\.2\d\d\), \d+ s$", ch.note), ch.note
        assert wait_for(lambda: ch.state == "on", 5) and not ch.message
        assert shg.lock.locked() and shg.lock.shg_state() == (5, "locked") and 0.005 < to_second_mode() < 0.06
        assert ch.last["why"] == "upkeep" and ch.last["ok"] and ch.last["upkeep"]["top"] > ch.last["upkeep"]["to"]
        writes = [(n, v) for n, v in dlc.log[before:] if ":recorder:" not in n]
        assert {n for n, _ in writes} == {"laser1:dl:cc:current-set"}  # (both locks left alone)
        currents = [i_set] + [v for _, v in writes]
        assert max(abs(a - b) for a, b in zip(currents, currents[1:])) <= 0.005 + 1e-9
    finally:
        e.stop()


def test_an_upkeep_that_goes_wrong_puts_the_current_back_and_it_is_watched_again(quiet_demo):
    """Its SHG signal drops out for a moment while the upkeep climbs: the current goes back where it was, a little at
    a time, and the laser is watched again (its optimum, cleared meanwhile, is left for the next Relock click)."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, upkeep=0.02))
    try:
        name = "laser1:nlo:pd:shg-int:photodiode"
        real, reads = dlc.params[name], [0]

        def intra():  # (gone for a few readings, in the first upkeep)
            if ch.upkeeping:
                reads[0] += 1
                if 20 <= reads[0] < 26:
                    return 0.05
            return real()

        dlc.params[name] = intra
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.state == "on" and ch.hop.optimum, 5)  # (taken by the first Relock click)
        ch.hop.opt_i = 0.0  # (cleared)
        i_set = dlc.params["laser1:dl:cc:current-set"]
        assert wait_for(lambda: ch.note.startswith("upkeep: its SHG signal fell to 0.050 V"), 40), (ch.note, ch.message)
        assert ch.note.endswith(": the current put back"), ch.note
        assert wait_for(lambda: ch.state == "on", 5) and not ch.message and shg.lock.locked()
        assert ch.last["why"] == "upkeep" and not ch.last["ok"] and not ch.hop.optimum
        written = dlc.written("laser1:dl:cc:current-set")
        assert written and written[-1] == pytest.approx(i_set)
        currents = [i_set] + written
        assert max(abs(a - b) for a, b in zip(currents, currents[1:])) <= 0.005 + 1e-9
    finally:
        e.stop()


def test_stopping_a_recovery_puts_current_and_offset_back(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[0]
    e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, step=0.1))
    try:
        e.engage(1)
        assert wait_for(lambda: ch.state == "on" and ch.guard.armed, 5)
        armed = dlc.offset(1)
        demo.laser(1).hop_mode(+1)
        assert wait_for(lambda: "mapping" in ch.hop_stage and dlc.current(1) < 149.5, 10)
        offset = ch.runner.rec.V0  # where the recovery took it over (the lock may have moved it a hair since it armed)
        assert offset == pytest.approx(armed, abs=0.01)
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


def test_a_recovery_brings_a_current_outside_its_window_into_it_first(quiet_demo):
    """150 mA, its window 144 … 146 mA: rather than refusing, a recovery brings it into the window first, 0.2 mA a
    step, and goes on from there (here it finds nothing on target there, and puts it back where it began)."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[0]
    e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, i_nom=145.0, i_window=1.0, step=0.2))
    try:
        e.recenter(1)
        assert ch.state == "recovering"
        assert wait_for(lambda: dlc.current(1) <= 146.0 + 1e-9, 20), ch.message
        currents = [150.0] + dlc.written("laser1:dl:cc:current-set")
        assert all(abs(a - b) <= 0.2 + 1e-9 for a, b in zip(currents, currents[1:]))
        assert wait_for(lambda: ch.state != "recovering", 90)
        first = "the current was 150.00 mA, above its window (144.00 … 146.00 mA): down into it"
        assert ch.last["notes"][0][1] == first
        if ch.message:  # (nothing on target within 144 … 146 mA: back where it began)
            assert dlc.current(1) == pytest.approx(150.0)
    finally:
        e.stop()


def test_tuning_its_current_follows_its_piezo_within_its_window_and_only_then():
    """Engine.Follow, a DL pro's feedforward of -0.6 mA/V that adds nothing: held on its setpoint, its current is left
    alone; tuned, its set current follows its piezo, kept within its window, from where a hand set it; once there, it
    is left where it is. What its feedforward does seen late in a tuning, it catches up, 0.01 mA a step."""
    from wmlock.engine import Follow

    f = Follow(-0.6, 70.0, 65.0, 65.0, 64.4, 67.4, 200.0, ff=0.0, now=0.0)
    assert f.k == -0.6 and f.step(71.0, 1.0) is None  # held: its piezo makes up for its cavity's drift
    f.tune(71.0, 2.0)
    assert f.step(71.5, 3.0) == pytest.approx(64.7)  # tuned: -0.6 mA/V from where it began
    assert f.step(71.5005, 3.1) is None  # (less than LEAST from the last)
    assert f.step(73.0, 4.0) == pytest.approx(64.4) and f.edge  # held at the bottom of its window
    assert f.step(71.0, 5.0) == pytest.approx(65.0) and not f.edge
    assert f.mine(64.7) and f.mine(64.4) and f.mine(65.0) and not f.mine(65.3)  # its own writes, polled
    f.by_hand(65.3, 71.0, 6.0)  # set by hand: followed from there
    assert f.step(72.0, 7.0) == pytest.approx(64.7)
    assert not any(f.tuned(3.0, 8.0 + k) for k in range(Follow.SETTLE_N - 1))
    assert f.tuned(3.0, 12.0) and not f.tuning and f.step(70.0, 13.0) is None  # there: left where it is
    assert f.total == pytest.approx(-0.6)  # (what its writes moved it: a hand's not counted)

    g = Follow(-0.6, 70.0, 65.0, 65.0, 64.4, 67.4, 200.0, now=0.0)  # what its feedforward does: not known yet
    g.tune(70.0, 0.0, act=65.0)
    assert g.step(70.5, 1.0) is None  # (nothing to go by: not moved)
    g.seen(0.0, 70.5, 65.0)  # it adds nothing: the 0.3 mA it should have moved since the tuning began, caught up
    out = [x for x in (g.step(70.5, 2.0 + k) for k in range(40)) if x is not None]
    assert len(out) == 30 and out[0] == pytest.approx(64.99) and out[-1] == pytest.approx(64.7)
    assert all(a - b == pytest.approx(0.01) for a, b in zip(out, out[1:]))
    assert not g.tuned(3.0, 50.0) and g.tuned(500.0, 61.0)  # (TUNE_S at most: then held, wherever it is)


def ff_adds_nothing(dlc, laser, factor=-0.6):
    """Its DLC pro's feedforward on, at `factor`, adding nothing as the scan's offset moves its piezo (as the lab's
    DL pros' do, their scan off)."""
    dlc.params[f"laser{laser}:dl:cc:feedforward-enabled"] = True
    dlc.params[f"laser{laser}:dl:cc:feedforward-factor"] = factor
    dlc.ff_from[laser] = None


def test_the_lock_moves_the_current_along_with_the_piezo_as_it_tunes_it_not_as_it_holds_it(quiet_demo):
    """Its feedforward (-0.6 mA/V) adds nothing as the scan's offset moves its piezo: held on its setpoint against its
    cavity's drift, its current is left alone (what its feedforward does is seen meanwhile, and kept); tuned to a new
    setpoint, its current follows its piezo by the factor and is left where it is once it is there, its own writes
    never taken for a hand's; tuned again after a hand set it, from where the hand left it."""
    from wmlock import view

    demo, dlc = quiet_demo, quiet_demo.dlcs[0]
    ff_adds_nothing(dlc, 1)
    e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, step=0.2))
    laser = demo.laser(1)
    written = lambda: dlc.written("laser1:dl:cc:current-set")  # noqa: E731
    try:
        e.engage(1)
        assert wait_for(lambda: ch.state == "on" and ch.guard.armed and ch.follow is not None, 5)
        assert ch.follow.k is None and "is seen once the wavemeter lock has moved that 0.17 V" in view.notes(e, 1)
        for _ in range(60):  # its cavity drifts 300 MHz: held
            laser.walk += 5.0
            time.sleep(0.1)
        assert wait_for(lambda: abs(ch.error_mhz) < 3, 5) and dlc.offset(1) < 69.5
        assert written() == [] and ch.hop.ff_set == -0.6 and ch.hop.ff_act == 0.0  # left alone; its feedforward seen
        resets = []
        reset = ch.guard.reset
        ch.guard.reset = lambda: (resets.append(1), reset())[1]
        v0, i0 = dlc.offset(1), dlc.current(1)
        e.set_setpoint(1, detune(ch, 400))
        assert ch.follow.tuning and view.follow_text(ch) == ", following its piezo"
        assert wait_for(lambda: not ch.follow.tuning, 20) and abs(ch.error_mhz) < 10
        dv, di = dlc.offset(1) - v0, dlc.current(1) - i0
        assert dv > 0.6 and di == pytest.approx(-0.6 * dv, abs=0.02), (dv, di)
        assert view.follow_text(ch) == "" and "moved -0." in view.notes(e, 1)
        n = len(written())
        for _ in range(40):  # drifts back 200 MHz: held, its current left where it is
            laser.walk -= 5.0
            time.sleep(0.1)
        assert wait_for(lambda: abs(ch.error_mhz) < 3, 5)
        assert len(written()) == n and ch.guard.armed and ch.state == "on" and len(resets) == 1  # (the setpoint's)
        dlc.params["laser1:dl:cc:current-set"] = 149.8  # by hand
        assert wait_for(lambda: ch.current_set == 149.8 and ch.follow.at == 149.8, 3)
        v0 = dlc.offset(1)
        e.set_setpoint(1, detune(ch, -250))
        assert wait_for(lambda: not ch.follow.tuning, 20)
        dv, di = dlc.offset(1) - v0, dlc.current(1) - 149.8
        assert dv < -0.5 and di == pytest.approx(-0.6 * dv, abs=0.02), (dv, di)
        cfg = e.to_config()["channels"]["1"]["hop"]
        assert cfg["ff_set"] == -0.6 and cfg["ff_act"] == 0.0  # (kept: its recoveries go by it too)
        e.check_setup(1)
        assert wait_for(lambda: ch.setup is not None, 5)
        text = "\n".join(t for _, t in ch.setup)
        assert ("as the wavemeter lock tunes it to a new setpoint, it moves the current along with the piezo by what "
                "the feedforward does not (-0.6000 mA/V), within its recovery's window; held on its setpoint, it "
                "leaves the current alone") in text
    finally:
        e.stop()


def test_a_feedforward_that_moves_the_current_itself_is_left_to_it(quiet_demo):
    """Its DLC pro's feedforward moves its current as its factor says: seen so, the lock adds nothing (tuned, its DLC
    pro moves its current itself)."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[0]
    dlc.params["laser1:dl:cc:feedforward-enabled"] = True
    dlc.params["laser1:dl:cc:feedforward-factor"] = -0.6
    dlc.ff_from[1] = dlc.offset(1)  # (it adds -0.6 mA a volt from here)
    e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, step=0.2))
    try:
        e.engage(1)
        assert wait_for(lambda: ch.state == "on" and ch.guard.armed and ch.follow is not None, 5)
        e.set_setpoint(1, detune(ch, 400))
        assert wait_for(lambda: ch.follow is None and abs(ch.error_mhz) < 10, 20)
        assert "moves its current as its factor says (-0.600 mA/V) as its piezo moves: nothing to add" in ch.follow_note
        assert dlc.written("laser1:dl:cc:current-set") == [] and ch.hop.ff_act == -0.6
    finally:
        e.stop()


def test_the_lock_tunes_it_with_what_its_recoveries_saw_its_feedforward_do(quiet_demo):
    """What its feedforward does, as its recoveries saw it (HopParams.ff_set, ff_act), at the factor it is set to:
    tuned from the first, its current follows its piezo at once, and no further than its window (144.5 mA, measured,
    at the bottom), held at its edge."""
    from wmlock import view

    demo, dlc = quiet_demo, quiet_demo.dlcs[0]
    ff_adds_nothing(dlc, 1)
    e, ch = hop_engine(demo, 1, dlc.key, 1, dict(enabled=True, step=0.2, i_nom=150.0, i_below=0.5, ff_set=-0.6,
                                                 ff_act=0.0))
    try:
        e.engage(1)
        assert wait_for(lambda: ch.state == "on" and ch.guard.armed and ch.follow is not None, 5)
        assert ch.follow.k == -0.6
        e.set_setpoint(1, detune(ch, 600))
        assert wait_for(lambda: ch.follow.edge, 10) and view.follow_text(ch) == ", held at its window's edge"
        assert wait_for(lambda: not ch.follow.tuning, 30) and abs(ch.error_mhz) < 10
        assert min(dlc.written("laser1:dl:cc:current-set")) == pytest.approx(149.5)  # (no further down)
    finally:
        e.stop()


def test_relock_brings_a_laser_left_high_and_off_target_back(quiet_demo):
    """The morning after: the SHG laser left 3 mA above nominal (where it lased again, say), its cavity lock off, off
    target. Relock: down into its window slowly, watching that it lases, on down until single-mode, walked onto the
    target, relocked."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    try:
        assert wait_for(lambda: ch.hop.i_nom > 0 and ch.hop.shg_lock, 10)
        dlc.execute("(param-set! 'laser1:dl:lock:lock-enabled #f)")
        i0 = dlc.current(1)
        for k in range(1, 16):
            dlc.execute(f"(param-set! 'laser1:dl:cc:current-set {i0 + 0.2 * k:.3f})")
        assert wait_for(lambda: abs(ch.error_mhz) > 1000 and ch.cavity_on is False, 5), ch.error_mhz
        e.engage(demo.SHG)  # Relock
        assert ch.state == "recovering" and ch.hop_why == "cavity lock off"
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked(), 120), ch.message
        assert abs(ch.error_mhz) < 3 and ch.current <= ch.hop.i_nom + ch.hop.i_window + 1e-6
        assert any("above its window" in text for _, text in ch.last["notes"]), ch.last["notes"]
    finally:
        e.stop()


def test_relock_only_watches_a_laser_its_dlc_pro_locks(quiet_demo):
    """For a laser its DLC pro locks to a cavity the button is Relock: it watches the laser, and never
    moves the piezo to the wavemeter (that is the cavity lock's job); after a hop it relocks it."""
    import time

    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, step=0.2))
    try:
        assert ch.relock
        e.set_setpoint(demo.SHG, C_NM_THZ / (ch.freq + 20e-6))  # 20 MHz off: a wavemeter lock would pull
        e.engage(demo.SHG)
        assert ch.state == "on" and not ch.lock.active and not ch.message
        time.sleep(1.0)
        assert dlc.written("laser1:scan:offset") == []  # watched, not locked to the wavemeter
        assert wait_for(lambda: ch.guard.armed, 5)
        e.toggle(demo.SHG)  # click again: stop watching
        assert ch.state == "off" and not ch.guard.armed
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.guard.armed, 5)
        shg.lock.jump(1)
        assert wait_for(lambda: ch.state == "recovering", 5)
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked() and bool(ch.note), 30), ch.message
        assert shg.lock.peak == pytest.approx(shg.lock.f_res, abs=1e-12)
    finally:
        e.stop()


def test_relock_relocks_a_laser_found_unlocked(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, step=0.2))
    try:
        dlc.execute("(param-set! 'laser1:dl:lock:lock-enabled #f)")  # turned off by hand, say
        assert wait_for(lambda: ch.cavity_on is False, 5) and not shg.lock.locked()
        e.engage(demo.SHG)  # Relock
        assert ch.state == "recovering" and ch.hop_why == "cavity lock off"
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked(), 60), ch.message
        assert abs(ch.error_mhz) < 3 and "relocked" in ch.note and shg.lock.shg_state()[0] == 5
        assert wait_for(lambda: ch.guard.armed, 5)  # and watched from now on
    finally:
        e.stop()


def test_relock_needs_recovery_enabled(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=False, cavity=True, shg=True))
    try:
        e.engage(demo.SHG)
        assert ch.state == "off" and "mode-hop recovery is off" in ch.message
        assert dlc.written("laser1:scan:offset") == []
    finally:
        e.stop()


def test_nominal_current_is_the_measured_one(quiet_demo):
    """The SHG laser's measured current is 0.42 mA below its set current: the nominal current, its
    window and what is shown are measured; the recovery writes set currents to match."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, step=0.2))
    try:
        assert wait_for(lambda: ch.hop.i_nom and ch.current is not None, 5)
        assert ch.hop.i_nom == pytest.approx(149.58, abs=1e-3) and ch.current == pytest.approx(149.58, abs=1e-3)
        assert ch.current_set == pytest.approx(150.0)
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.guard.armed, 5)
        shg.hop_mode(+1)
        assert wait_for(lambda: ch.state == "recovering", 5)
        assert wait_for(lambda: ch.state == "on" and "walked" in ch.note, 60), ch.message
        top = 153.0 if ch.last.get("reset") else 152.0  # (the reset: up to 3 mA above the optimum's, 149.58 mA)
        assert_gentle(currents_written(dlc, 1), 149.0, top)  # set: the measured 148.58 ... 151.58 (152.58)
        assert "mA measured" in ch.note and ch.last["i_window"] == [148.58, 151.58]
        assert ch.last["act_offset"] == pytest.approx(-0.42, abs=0.01)
    finally:
        e.stop()


def test_a_laser_that_stops_lasing_is_brought_back_at_once(quiet_demo):
    """Its TA's seed power says it stopped lasing: recovered at once, whatever the hourly limit; the
    current jumps up, then comes down slowly, and the laser is relocked as after a hop."""
    import time

    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, max_per_hour=1))
    try:
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.guard.armed and ch.seed_name == "amp:pd:seed:power" and ch.hop.seed_ok, 5)
        assert ch.lasing is True and ch.hop.seed_ok == pytest.approx(20.0, abs=0.5)  # learned, lasing on target
        ch.guard.recent.append(time.monotonic())  # this hour's one recovery: used up
        shg.stop_lasing()
        assert wait_for(lambda: ch.lasing is False, 3)
        assert wait_for(lambda: ch.state == "recovering", 3) and ch.hop_why.startswith("stopped lasing (seed 0.")
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked(), 90), ch.message
        assert shg.ecdl.lasing and shg.lock.ta.on() and abs(ch.error_mhz) < 3 and ch.lasing is True
        assert ch.note.startswith("stopped lasing (seed 0.") and "lasing again" in ch.note, ch.note
        currents = currents_written(dlc, 1)
        assert max(currents) >= 150.0 + 2.4 and max(currents) <= 150.0 + 5.0 + 1e-9  # (set: nominal 149.58 measured)
        assert_gentle(currents, 149.0, 155.0)
    finally:
        e.stop()


def test_relock_takes_over_a_laser_locked_on_the_wrong_peak(quiet_demo):
    """Locked, but 60 MHz from its target: the wavemeter is never that far off, so it is on another peak
    (further than peak_tol, 50 MHz, though well inside the 100 MHz a hop of a wavemeter-locked laser takes):
    relocked at once when Relock is pressed, and when it jumps there while watched."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    try:
        assert ch.hop.tolerance == 100.0 and ch.hop.peak_tol == 50.0
        with shg.lock.mutex:
            shg.lock.peak += 60e-6  # held 60 MHz off
        assert wait_for(lambda: abs(ch.error_mhz - 60) < 5, 3)
        e.engage(demo.SHG)  # Relock
        assert ch.state == "recovering" and re.fullmatch(r"off by \+(5[5-9]|6[0-5]) MHz", ch.hop_why), ch.hop_why
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked(), 60), ch.message
        assert abs(ch.error_mhz) < 3 and shg.lock.peak == pytest.approx(shg.lock.f_res, abs=1e-12)
        assert wait_for(lambda: ch.guard.armed, 5)
        with shg.lock.mutex:
            shg.lock.peak -= 60e-6
        assert wait_for(lambda: ch.state == "recovering", 6), ch.message
        assert re.fullmatch(r"locked -[56]\d MHz off: the wrong peak", ch.hop_why), ch.hop_why
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked(), 60), ch.message
        assert abs(ch.error_mhz) < 3
    finally:
        e.stop()


def test_a_cavity_lock_that_keeps_losing_it_is_taken_over(quiet_demo):
    """The master goes multimode (a third of its power in the next mode), quietly for the wavemeter: its cavity
    lock keeps losing it and relocking, its transmission (Fine In 1, its lock detection's input) far below its level
    locked on average: taken over, the mode made single, relocked; the levels locked, of the transmission and the
    SHG signal, were learned while it ran."""
    import time

    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    e.LEVEL_S = 4.0
    try:
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.guard.armed and ch.trans_name == "io:fine-1:value-act" and ch.hop.trans_locked
                        and ch.hop.shg_lock, 10)
        assert ch.hop.trans_locked == pytest.approx(0.8, abs=0.02) and ch.hop.shg_lock == pytest.approx(1.23, abs=0.02)
        assert wait_for(lambda: e.trans_levels(ch)[0] > 0.75 and len(ch.trans_hist) >= 5, 5)
        shg.go_multimode(1)
        assert wait_for(lambda: ch.state == "recovering", 15), ch.message
        assert re.fullmatch(r"the cavity lock keeps losing it: transmission 0\.[0-3]\d\d V on average over 4 s "
                            r"\(locked 0\.[78]\d\d V, zero 0\.000 V\)", ch.hop_why), ch.hop_why  # (not a hop: the
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked(), 90), ch.message  # wavemeter saw it close)
        relocks = shg.lock.relocks
        time.sleep(3)
        assert shg.lock.relocks == relocks and shg.lock.locked() and e.trans_levels(ch)[0] > 0.75  # it holds
        assert ("single mode" in ch.note or "of the optimum's" in ch.note) and not shg.lock._slips()
        assert ch.hop.trans_zero == pytest.approx(0.0, abs=0.003)  # measured while the master scanned
    finally:
        e.stop()


def test_the_transmission_is_watched_from_its_zero_to_its_level_locked(quiet_demo):
    """Its transmission as on a real one: 0.05 V locked, on a DC offset of 0.01 V, and its level locked learned
    wrong before (four times too high). Relock is not fooled: the level locked is the median of what it reads locked
    on target (whether watched or not); a lock turned off and on again by hand is no lock that keeps losing it;
    one that does is relocked, its zero measured while the master scans, and watched against both, in volts."""
    import time

    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    with shg.lock.mutex:
        shg.lock.t00, shg.lock.t_dc = 0.05, 0.01
        x, _, kind = shg.lock.lockpoint
        shg.lock.lockpoint = (x, 0.06, kind)  # (chosen by hand on that peak)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, trans_locked=0.24))
    e.LEVEL_S = 4.0
    try:
        assert wait_for(lambda: ch.hop.trans_locked < 0.1, 10)  # learned afresh, not yet watched
        assert ch.hop.trans_locked == pytest.approx(0.06, abs=0.003) and ch.trans_name == "io:fine-1:value-act"
        assert wait_for(lambda: len(ch.trans_hist) >= 5 and ch.trans_hist[-1][0] - ch.trans_hist[0][0] > 3.5, 5)
        e.engage(demo.SHG)  # Relock: watched, nothing to relock
        assert ch.state == "on" and not ch.hop_why
        now, usual, zero, limit = e.trans_levels(ch)
        assert now == pytest.approx(0.06, abs=0.003) and zero == 0.0 and limit == pytest.approx(0.03, abs=0.002)
        dlc.execute("(param-set! 'laser1:dl:lock:lock-enabled #f)")  # by hand, for a while (its transmission ~0.01 V)
        time.sleep(5)
        assert ch.state == "on" and not ch.trans_hist and not ch.guard.armed
        dlc.execute("(param-set! 'laser1:dl:lock:lock-enabled #t)")  # and on again: locked at once
        time.sleep(6)
        assert ch.state == "on" and not ch.hop_why and shg.lock.locked()  # no relock
        shg.go_multimode(1)
        assert wait_for(lambda: ch.state == "recovering", 15), ch.message
        assert re.fullmatch(r"the cavity lock keeps losing it: transmission 0\.0[1-3]\d\d V on average over 4 s "
                            r"\(locked 0\.0[56]\d\d V, zero 0\.0000 V\)", ch.hop_why), ch.hop_why
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked(), 90), ch.message
        assert ch.hop.trans_zero == pytest.approx(0.01, abs=0.002)  # measured while the master scanned
        assert wait_for(lambda: len(ch.trans_hist) >= 5, 5)
        now, usual, zero, limit = e.trans_levels(ch)
        assert zero == ch.hop.trans_zero and usual == pytest.approx(0.06, abs=0.003)
        assert limit == pytest.approx(0.035, abs=0.002) and now > limit  # half way from 0.01 to 0.06 V
        cfg = e.to_config()["channels"][str(demo.SHG)]["hop"]
        assert cfg["trans_zero"] == ch.hop.trans_zero and cfg["trans_locked"] == ch.hop.trans_locked
    finally:
        e.stop()


def test_relock_recenters_the_target_on_the_lock(quiet_demo):
    """The setpoint is 20 MHz off where the wavemeter reads the laser locked on the right peak (it has drifted, say):
    after a relock the target is moved onto the lock, in the wavemeter's numbers."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    try:
        e.set_setpoint(demo.SHG, C_NM_THZ / (ch.freq + 20e-6))  # 20 MHz above the lock: inside the 50 MHz
        assert wait_for(lambda: abs(ch.error_mhz + 20) < 3, 3)
        e.engage(demo.SHG)  # Relock: watched (20 MHz is not the wrong peak)
        assert ch.state == "on" and wait_for(lambda: ch.guard.armed, 5)
        shg.lock.jump(1)
        assert wait_for(lambda: ch.state == "recovering", 5)
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked() and bool(ch.note), 60), ch.message
        assert shg.lock.peak == pytest.approx(shg.lock.f_res, abs=1e-12)  # the right peak...
        assert abs(ch.error_mhz) < 3 and ch.guard.track == 0.0  # ...and the target moved onto it
        assert re.search(r"target re-centered on the lock, -(19|20|21)\.\d MHz", ch.note), ch.note
        assert e.to_config()["channels"][str(demo.SHG)]["setpoint_nm"] == pytest.approx(ch.setpoint_nm)
    finally:
        e.stop()


def test_a_laser_that_stops_lasing_unseen_by_its_seed_is_brought_back(quiet_demo):
    """It stops lasing but still sends its TA most of its power, broadband: the seed power cannot tell. Its SHG
    signal (and cavity transmission) is gone, though: recovered at once, the hourly limit notwithstanding (the
    wavemeter has nothing to read either); the SHG sweep is flat, so the current jumps up until it shows
    resonances again, comes down slowly, and the laser is relocked."""
    import time

    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, max_per_hour=1))
    e.LEVEL_S = 3.0
    try:
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.guard.armed and ch.hop.seed_ok and ch.hop.shg_lock and ch.hop.trans_locked, 10)
        ch.guard.recent.append(time.monotonic())  # this hour's one recovery: used up
        shg.stop_lasing(ase=0.7)
        assert wait_for(lambda: ch.state == "recovering", 10), ch.message
        assert ch.lasing is not False  # (its seed power says it lases)
        assert ch.hop_why.startswith(("the SHG signal is gone", "the cavity lock keeps losing it")), ch.hop_why
        assert wait_for(lambda: ch.state == "on" and shg.lock.locked(), 90), ch.message
        assert shg.ecdl.lasing and shg.lock.ta.on() and abs(ch.error_mhz) < 3
        assert "lasing again" in ch.note and max(currents_written(dlc, 1)) >= 150.0 + 2.0, ch.note
    finally:
        e.stop()


# ------------------------------------------- a wavemeter channel switched off
def test_a_channel_switched_off_keeps_its_relock_watching_and_its_lock_held(quiet_demo):
    """Someone unticks channels in the WLM software (for a faster rate on theirs): a laser its DLC pro locks stays
    watched (by what its DLC pro says), one locked to the wavemeter is held, both still shown; nothing is touched
    while all is well. Ticked again, the wavemeter lock goes on."""
    import time

    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    try:
        e.link(1, demo.dlcs[0].key, 1)
        e.setpoint_to_current(1)
        e.engage(1)
        e.engage(demo.SHG)
        c1, src = e.channels[1], e.source
        assert wait_for(lambda: c1.state == "on" and ch.guard.armed and ch.cavity_on and ch.shg_txt == "locked", 5)
        src.untick(1)
        src.untick(demo.SHG)
        assert wait_for(lambda: ch.off and c1.off, 2)
        assert ch.state == c1.state == "on" and not ch.message and not c1.message  # watched, and held
        assert math.isnan(ch.freq) and ch.status == "channel off" and e.rows() == [1, 2, 3, 4, 5, 6, 7]
        time.sleep(2)
        assert src.switched == [] and ch.state == "on" and not ch.look_why  # all is well: left alone
        src.tick(1)
        assert wait_for(lambda: not c1.off and c1.freq > 0, 2) and c1.state == "on"
        assert wait_for(lambda: abs(c1.error_mhz) < 5, 5) and c1.lock.active  # locked again
        e.disengage(demo.SHG)
        src.untick(2)
        assert wait_for(lambda: e.channels[2].off, 2)
        assert e.rows() == [1, 3, 4, 5, 6, 7]  # switched off: shown if locked, watched, or Relock could watch it
    finally:
        e.stop()


def test_a_relock_with_its_channel_off_looks_when_its_dlc_pro_says_so_and_relocks(quiet_demo):
    """Its channel switched off, the master hops: its DLC pro's lock searches, then catches a peak 6 GHz off and
    says locked. That it searched was seen: its channel is switched on for a look, the laser relocked onto its
    target, and the channel switched off again; switched off by someone meanwhile, it is switched on again."""
    import time

    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    shg.lock.hop_search = 1.5  # (the DLC pro's ReLock takes that long to catch a resonance again)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, step=0.2))
    src = e.source
    try:
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.guard.armed and ch.cavity_on and ch.shg_txt == "locked", 5)
        target = ch.f_set
        src.untick(demo.SHG)
        assert wait_for(lambda: ch.off, 2)
        shg.hop_mode(-1)
        assert wait_for(lambda: ch.state == "recovering", 15), ch.looked
        assert re.fullmatch(r"its (cavity|SHG cavity's) lock said “relocking”, then jumped -[56]\.\d{3} GHz",
                            ch.hop_why), ch.hop_why
        assert src.switched == [(demo.SHG, True)] and src.use[demo.SHG]
        assert wait_for(lambda: ch.runner is not None and ch.runner.stage.startswith(("walking", "the piezo")), 30)
        src.untick(demo.SHG)  # someone unticks it again, mid-recovery
        assert wait_for(lambda: src.use[demo.SHG] and len(src.switched) == 2, 3)  # on again at once
        assert wait_for(lambda: ch.state != "recovering", 90)
        assert ch.state == "on" and not ch.message and "relocked, caught the right mode" in ch.note, ch.message
        time.sleep(0.3)
        assert src.switched == [(demo.SHG, True), (demo.SHG, True), (demo.SHG, False)] and not src.use[demo.SHG]
        assert ch.off and shg.lock.locked() and abs(shg.true_freq(time.monotonic()) - target) * 1e6 < 5
    finally:
        e.stop()


def test_a_look_gives_its_dlc_pro_the_time_to_relock_by_itself(quiet_demo):
    """Its channel off, its lock is knocked out: its DLC pro says relocking, a look. While its ReLock searches
    (the laser anywhere) its readings do not count; it catches the same peak: locked on target, left alone."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    src = e.source
    try:
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.guard.armed and ch.cavity_on and ch.shg_txt == "locked", 5)
        src.untick(demo.SHG)
        assert wait_for(lambda: ch.off, 2)
        shg.lock.knock(2.5)
        assert wait_for(lambda: ch.look_why == "its cavity lock said “relocking”", 3), ch.looked
        assert wait_for(lambda: not ch.look_why, 15)
        assert ch.looked.startswith("its cavity lock said “relocking”: locked on target"), ch.looked
        assert ch.state == "on" and ch.last is None and wait_for(lambda: ch.off, 2)  # no recovery
        assert src.switched == [(demo.SHG, True), (demo.SHG, False)]
    finally:
        e.stop()


def test_relock_clicked_with_its_channel_off_has_a_look(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    src = e.source
    try:
        assert wait_for(lambda: ch.cavity_on and ch.shg_txt == "locked", 5)
        src.untick(demo.SHG)
        assert wait_for(lambda: ch.off, 2)
        e.engage(demo.SHG)  # Relock: is it locked on target? A look, its channel switched on for it
        assert ch.state == "on" and ch.look_why == "Relock clicked" and ch.borrowed
        assert wait_for(lambda: not ch.look_why, 10)
        assert ch.looked.startswith("Relock clicked: locked on target") and ch.state == "on", ch.looked
        assert wait_for(lambda: ch.off, 2) and src.switched == [(demo.SHG, True), (demo.SHG, False)]
    finally:
        e.stop()


def test_a_laser_that_stops_lasing_with_its_channel_off_is_jumped_at_once(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    src = e.source
    try:
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.guard.armed and ch.hop.seed_ok, 5)
        src.untick(demo.SHG)
        assert wait_for(lambda: ch.off, 2)
        shg.stop_lasing(0.0)
        assert wait_for(lambda: ch.state == "recovering", 5) and ch.hop_why.startswith("stopped lasing")
        assert wait_for(lambda: ch.runner is not None, 2)  # at once: the current jump does not wait for readings
        assert wait_for(lambda: ch.state != "recovering", 120)
        assert ch.state == "on" and shg.ecdl.lasing and shg.lock.locked() and "lasing again" in ch.note, ch.message
        assert wait_for(lambda: ch.off, 2) and src.switched == [(demo.SHG, True), (demo.SHG, False)]
    finally:
        e.stop()


def test_no_relock_when_its_channel_cannot_be_switched_on(quiet_demo):
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    src = e.source
    src._use = lambda n, on, record=True: "SetSwitcherSignalStates: WLM busy" if on else ""
    try:
        assert wait_for(lambda: ch.cavity_on, 5)
        src.untick(demo.SHG)
        dlc.params["laser1:dl:lock:lock-enabled"] = False  # its cavity lock off: Relock relocks it
        assert wait_for(lambda: ch.off and ch.cavity_on is False, 2)
        e.engage(demo.SHG)
        assert ch.state == "recovering"
        assert wait_for(lambda: ch.state == "off", 5)
        assert ch.message == ("cavity lock off: no recovery, cannot switch channel 7 on on the wavemeter: "
                              "SetSwitcherSignalStates: WLM busy")
        assert dlc.written("laser1:dl:cc:current-set") == [] and not ch.borrowed  # nothing touched
    finally:
        e.stop()


def near_modes(demo):
    """The demo's SHG master made like the 739 nm one: its modes 2.2 GHz (4 pm) apart, going multimode near their
    edges; locked by hand on target, as before."""
    shg = demo.laser(demo.SHG)
    e = shg.ecdl
    e.fsr, e.hysteresis, e.multimode = 2.2, 0.3, 0.4
    e.m, e.shift = 0, 0.0
    shg.lock.settle()
    return shg


def test_relock_takes_the_optimum_and_a_hop_to_the_near_mode_is_caught_back(quiet_demo):
    """Clicked on a laser set up by hand, Relock takes its optimum (current, piezo voltage, SHG signal locked), the
    first time only. It hops to the next mode up (739.053 → 739.049 nm): caught back on its own mode by the current,
    the piezo free to go to the optimum's ± 30 V for that."""
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = near_modes(demo)
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True))
    try:
        assert wait_for(lambda: ch.cavity_on is True and ch.shg_txt == "locked" and ch.current is not None, 5)
        e.engage(demo.SHG)  # Relock
        assert ch.state == "on" and wait_for(lambda: ch.hop.optimum, 5)
        hop = ch.hop
        assert hop.opt_i == pytest.approx(dlc.current_act(1), abs=0.01) and hop.i_nom == hop.opt_i
        assert hop.opt_v == pytest.approx(dlc.offset(1)) and hop.opt_shg == pytest.approx(1.23, abs=0.03)
        assert ch.note.startswith("optimum taken: ") and hop.opt_t
        e.toggle(demo.SHG)  # stop watching, and click Relock again: not taken again
        hop.opt_t = "kept"
        e.engage(demo.SHG)
        assert wait_for(lambda: ch.guard.armed and ch.fine_at is not None, 10) and hop.opt_t == "kept"

        shg.hop_mode(+1)
        assert wait_for(lambda: ch.state == "recovering", 5)
        assert wait_for(lambda: ch.state == "on" and "caught the right mode" in ch.note, 60), ch.message
        assert shg.ecdl.m == 0 and shg.lock.locked() and abs(ch.error_mhz) < 3 and not ch.message
        lo, hi = ch.last["v_window"]
        assert lo <= hop.opt_v - 30.0 + 1e-6 and hi >= hop.opt_v + 30.0 - 1e-6  # room for its tries from there
        assert ch.last["ref"][0] == pytest.approx(hop.opt_v, abs=0.2) and ch.last["optimum"][0] == hop.opt_i
        assert ch.last["catch"][-1]["caught"]

        assert wait_for(lambda: ch.guard.armed, 5)
        e.take_optimum(demo.SHG)  # by hand, as it is now
        assert wait_for(lambda: hop.opt_t != "kept", 5) and ch.note.startswith("optimum taken: ")
    finally:
        e.stop()


# ------------------------------------------------------------------ laser maps
def test_a_laser_map_through_the_engine_records_everything_and_puts_the_laser_back(engine, demo, tmp_path):
    from wmlock import view
    from wmlock.lasermap import MapPlan, load

    engine.link(3, demo.dlcs[1].key, 1)
    for n in (1, 2, 4, 5, 6, 7):  # (the switch on its laser alone: a reading every exposure)
        engine.source.untick(n)
    ch, dlc = engine.channels[3], demo.dlcs[1]
    i0, v0 = dlc.current(1), dlc.offset(1)
    assert engine.start_map(3, MapPlan()).startswith("give the highest current it may have (mA, measured)")
    plan = MapPlan(i_lo=145.0, i_hi=152.0, i_step=1.0, v_lo=68.0, v_hi=72.0, v_step=2.0, reads=1, settle=0.0,
                   patterns="all", repeat=False, folder=str(tmp_path))
    s, size = engine.map_estimate(3, plan)
    assert 0 < s < 600 and size > 0
    assert engine.start_map(3, plan) == ""
    assert ch.state == "mapping" and engine.start_map(3, plan) == "it is being mapped already"
    assert wait_for(lambda: ch.mapper is not None and ch.mapper.mapper.points > 3, 10)
    v = view.row(engine, 3, time.monotonic())
    assert v["state"] == "mapping" and v["info"].startswith("laser map · ") and "points" in v["info"]
    assert engine.source._detail == {3: True}
    engine.recenter(3)
    assert ch.state == "mapping"  # (nothing else touches it meanwhile)
    assert wait_for(lambda: ch.state == "off", 60), ch.mapper.stage if ch.mapper else ch.map_note
    assert ch.map_note.startswith("map done in ") and "saved to" in ch.map_note and ch.message == ""
    assert engine.source._detail == {}
    assert dlc.current(1) == pytest.approx(i0) and dlc.offset(1) == pytest.approx(v0)
    assert ch.to_config()["map"]["i_hi"] == 152.0  # (the plan kept, for the next)
    path = ch.map_last.mapper.out.path
    assert wait_for(lambda: not ch.map_last.mapper.out._thread.is_alive(), 10)
    with load(path) as f:
        legs = [(leg, len(pts)) for _, leg, pts in f.legs()]  # (first down from where it was, 150 mA; then each line
        assert legs == [(0, 6)] + [(1, 8), (2, 7)] * 3  # up, and back down from below its top)
        assert {ln["axis"] for ln in f.lines} == {"current"}
        assert all(q["f"] is not None for q in f.points)
        assert f.meta["laser"]["channel"] == 3 and f.meta["end"]["ok"] and f.meta["wavemeter"]["model"]
        n = f.db.execute("SELECT COUNT(*) FROM patterns WHERE kind = 0").fetchone()[0]
        assert n == len(f.points)
        assert f.meta["end"]["pattern_info"]["0"][0] > 0


def test_a_laser_map_stopped_puts_the_laser_back_and_keeps_what_it_had(engine, demo, tmp_path):
    from wmlock.lasermap import MapPlan, load

    engine.link(3, demo.dlcs[1].key, 1)
    for n in (1, 2, 4, 5, 6, 7):
        engine.source.untick(n)
    ch, dlc = engine.channels[3], demo.dlcs[1]
    i0, v0 = dlc.current(1), dlc.offset(1)
    plan = MapPlan(i_lo=140.0, i_hi=155.0, i_step=0.5, v_lo=60.0, v_hi=80.0, v_step=1.0, reads=2, patterns="off",
                   folder=str(tmp_path))
    assert engine.start_map(3, plan) == ""
    assert wait_for(lambda: ch.mapper is not None and ch.mapper.mapper.points > 10, 10)
    engine.stop_map(3)
    assert wait_for(lambda: ch.state == "off", 10)
    assert ch.message.startswith("map stopped after") and "everything put back" in ch.message
    assert dlc.current(1) == pytest.approx(i0) and dlc.offset(1) == pytest.approx(v0)
    out = ch.map_last.mapper.out
    assert wait_for(lambda: not out._thread.is_alive(), 10)
    with load(out.path) as f:
        assert len(f.points) >= 10 and f.meta["end"]["error"] == "stopped" and f.meta["end"]["restored"]


def test_a_laser_it_cannot_map_is_not_mapped(engine, demo, tmp_path):
    from wmlock.lasermap import MapPlan

    plan = MapPlan(i_lo=145.0, i_hi=152.0, v_lo=68.0, v_hi=72.0, folder=str(tmp_path))
    assert engine.start_map(3, plan) == "link a laser first"
    engine.link(3, demo.dlcs[1].key, 1)
    ch = engine.channels[3]
    engine.setpoint_to_current(3)
    engine.engage(3)
    assert wait_for(lambda: ch.state == "on", 2)
    assert engine.start_map(3, plan) == "unlock it first (or stop watching it)"
    engine.disengage(3)
    engine.link(7, demo.dlcs[2].key, 1)  # the TA-SHG pro: its TA is on
    assert engine.start_map(7, plan) == ""
    ch7 = engine.channels[7]
    assert wait_for(lambda: ch7.state == "off", 10)
    assert "its TA is on" in ch7.message and not list(tmp_path.glob("*.sqlite-journal"))
