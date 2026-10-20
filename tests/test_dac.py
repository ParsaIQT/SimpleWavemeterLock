"""Lasers without a digital controller, on an analog output board (dac.py): the boards, and a simulated laser locked
through a simulated board, end to end."""
import sys
import time
import types
from enum import IntEnum

import pytest

from conftest import wait_for
from wmlock import dac
from wmlock.dlcpro import DecopError
from wmlock.engine import DAC_LOCK, Engine
from wmlock.lock import LockParams
from wmlock.modehop import HopParams
from wmlock.sim import Demo, SimWavemeter
from wmlock.wavemeter import C_NM_THZ


def test_keys_and_what_was_last_written():
    assert dac.key("mcc", 0) == "dac:mcc:0" and dac.parse("dac:mcc:0") == ("mcc", 0)
    assert dac.is_dac("dac:sim:1") and not dac.is_dac("192.168.1.20") and not dac.is_dac(None)
    assert dac.title("dac:mcc:0") == "MCC board 0"
    m = dac.Memory({"last": {"mcc:0/3": 0.512, "mcc:0/4": "x", "mcc:0/5": True}})
    assert m.get("dac:mcc:0", 3) == 0.512 and m.get("dac:mcc:0", 4) is None and m.get("dac:mcc:0", 5) is None
    assert not m.changed
    m.put("dac:mcc:0", 7, -0.25)
    assert m.changed and m.get("dac:mcc:0", 7) == -0.25
    assert m.to_config() == {"last": {"mcc:0/3": 0.512, "mcc:0/7": -0.25}} and not m.changed


def test_a_board_is_seen_as_a_dlc_pro_sees_its_offset():
    board = dac.SIM_BOARDS[90] = dac.SimBoard(90)
    try:
        m = dac.Memory()
        c = dac.DacClient("dac:sim:90", m)
        assert not c.is_open
        with pytest.raises(DecopError, match="its voltage is not known"):
            c.get_float("laser2:scan:offset")  # (never written: a DAC cannot be read back)
        assert c.is_open and c.get_str("laser2:scan:unit") == "V"
        assert c.set("laser2:scan:offset", 0.25) == 0
        assert board.volts[2] == 0.25 and c.get_float("laser2:scan:offset") == 0.25 and m.get("dac:sim:90", 2) == 0.25
        for bad in ("laser2:dl:cc:current-set", "system-label"):
            with pytest.raises(DecopError, match="not on an analog output"):
                c.get(bad)
        with pytest.raises(DecopError, match="has no output 16"):
            c.set("laser16:scan:offset", 0.1)
        assert m.get("dac:sim:90", 16) is None
        report = c.report(2, 3)
        assert report[0] == ("head", "Analog output") and "last set to 0.2500 V" in report[2][1]
        assert report[3][0] == "warn" and "not set by this program yet" in report[3][1]
        with pytest.raises(OSError, match="no simulated DAC 91"):
            dac.DacClient("dac:sim:91", m).set("laser0:scan:offset", 0.0)
    finally:
        dac.SIM_BOARDS.pop(90, None)


class FakeMcculw:
    """mcculw as far as dac.MccBoard uses it, with a USB-3106 as board 0 (its output 3 set to ±5 V in InstaCal)."""

    def __init__(self, monkeypatch, board=0):
        class ULRange(IntEnum):
            BIP5VOLTS = 0
            BIP10VOLTS = 1
            UNI10VOLTS = 100

        class InfoType(IntEnum):
            BOARDINFO = 2

        class BoardInfo(IntEnum):
            NUMDACHANS = 13
            DACRANGE = 114

        class ULError(Exception):
            def __init__(self, code):
                super().__init__(code)
                self.errorcode = code

            def __str__(self):
                return f"Error {self.errorcode}: bad"

        self.calls = []
        fake = self

        def get_board_name(n):
            if n != board:
                raise ULError(1)
            return "USB-3106"

        def get_config(info, n, dev, item):
            fake.calls.append(("get_config", n, dev, item))
            if item == BoardInfo.NUMDACHANS:
                return 16
            if dev == 3:
                return int(ULRange.BIP5VOLTS)
            raise ULError(35)  # (no range said for the others)

        def v_out(n, ch, rng, value, options=0):
            fake.calls.append(("v_out", n, ch, rng, value))
            if value > 9.0:
                raise ULError(42)

        ul = types.SimpleNamespace(ULError=ULError, get_board_name=get_board_name, get_config=get_config, v_out=v_out)
        enums = types.SimpleNamespace(ULRange=ULRange, InfoType=InfoType, BoardInfo=BoardInfo)
        pkg = types.ModuleType("mcculw")
        pkg.ul, pkg.enums = ul, enums
        monkeypatch.setitem(sys.modules, "mcculw", pkg)
        monkeypatch.setitem(sys.modules, "mcculw.ul", ul)
        monkeypatch.setitem(sys.modules, "mcculw.enums", enums)
        self.ULRange = ULRange


def test_an_mcc_board_through_mcculw(monkeypatch):
    fake = FakeMcculw(monkeypatch)
    b = dac.MccBoard(0)
    b.open()
    assert b.name == "USB-3106 (MCC board 0)" and b.channels == 16
    assert not [c for c in fake.calls if c[0] == "v_out"]  # (opening writes nothing: no output is jumped)
    b.write(3, 0.5)
    b.write(5, -0.25)
    writes = [c for c in fake.calls if c[0] == "v_out"]
    assert writes == [("v_out", 0, 3, fake.ULRange.BIP5VOLTS, 0.5), ("v_out", 0, 5, fake.ULRange.BIP10VOLTS, -0.25)]
    with pytest.raises(DecopError, match="output 3: Error 42"):
        b.write(3, 9.5)
    with pytest.raises(DecopError, match="has no output 16"):
        b.write(16, 0.0)
    assert "output 3 in the range BIP5VOLTS" in b.describe(3)
    with pytest.raises(OSError, match="MCC board 1: Error 1: bad .is it in InstaCal"):
        dac.MccBoard(1).open()


def test_without_mcculw_the_board_is_offline_and_says_why(monkeypatch):
    monkeypatch.setitem(sys.modules, "mcculw", None)  # (import mcculw: ImportError)
    with pytest.raises(OSError, match="pip install mcculw, and install InstaCal"):
        dac.MccBoard(0).open()


@pytest.fixture
def old():
    """The demo with a laser on a simulated analog output board: channel 8, output 2, its current on output 3."""
    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004, dac=True)
    yield demo
    demo.close()


def dac_engine(demo, cfg=None):
    e = Engine(cfg or {}, demo.make_source, discovery_targets=["127.0.0.1"])
    e.start()
    assert wait_for(lambda: Demo.DAC in e.channels and e.channels[Demo.DAC].freq > 0, 5)
    return e, e.channels[Demo.DAC]


def test_a_laser_on_an_analog_output_is_locked_through_it(old):
    board = old.board
    e, ch = dac_engine(old)
    try:
        e.link(Demo.DAC, old.dac_key, 2, ff_ao=3, ff_ratio=5.0)
        assert ch.lock.p == LockParams(**{**LockParams().to_dict(), **DAC_LOCK}) and not ch.hop.enabled
        e.set_setpoint(Demo.DAC, 935.17976)
        e.engage(Demo.DAC)  # its output was never set: not known, not jumped to
        assert wait_for(lambda: ch.state == "off" and ch.message, 3)
        assert ch.message == "cannot lock: output 2: its voltage is not known (a DAC cannot be read back): set it first"
        assert board.writes == []
        assert e.set_output(Demo.DAC, 0.25) == 0.25 and e.set_output(Demo.DAC, 0.0, ff=True) == 0.0
        assert wait_for(lambda: board.volts[2] == 0.25 and ch.offset == 0.25 and ch.message == "", 2)
        assert e.set_output(Demo.DAC, 5.0) == 1.2  # (within its lock's limits)
        assert wait_for(lambda: board.volts[2] == 1.2, 2)
        e.set_output(Demo.DAC, 0.25)
        assert wait_for(lambda: board.volts[2] == 0.25, 2)
        e.engage(Demo.DAC)
        assert wait_for(lambda: ch.state == "on", 3)
        assert e.set_output(Demo.DAC, 0.0) is None and ch.message == "set by hand: not while it is locked"
        assert wait_for(lambda: abs(ch.error_mhz) < 15.0, 20), ch.error_mhz  # (8 GHz/V; it was 400 MHz off)
        u, ff = board.volts[2], board.volts[3]
        assert u == pytest.approx(0.3, abs=0.01) and ff == pytest.approx((u - 0.25) / 5.0, abs=1e-6)
        assert e.where(ch) == "AO 2 (+ AO 3 fed forward) @ simulated DAC 0"
        assert e.to_config()["channels"]["8"]["link"] == {"device": old.dac_key, "ao": 2, "ff_ao": 3, "ff_ratio": 5.0}
        assert e.to_config()["dac"]["last"][old.dac_key[4:] + "/2"] == pytest.approx(u, abs=1e-5)

        f0 = C_NM_THZ / 935.17976
        for k in range(1, 9):  # the setpoint walked up a GHz at a time: at most ±0.8 V from where it locked (0.25 V)
            e.set_setpoint(Demo.DAC, C_NM_THZ / (f0 + k * 1e-3))
            if not wait_for(lambda: ch.state == "off" or abs(ch.error_mhz) < 100.0, 10):
                break
            if ch.state == "off":
                break
        assert ch.state == "off" and ch.message.startswith("unlocked: offset would leave -0.55 … 1.05")
        assert board.volts[2] <= 1.05 + 1e-9 and k >= 6
    finally:
        e.stop()


def test_an_output_is_put_back_after_a_restart(old):
    """The board reset (a power cut: 0 V), the program started again: each output is set back to what it was last
    set to, and the lock starts from there."""
    board = old.board
    e, ch = dac_engine(old)
    try:
        e.link(Demo.DAC, old.dac_key, 2)
        e.set_output(Demo.DAC, 0.3)
        assert wait_for(lambda: board.volts[2] == 0.3, 2)
        cfg = e.to_config()
    finally:
        e.stop()
    board.volts = [0.0] * board.channels
    board.writes.clear()
    e, ch = dac_engine(old, cfg)
    try:
        assert wait_for(lambda: board.volts[2] == 0.3, 3) and board.writes == [(2, 0.3)]
        e.set_setpoint(Demo.DAC, 935.17976)
        e.engage(Demo.DAC)
        assert wait_for(lambda: ch.state == "on", 3) and ch.lock.u0 == 0.3
    finally:
        e.stop()


def test_a_board_not_there_at_the_start_is_put_back_when_it_is(old):
    """Unplugged when the program starts: offline, and its outputs are set back as soon as it can be opened."""
    board = old.board
    del dac.SIM_BOARDS[board.number]
    e, ch = dac_engine(old, {"channels": {"8": {"link": {"device": old.dac_key, "ao": 2}}},
                             "dac": {"last": {old.dac_key[4:] + "/2": 0.3}}})
    try:
        c = e.controllers[old.dac_key]
        assert wait_for(lambda: c.online is False, 3) and "no simulated DAC" in c.error
        assert board.writes == []
        dac.SIM_BOARDS[board.number] = board  # (plugged in)
        assert wait_for(lambda: board.writes == [(2, 0.3)] and c.online, 5)
        assert e.where(ch) == "AO 2 @ simulated DAC 0"
    finally:
        e.stop()
        dac.SIM_BOARDS[board.number] = board


def test_a_write_the_board_refuses_unlocks_it(old):
    board = old.board
    e, ch = dac_engine(old, {"dac": {"last": {old.dac_key[4:] + "/2": 0.3}}})
    try:
        e.link(Demo.DAC, old.dac_key, 2)
        e.set_setpoint(Demo.DAC, 935.1797)
        e.engage(Demo.DAC)
        assert wait_for(lambda: ch.state == "on", 3)
        board.fail = DecopError("simulated DAC 0, output 2: Error 42: USB transfer failed")
        assert wait_for(lambda: ch.state == "off", 5)
        assert ch.message == "unlocked: simulated DAC 0, output 2: Error 42: USB transfer failed"
    finally:
        e.stop()


def test_mode_hop_recovery_is_not_for_an_analog_output(old):
    e, ch = dac_engine(old)
    try:
        e.link(Demo.DAC, old.dac_key, 2)
        e.set_hop_params(Demo.DAC, HopParams(enabled=True))
        assert not ch.hop.enabled
        e.recenter(Demo.DAC)
        assert ch.message == "recenter: not on an analog output (it needs a DLC pro)" and ch.state == "off"
        e.take_optimum(Demo.DAC)
        assert ch.message == "take the optimum: not on an analog output"
        e.check_setup(Demo.DAC)
        assert wait_for(lambda: ch.setup is not None, 3)
        assert ch.setup[0] == ("head", "Analog output") and "16 outputs" in ch.setup[1][1]
        assert any("not set by this program yet" in text for _, text in ch.setup)
    finally:
        e.stop()


def test_another_pc_locks_a_laser_on_an_analog_output(old):
    """Through share.py, as any other: Lock from another PC (Engine.remote)."""
    e, ch = dac_engine(old, {"dac": {"last": {old.dac_key[4:] + "/2": 0.3}}})
    try:
        e.link(Demo.DAC, old.dac_key, 2)
        e.set_setpoint(Demo.DAC, 935.17976)
        assert e.remote(Demo.DAC, "lock", {}, "Lab B") == (True, "")
        assert wait_for(lambda: ch.state == "on", 3) and ch.remote_note == "locked from Lab B"
        time.sleep(0.2)
        assert e.remote(Demo.DAC, "unlock", {}, "Lab B") == (True, "")
        assert ch.state == "off"
    finally:
        e.stop()
