import socket
import time

import pytest

from wmlock.dlcpro import DLCPro, DecopError, Device, discover, encode, probe
from wmlock.sim import FakeDiscovery, FakeDLCPro

LASERS = {1: {"product": "DL pro 397", "label": "cooling"}, 2: {"product": "DL pro 866"}}


@pytest.fixture(params=[("\r\n", False), ("\n", False), ("\r\n", True)], ids=["crlf", "lf", "echo"])
def fake(request):
    eol, echo = request.param
    f = FakeDLCPro("SN-1", "lab A", LASERS, eol=eol, echo=echo)
    yield f
    f.close()


def test_get_set_roundtrip(fake):
    c = DLCPro("127.0.0.1", fake.port)
    assert c.get_float("laser1:scan:offset") == 70.0
    assert c.get_str("laser1:scan:unit") == "V"
    assert c.get_str("system-label") == "lab A"
    assert c.set("laser1:scan:offset", 71.25) == 0
    assert fake.offset(1) == 71.25
    assert c.get_float("laser1:scan:offset") == 71.25
    c.close()


def test_errors_keep_the_connection_usable(fake):
    c = DLCPro("127.0.0.1", fake.port)
    with pytest.raises(DecopError):
        c.get("laser3:product-name")
    with pytest.raises(DecopError):
        c.set("system-label", 1.0)
    with pytest.raises(DecopError):
        c.get_float("system-label")
    assert c.is_open
    assert c.get_float("laser2:scan:offset") == 70.0


def test_reconnects_after_the_device_drops_the_connection(fake):
    c = DLCPro("127.0.0.1", fake.port, timeout=1)
    c.get("system-type")
    fake.drop_connections()
    time.sleep(0.05)
    with pytest.raises(OSError):
        c.get("system-type")
    assert not c.is_open
    assert c.get_str("system-type") == "DLCpro"


def test_unreachable_device_fails_fast():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    t = time.monotonic()
    with pytest.raises(OSError):
        DLCPro("127.0.0.1", port, timeout=0.5).get("system-type")
    assert time.monotonic() - t < 1.0


def test_encode():
    assert encode(True) == "#t" and encode(False) == "#f"
    assert encode(3) == "3"
    assert encode(70.123456789) == "70.123457"
    assert encode(1e-7) == "0.000000"
    assert encode('a"b') == '"ab"'


def test_discovery_and_probe():
    a = FakeDLCPro("SN-1", "lab A", LASERS)
    b = FakeDLCPro("SN-2", "", {1: {"product": "DL pro 854"}})
    responder = FakeDiscovery([a, b], port=0)
    try:
        found = discover(timeout=0.3, targets=["127.0.0.1"], port=responder.port)
        assert set(found) == {a.key, b.key}
        dev = found[a.key]
        assert dev.ids == ("SN-1", "lab A") and dev.port == a.port
        probe(dev)
        assert dev.error == "" and dev.serial == "SN-1" and dev.label == "lab A"
        assert dev.lasers == {1: "DL pro 397  ·  cooling", 2: "DL pro 866"}
        assert "lab A" in dev.title() and dev.name == "lab A"
        dev_b = probe(found[b.key])
        assert dev_b.lasers == {1: "DL pro 854"} and dev_b.name == "SN-2"
    finally:
        responder.close()
        a.close()
        b.close()


def test_probe_unreachable_and_device_keys():
    d = probe(Device("127.0.0.1", 1), timeout=0.3)
    assert d.error and d.probed and "unreachable" in d.title()
    assert Device.from_key("10.0.0.5").key == "10.0.0.5"
    assert Device.from_key("10.0.0.5:1998").key == "10.0.0.5"
    assert Device.from_key(" 10.0.0.5:2000 ").port == 2000


def test_binary_tuples_ints_and_commands(fake):
    from wmlock.dlcpro import encode_blocks, float_arrays, float_block, lock_points, points_block
    from wmlock.sim import CommandError

    trace = encode_blocks([float_block("x", [1.0, 2.0, 3.0]), float_block("y", [0.5, 0.25, 0.125])])
    fake.params.update({"laser1:scope:data": trace, "laser1:dl:lock:lockpoint:position": (70.25, 0.5),
                        "laser1:dl:lock:spectrum-input-channel": 0,
                        "laser1:dl:lock:candidates": lambda: encode_blocks(
                            [points_block("c", [(70.1, 0.8, "t"), (71.2, 0.2, "t")]), ("s", bytes([3]))])})
    selected = []

    def select(x, y, kind):
        if abs(x - 70.1) > 1e-3:
            raise CommandError("-7 lockpoint not found")
        selected.append((x, y, kind))
        return ""

    fake.commands["laser1:dl:lock:select-lockpoint"] = select
    c = DLCPro("127.0.0.1", fake.port)
    data = float_arrays(c.get_bytes("laser1:scope:data"))
    assert data == {"x": [1.0, 2.0, 3.0], "y": [0.5, 0.25, 0.125]}
    assert c.get_tuple("laser1:dl:lock:lockpoint:position") == (70.25, 0.5)
    assert c.get_int("laser1:dl:lock:spectrum-input-channel") == 0
    pts = lock_points(c.get_value("laser1:dl:lock:candidates"))  # binary, by its name
    assert pts["s"] == 3 and [(round(x, 3), t) for x, _, t in pts["c"]] == [(70.1, "t"), (71.2, "t")]
    assert c.exec("laser1:dl:lock:select-lockpoint", 70.1, 0.8, 116) == ""
    assert selected == [(70.1, 0.8, 116)]
    with pytest.raises(DecopError, match="lockpoint not found"):
        c.exec("laser1:dl:lock:select-lockpoint", 75.0, 0.8, 116)
    with pytest.raises(DecopError, match="unknown command"):
        c.exec("laser1:dl:lock:no-such-thing")
    with pytest.raises(DecopError):
        c.get_bytes("laser1:scan:offset")
    assert c.get_value("laser1:scan:enabled") is False and c.get_value("laser1:scan:offset") == 70.0
    assert c.is_open
    c.close()


def test_decode_and_bad_binary_data():
    from wmlock.dlcpro import data_blocks, decode

    assert decode("#t") is True and decode("3") == 3 and decode("2.5") == 2.5 and decode('"a b"') == "a b"
    assert decode("'(1.5 2)") == (1.5, 2) and decode("(70.1 0.82)") == (70.1, 0.82)
    assert decode("&AAE=") == b"\x00\x01"
    with pytest.raises(DecopError):
        decode("Error: -14 unknown parameter")
    with pytest.raises(ValueError):
        decode("what")
    with pytest.raises(DecopError):
        data_blocks(b"x10\x00abc")  # cut short
    with pytest.raises(DecopError):
        data_blocks(b"xab\x00abc")  # bad length
    assert data_blocks(b"") == []


def test_which_parameters_set_the_piezo_and_the_current_is_what_the_scan_drives():
    """The scan's offset is the DC level of whatever its scan generator drives: the piezo, as a rule; on the lab's
    TA-SHG pro, the current (160 mA there, its piezo at 41.3 V on its own set value)."""
    from wmlock.dlcpro import drives, output_name, scan_output

    assert drives(50) == drives(None) == ("scan:offset", "dl:cc:current-set")  # (not known: as on most)
    assert drives(51) == ("dl:pc:voltage-set", "scan:offset")
    assert drives(20) == ("dl:pc:voltage-set", "dl:cc:current-set")  # (Out A: neither)
    assert output_name(51) == "the current" and output_name(21) == "Out B" and output_name(77) == "output channel 77"
    fake = FakeDLCPro("SN-2", "TA-SHG", {1: {"product": "TA-SHG pro", "scan_out": 51, "current": 160.0,
                                              "offset": 41.3}})
    try:
        c = DLCPro("127.0.0.1", fake.port)
        assert scan_output(c, 1) == 51 and c.get_float("laser1:scan:offset") == 160.0
        assert c.get_str("laser1:scan:unit") == "mA" and c.get_float("laser1:dl:pc:voltage-set") == 41.3
        assert fake.piezo(1) == 41.3 and fake.current(1) == 160.0
        assert c.set("laser1:scan:offset", 160.2) == 0 and fake.current_act(1) == pytest.approx(160.2)
        assert c.set("laser1:dl:cc:current-set", 150.0) == 0 and fake.current(1) == 160.2  # (it drives nothing)
        with pytest.raises(DecopError, match="current-clip"):
            c.set("laser1:scan:offset", 250.0)
        assert c.set("laser1:scan:output-channel", 50) == 0  # switched by hand to the piezo: nothing jumps
        assert fake.piezo(1) == 41.3 and fake.current(1) == pytest.approx(160.2)
        assert c.get_float("laser1:scan:offset") == 41.3 and c.get_str("laser1:scan:unit") == "V"
        del fake.params["laser1:scan:output-channel"]
        assert scan_output(c, 1) is None  # (a controller that does not say)
        c.close()
    finally:
        fake.close()
