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
