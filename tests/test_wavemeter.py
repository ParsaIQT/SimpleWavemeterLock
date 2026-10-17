import ctypes
import importlib
import json
import math
import sys
import threading
import time
import types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from conftest import wait_for
from wmlock import wavemeter
from wmlock.wavemeter import C_NM_THZ, DllSource, HttpSource, WlmDll


class Sink:
    def __init__(self):
        self.readings, self.active, self.off, self.states, self.failed = [], [], [], [], []

    def on_readings(self, rs):
        self.readings.extend(rs)

    def on_active(self, chans, off=()):
        self.active.append(chans)
        self.off.append(list(off))

    def on_state(self, ok, msg):
        self.states.append((ok, msg))

    def on_switch_failed(self, ch, on, why):
        self.failed.append((ch, on, why))


# ---------------------------------------------------------------- HTTP source
class Handler(BaseHTTPRequestHandler):
    payload, status = {}, 200
    posts, post_status = [], 200  # what was POSTed; 404: a server.py that cannot switch channels

    def _send(self, status, data):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self._send(Handler.status, Handler.payload)

    def do_POST(self):
        data = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        Handler.posts.append((self.path, data))
        if Handler.post_status != 200:
            self._send(Handler.post_status, {"detail": "Not Found"})
            return
        ch = int(self.path.split("/")[-2])
        Handler.payload["channels"][f"channel_{ch}"] = channel("Valid", 500.0) if data["use"] else \
            {**channel("Disabled"), "use": False}
        self._send(200, {"channel": ch, "use": data["use"]})

    def log_message(self, *args):
        pass


@pytest.fixture
def http_server():
    Handler.protocol_version = "HTTP/1.1"  # keep-alive, like uvicorn
    srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()


def channel(status, f=None):
    return {"status": status, "is_valid": f is not None, "frequency_thz": f,
            "wavelength_nm": C_NM_THZ / f if f else None}


def test_http_source_reads_server_json(http_server):
    Handler.status = 200
    Handler.payload = {"server_status": "Online", "channels": {
        "channel_1": channel("Valid", 755.2227),
        "channel_2": channel("Low Signal"),
        "channel_3": channel("Disabled"),
        "channel_4": channel("High Signal"),
    }}
    sink = Sink()
    src = HttpSource(sink, f"127.0.0.1:{http_server.server_port}", interval=0.01)
    src.start()
    try:
        assert wait_for(lambda: len(sink.readings) >= 3)
        assert sink.active[0] == [1, 2, 4]
        by_ch = {r.ch: r for r in sink.readings}
        assert by_ch[1].freq == 755.2227 and by_ch[1].status == ""
        assert math.isnan(by_ch[2].freq) and by_ch[2].status == "low signal"
        assert by_ch[4].status == "overexposed"
        n = len(sink.readings)
        Handler.payload["channels"]["channel_1"] = channel("Valid", 755.2228)
        assert wait_for(lambda: len(sink.readings) == n + 1)  # only the change is pushed
        assert sink.readings[-1].ch == 1
        Handler.status = 503
        assert wait_for(lambda: sink.states[-1] == (False, "wlmServer is not running"))
    finally:
        src.stop()


def test_http_source_switches_a_channel_on_and_off(http_server):
    Handler.status, Handler.post_status, Handler.posts = 200, 200, []
    Handler.payload = {"server_status": "Online", "channels": {
        "channel_1": channel("Valid", 755.2227),
        "channel_2": {**channel("Disabled"), "use": False},  # 'Use' unticked: it can be ticked
        "channel_3": channel("Disabled"),                    # not measured for another reason (or an old server)
    }}
    sink = Sink()
    src = HttpSource(sink, f"127.0.0.1:{http_server.server_port}", interval=0.01)
    src.start()
    try:
        assert wait_for(lambda: sink.active and sink.active[-1] == [1] and sink.off[-1] == [2])
        src.switch(2, True)
        assert wait_for(lambda: sink.active[-1] == [1, 2] and sink.off[-1] == [])
        assert Handler.posts == [("/api/v1/channel/2/use", {"use": True})]
        assert wait_for(lambda: any(r.ch == 2 and r.freq == 500.0 for r in sink.readings))
        src.switch(2, False)
        assert wait_for(lambda: sink.off[-1] == [2])
        Handler.post_status = 404  # a server.py from before: it cannot
        src.switch(2, True)
        assert wait_for(lambda: sink.failed)
        assert sink.failed == [(2, True, "this server cannot switch channels: update its server.py")]
        assert sink.off[-1] == [2]
    finally:
        src.stop()


# ----------------------------------------------------------------- DLL logic
class FakeLib:
    """Stands in for wlmData.dll."""

    def __init__(self, n=4, mode=1, use=(1, 2, 4), selected=2):
        self.n, self.mode, self.use, self.selected = n, mode, set(use), selected
        self.freqs = {1: 755.1, 2: -3.0, 3: 0.0, 4: 384.2}
        self.running = 1
        self.set_result = 0
        self.sets = []  # SetSwitcherSignalStates calls

    def SetSwitcherSignalStates(self, ch, use, show):
        self.sets.append((ch, use, show))
        if self.set_result >= 0:
            (self.use.add if use else self.use.discard)(ch)
        return self.set_result

    def GetWLMCount(self, _):
        return self.running

    def GetChannelsCount(self, _):
        return self.n

    def GetSwitcherMode(self, _):
        return self.mode

    def GetSwitcherChannel(self, _):
        return self.selected

    def GetSwitcherSignalStates(self, ch, use, show):
        use._obj.value = int(ch in self.use)
        return 0

    def GetFrequencyNum(self, ch, _):
        return self.freqs.get(ch, 0.0)


def fake_wlm(lib, switch=True):
    w = object.__new__(WlmDll)
    w.lib = lib
    w.has = {"GetChannelsCount", "GetSwitcherMode", "GetSwitcherChannel", "GetSwitcherSignalStates"}
    if switch:
        w.has.add("SetSwitcherSignalStates")
    w._use, w._show = ctypes.c_long(), ctypes.c_long()
    return w


def test_active_channels_follow_the_switch():
    lib = FakeLib()
    w = fake_wlm(lib)
    assert w.active_channels() == [1, 2, 4]
    assert w.channels() == ([1, 2, 4], [3])  # 3: 'Use' unticked, and it can be ticked
    assert fake_wlm(lib, switch=False).channels() == ([1, 2, 4], [])
    assert w.set_use(3, True) == "" and lib.sets[-1][:2] == (3, 1) and w.channels() == ([1, 2, 3, 4], [])
    lib.set_result = -2
    assert w.set_use(3, False) == "SetSwitcherSignalStates: could not set"
    assert fake_wlm(lib, switch=False).set_use(3, False) == "wlmData has no SetSwitcherSignalStates"
    lib.mode = 0
    assert w.active_channels() == [2] and w.channels() == ([2], [])  # not cycling: nothing to tick
    lib.n = 1
    assert w.active_channels() == [1]


def test_dll_source_pushes_changes_only(monkeypatch):
    lib = FakeLib()
    monkeypatch.setattr(wavemeter, "WlmDll", lambda path: fake_wlm(lib))
    sink = Sink()
    src = DllSource(sink, interval=0.001)
    src.start()
    try:
        assert wait_for(lambda: len(sink.readings) >= 3)
        assert sink.active[0] == [1, 2, 4] and sink.states[-1] == (True, "")
        time.sleep(0.05)
        assert len(sink.readings) == 3  # nothing changed, nothing pushed
        assert {r.ch: r.status for r in sink.readings} == {1: "", 2: "low signal", 4: ""}
        lib.freqs[4] = 384.3
        assert wait_for(lambda: len(sink.readings) == 4)
        assert sink.readings[-1].ch == 4 and sink.readings[-1].freq == 384.3
        lib.freqs[1] = wavemeter.INF_NOTHING_CHANGED
        time.sleep(0.05)
        assert len(sink.readings) == 4
        lib.running = 0
        assert wait_for(lambda: sink.states[-1] == (False, "wlmServer is not running"), timeout=2.5)
    finally:
        src.stop()


def test_dll_source_switches_a_channel_and_skips_its_old_value(monkeypatch):
    lib = FakeLib(use=(1, 3))
    lib.freqs[3] = 500.0
    monkeypatch.setattr(wavemeter, "WlmDll", lambda path: fake_wlm(lib))
    sink = Sink()
    src = DllSource(sink, interval=0.001)
    src.start()
    try:
        assert wait_for(lambda: any(r.ch == 3 for r in sink.readings))
        assert sink.active[-1] == [1, 3] and sink.off[-1] == [2, 4]
        lib.use.discard(3)  # someone unticks it in the WLM software
        assert wait_for(lambda: sink.off[-1] == [2, 3, 4], timeout=2.5)
        n = len(sink.readings)
        src.switch(3, True)
        assert wait_for(lambda: sink.active[-1] == [1, 3])  # at once, not at the next look at the switch
        assert lib.sets == [(3, 1, 0)]  # ('Show' as it was)
        time.sleep(0.05)
        assert len(sink.readings) == n  # 500.0 again: its value from before, not a measurement
        lib.freqs[3] = 500.1  # measured anew
        assert wait_for(lambda: len(sink.readings) == n + 1)
        assert sink.readings[-1].ch == 3 and sink.readings[-1].freq == 500.1
        lib.set_result = -10
        src.switch(4, True)
        assert wait_for(lambda: sink.failed)
        assert sink.failed == [(4, True, "SetSwitcherSignalStates: channel not available")]
    finally:
        src.stop()


def test_dll_source_reports_missing_dll():
    sink = Sink()
    src = DllSource(sink, path="/nonexistent/wlmData.dll")
    src.start()
    try:
        assert wait_for(lambda: sink.states and not sink.states[-1][0])
        assert "cannot load wlmData" in sink.states[-1][1]
    finally:
        src.stop()


# ------------------------------------------------- server/server.py (FastAPI)
def test_server_reports_only_used_channels(monkeypatch):
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient

    lib = FakeLib(n=4, use=(1, 4))
    lib.GetTemperature = lambda _: 24.5
    lib.GetPressure = lambda _: 1001.0
    lib.GetWavelengthNum = lambda ch, _: C_NM_THZ / lib.GetFrequencyNum(ch, 0)
    wlm_data = types.ModuleType("wlmData")
    wlm_data.dll = lib
    wlm_const = types.ModuleType("wlmConst")
    for name, value in dict(ErrWlmMissing=-5, ErrNoSignal=-1, ErrBadSignal=-2, ErrLowSignal=-3,
                            ErrBigSignal=-4, ErrOutOfRange=-14, ErrTemperature=-1000).items():
        setattr(wlm_const, name, value)
    monkeypatch.setitem(sys.modules, "wlmData", wlm_data)
    monkeypatch.setitem(sys.modules, "wlmConst", wlm_const)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "server"))
    loaded = []
    wlm_data.LoadDLL = loaded.append
    server = importlib.import_module("server")
    try:
        with TestClient(server.app) as client:  # runs the lifespan (DLL load)
            data = client.get("/api/v1/wavelength").json()
            r = client.post("/api/v1/channel/2/use", json={"use": True})  # tick it
            assert r.status_code == 200 and r.json() == {"channel": 2, "use": True} and 2 in lib.use
            assert client.get("/api/v1/wavelength").json()["channels"]["channel_2"]["status"] == "Low Signal"
            assert client.post("/api/v1/channel/2/use", json={"use": False}).status_code == 200 and 2 not in lib.use
            assert client.post("/api/v1/channel/9/use", json={"use": True}).status_code == 404
            lib.mode = 0  # the switch not cycling: only its channel 2 is measured, and nothing can be ticked
            assert client.post("/api/v1/channel/3/use", json={"use": True}).status_code == 409
            assert "use" not in client.get("/api/v1/wavelength").json()["channels"]["channel_3"]
            lib.mode = 1
            lib.running = 0
            assert client.get("/api/v1/wavelength").status_code == 503
    finally:
        sys.modules.pop("server", None)
    assert loaded == ["wlmData.dll"]
    ch = data["channels"]
    assert list(ch) == ["channel_1", "channel_2", "channel_3", "channel_4"]
    assert ch["channel_1"]["frequency_thz"] == 755.1
    assert ch["channel_2"]["status"] == ch["channel_3"]["status"] == "Disabled"
    assert ch["channel_2"]["use"] is False  # 'Use' unticked: POST .../use can tick it
    assert ch["channel_4"]["is_valid"]
    assert data["environment"] == {"temperature_c": 24.5, "pressure_mbar": 1001.0}
