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


# ------------------------------------------------- several wavemeters on one PC
class Server:
    """One wavemeter's WLM software, as wlmData sees it."""

    def __init__(self, serial, kind=7, freqs=None, n=4, use=(1, 2, 3, 4)):
        self.serial, self.kind, self.n, self.use = serial, kind, n, set(use)
        self.freqs = dict(freqs or {})


class MultiLib:
    """Stands in for wlmData.dll with several wavemeters running: PresetWLMIndex has every other call address one of
    them, for the whole program (the first until it is called)."""

    def __init__(self, *servers):
        self.servers = list(servers)
        self.index = 0
        self.sets = []  # (serial, ch, use): SetSwitcherSignalStates, on which wavemeter

    def _s(self):
        return self.servers[self.index] if 0 <= self.index < len(self.servers) else None

    def GetWLMCount(self, _):
        return len(self.servers)

    def PresetWLMIndex(self, i):
        self.index = i
        return 0

    def GetWLMVersion(self, k):
        s = self._s()
        return -5 if s is None else {0: s.kind, 1: s.serial}.get(k, 0)

    def GetFrequencyNum(self, ch, _):
        s = self._s()
        return -5.0 if s is None else s.freqs.get(ch, 0.0)

    def GetChannelsCount(self, _):
        s = self._s()
        return s.n if s else 0

    def GetSwitcherMode(self, _):
        return 1

    def GetSwitcherChannel(self, _):
        return 1

    def GetSwitcherSignalStates(self, ch, use, show):
        s = self._s()
        use._obj.value = int(s is not None and ch in s.use)
        return 0

    def SetSwitcherSignalStates(self, ch, use, show):
        s = self._s()
        (s.use.add if use else s.use.discard)(ch)
        self.sets.append((s.serial, ch, use))
        return 0


class MeterSink(Sink):
    def __init__(self):
        super().__init__()
        self.meters = []

    def on_wavemeter(self, model, serial):
        self.meters.append((model, serial))


def multi_hub(lib, claims=None):
    hub = wavemeter.WlmHub(claims=claims)
    w = fake_wlm(lib)
    w.has |= {"GetWLMVersion", "PresetWLMIndex"}
    hub.wlm = w
    return hub


def a_and_b():
    a = Server(1234, 7, {1: 400.0, 2: 401.0}, use=(1, 2))
    b = Server(5678, 6, {1: 600.0, 2: 601.0, 3: 602.0}, use=(1, 2, 3))
    return a, b


def test_the_wavemeters_of_a_pc_are_found_by_serial_number():
    a, b = a_and_b()
    lib = MultiLib(a, b)
    w = multi_hub(lib).dll()
    assert w.scan() == [wavemeter.Found(0, "WS7", 1234), wavemeter.Found(1, "WS6", 5678)]
    assert w.preset_used
    lib.servers = [b]  # one stopped: the one left is the first
    assert w.scan() == [wavemeter.Found(0, "WS6", 5678)]
    assert wavemeter.meter_name("WS7", 1234) == "WS7 1234" and wavemeter.meter_name("", 0) == ""


def test_each_source_reads_its_own_wavemeter_never_the_others():
    a, b = a_and_b()
    lib = MultiLib(a, b)
    hub = multi_hub(lib)
    sa, sb = MeterSink(), MeterSink()
    src_a = DllSource(sa, interval=0.0003, serial=1234, hub=hub)
    src_b = DllSource(sb, interval=0.0003, serial=5678, hub=hub)
    src_a.start()
    src_b.start()
    stop = threading.Event()

    def change():  # new readings all the time, on both
        k = 0
        while not stop.is_set():
            k += 1
            a.freqs[1], b.freqs[1] = 400.0 + k * 1e-6, 600.0 + k * 1e-6
            time.sleep(0.0002)

    t = threading.Thread(target=change, daemon=True)
    t.start()
    try:
        assert wait_for(lambda: len(sa.readings) > 300 and len(sb.readings) > 300, 10)
        assert sa.meters == [("WS7", 1234)] and sb.meters == [("WS6", 5678)]
        assert sa.active[-1] == [1, 2] and sb.active[-1] == [1, 2, 3]
        lib.servers = [b, a]  # their WLM software started again, the other way round
        n_a, n_b = len(sa.readings), len(sb.readings)
        assert wait_for(lambda: len(sa.readings) > n_a + 300 and len(sb.readings) > n_b + 300, 10)
    finally:
        stop.set()
        src_a.stop()
        src_b.stop()
    assert all(r.freq < 500 for r in sa.readings) and all(r.freq > 500 for r in sb.readings)


def test_a_source_given_none_takes_the_one_it_is_told_and_keeps_to_it():
    a, b = a_and_b()
    lib = MultiLib(a, b)
    sink = MeterSink()
    seen = []

    def choose(cands):
        seen.extend(cands)
        return next(f for f, freqs in cands if abs(freqs.get(1, 0) - 600.0) < 0.01)

    src = DllSource(sink, interval=0.001, hub=multi_hub(lib), choose=choose)
    src.start()
    try:
        assert wait_for(lambda: sink.meters == [("WS6", 5678)] and sink.readings)
        assert [(f.serial, freqs) for f, freqs in seen] == [(1234, {1: 400.0, 2: 401.0}),
                                                             (5678, {1: 600.0, 2: 601.0, 3: 602.0})]
        assert src.wanted == 5678 and all(r.freq > 500 for r in sink.readings)
    finally:
        src.stop()
    sink = MeterSink()
    src = DllSource(sink, interval=0.001, hub=multi_hub(lib), choose=lambda cands: None)  # no telling: the first
    src.start()
    try:
        assert wait_for(lambda: sink.meters == [("WS7", 1234)])
    finally:
        src.stop()


def test_a_wavemeter_not_running_is_not_stood_in_for():
    a, b = a_and_b()
    lib = MultiLib(a)
    sink = MeterSink()
    src = DllSource(sink, interval=0.001, serial=5678, model="WS6", hub=multi_hub(lib))
    src.start()
    try:
        assert wait_for(lambda: sink.states and sink.states[-1] == (False, "WS6 5678 is not running (this PC has "
                                                                           "WS7 1234)"))
        time.sleep(0.05)
        assert sink.readings == [] and sink.meters == []
        lib.servers = [a, b]  # its WLM software started
        assert wait_for(lambda: sink.states[-1] == (True, "") and sink.readings, 3)
        assert sink.meters == [("WS6", 5678)] and all(r.freq > 500 for r in sink.readings)
    finally:
        src.stop()


def test_a_channel_is_switched_on_its_own_wavemeter():
    a, b = a_and_b()
    lib = MultiLib(a, b)
    hub = multi_hub(lib)
    sa, sb = MeterSink(), MeterSink()
    src_a = DllSource(sa, interval=0.001, serial=1234, hub=hub)
    src_b = DllSource(sb, interval=0.001, serial=5678, hub=hub)
    src_a.start()
    src_b.start()
    try:
        assert wait_for(lambda: sa.off and sa.off[-1] == [3, 4] and sb.off and sb.off[-1] == [4])
        src_b.switch(4, True)
        assert wait_for(lambda: sb.active[-1] == [1, 2, 3, 4])
        assert lib.sets == [(5678, 4, 1)] and 4 not in a.use
    finally:
        src_a.stop()
        src_b.stop()


def test_two_copies_of_the_program_never_read_the_same_wavemeter(tmp_path):
    a, b = a_and_b()
    lib = MultiLib(a, b)
    first, second = multi_hub(lib, wavemeter.Claims(tmp_path)), multi_hub(lib, wavemeter.Claims(tmp_path))
    s1, s2 = MeterSink(), MeterSink()
    src1 = DllSource(s1, interval=0.001, serial=1234, hub=first)
    src1.start()
    try:
        assert wait_for(lambda: s1.meters == [("WS7", 1234)])
        assert not second.free(1234) and second.free(5678)
        src2 = DllSource(s2, interval=0.001, hub=second)  # (given none: the one the other copy does not read)
        src2.start()
        try:
            assert wait_for(lambda: s2.meters == [("WS6", 5678)])
            src2.select(1234)
            assert wait_for(lambda: s2.states[-1] == (False, "WS7 1234: another copy of this program on this PC "
                                                             "reads it"))
        finally:
            src2.stop()
            src2.join(2)
    finally:
        src1.stop()
        src1.join(2)
    assert first.free(1234) and second.free(5678)  # (let go of when they stopped)


def test_two_sources_swap_their_wavemeters():
    a, b = a_and_b()
    lib = MultiLib(a, b)
    hub = multi_hub(lib)
    sa, sb = MeterSink(), MeterSink()
    src_a = DllSource(sa, interval=0.001, serial=1234, hub=hub)
    src_b = DllSource(sb, interval=0.001, serial=5678, hub=hub)
    src_a.start()
    src_b.start()
    try:
        assert wait_for(lambda: sa.readings and sb.readings)
        hub.swap(src_a, src_b)
        assert wait_for(lambda: sa.meters[-1] == ("WS6", 5678) and sb.meters[-1] == ("WS7", 1234))
        n_a, n_b = len(sa.readings), len(sb.readings)
        a.freqs[2], b.freqs[2] = 401.5, 601.5
        assert wait_for(lambda: any(r.freq == 601.5 for r in sa.readings[n_a:])
                        and any(r.freq == 401.5 for r in sb.readings[n_b:]))
        assert all(r.freq > 500 for r in sa.readings[n_a:]) and all(r.freq < 500 for r in sb.readings[n_b:])
        assert hub.taken == {1234: src_b, 5678: src_a}
    finally:
        src_a.stop()
        src_b.stop()


def test_the_settings_lock_lets_one_copy_have_them(tmp_path):
    from wmlock.config import FileLock, lock_path

    path = lock_path(tmp_path / "s.json")
    assert path.name == "s.json.lock"
    one, two = FileLock(path), FileLock(path)
    assert one.acquire() and one.held
    assert not two.acquire() and not two.held
    one.release()
    assert two.acquire()
    two.release()


# ------------------------------------------------- HTTP: a server.py with several wavemeters
class MultiHandler(BaseHTTPRequestHandler):
    """A server.py serving two wavemeters (?wlm=), or one from before (old: it ignores ?wlm, says nothing of whose)."""
    meters = {1234: 755.2227, 5678: 384.2}
    old = False
    paths = []

    def _send(self, status, data):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        MultiHandler.paths.append(self.path)
        path, _, query = self.path.partition("?")
        if path == "/api/v1/wavemeters":
            if self.old:
                self._send(404, {"detail": "Not Found"})
            else:
                self._send(200, {"wavemeters": [{"index": i, "model": "WS7", "serial": s}
                                                for i, s in enumerate(self.meters)]})
            return
        wlm = int(query.split("=")[1]) if query.startswith("wlm=") and not self.old else next(iter(self.meters))
        if wlm not in self.meters:
            self._send(404, {"detail": f"wavemeter {wlm} is not running there (it has WS7 1234, WS7 5678)"})
            return
        data = {"channels": {"channel_1": channel("Valid", self.meters[wlm])}}
        if not self.old:
            data["wavemeter"] = {"model": "WS7", "serial": wlm}
        self._send(200, data)

    def log_message(self, *args):
        pass


@pytest.fixture
def multi_server():
    MultiHandler.protocol_version = "HTTP/1.1"
    MultiHandler.old, MultiHandler.paths = False, []
    srv = ThreadingHTTPServer(("127.0.0.1", 0), MultiHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()


def test_http_source_reads_the_wavemeter_it_keeps_to(multi_server):
    url = f"127.0.0.1:{multi_server.server_port}"
    found = wavemeter.HttpMeters(url)
    try:
        assert wait_for(lambda: [f.serial for f in found.scan()] == [1234, 5678])
    finally:
        found.stop()
    first, second = MeterSink(), MeterSink()
    s1 = HttpSource(first, url, interval=0.01)  # (given none: the server's first, kept to)
    s2 = found.make_source(second, 5678, "WS7")
    s1.start()
    s2.start()
    try:
        assert wait_for(lambda: first.meters == [("WS7", 1234)] and second.meters == [("WS7", 5678)])
        assert wait_for(lambda: first.readings and second.readings)
        assert first.readings[0].freq == 755.2227 and second.readings[0].freq == 384.2
        assert s1.wanted == 1234 and any(p.endswith("?wlm=5678") for p in MultiHandler.paths)
        s2.select(4321)  # not running there
        assert wait_for(lambda: second.states[-1] == (False, "wavemeter 4321 is not running there (it has WS7 1234, "
                                                              "WS7 5678)"))
        MultiHandler.old = True  # a server.py from before: it cannot say whose its readings are
        s2.select(5678)
        assert wait_for(lambda: second.states[-1][1].endswith("cannot tell its wavemeters apart: update its "
                                                              "server.py"))
    finally:
        s1.stop()
        s2.stop()


def test_server_serves_each_wavemeter_of_its_pc(monkeypatch):
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient

    a, b = a_and_b()
    lib = MultiLib(a, b)
    lib.GetTemperature = lambda _: 24.5
    lib.GetPressure = lambda _: 1001.0
    lib.GetWavelengthNum = lambda ch, _: C_NM_THZ / lib.GetFrequencyNum(ch, 0)
    wlm_data = types.ModuleType("wlmData")
    wlm_data.dll = lib
    wlm_data.LoadDLL = lambda path: None
    wlm_const = types.ModuleType("wlmConst")
    for name, value in dict(ErrWlmMissing=-5, ErrNoSignal=-1, ErrBadSignal=-2, ErrLowSignal=-3,
                            ErrBigSignal=-4, ErrOutOfRange=-14, ErrTemperature=-1000).items():
        setattr(wlm_const, name, value)
    monkeypatch.setitem(sys.modules, "wlmData", wlm_data)
    monkeypatch.setitem(sys.modules, "wlmConst", wlm_const)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "server"))
    server = importlib.import_module("server")
    try:
        with TestClient(server.app) as client:
            assert client.get("/api/v1/wavemeters").json() == {"wavemeters": [
                {"index": 0, "model": "WS7", "serial": 1234}, {"index": 1, "model": "WS6", "serial": 5678}]}
            first = client.get("/api/v1/wavelength").json()  # (none asked for: the first, as from before)
            assert first["wavemeter"] == {"model": "WS7", "serial": 1234}
            assert first["channels"]["channel_1"]["frequency_thz"] == 400.0
            other = client.get("/api/v1/wavelength?wlm=5678").json()
            assert other["wavemeter"] == {"model": "WS6", "serial": 5678}
            assert other["channels"]["channel_3"]["frequency_thz"] == 602.0
            assert client.get("/api/v1/wavelength").json()["wavemeter"]["serial"] == 1234  # (set back each time)
            r = client.get("/api/v1/wavelength?wlm=4321")
            assert r.status_code == 404 and "WS7 1234, WS6 5678" in r.json()["detail"]
            assert client.post("/api/v1/channel/4/use?wlm=5678", json={"use": True}).status_code == 200
            assert lib.sets == [(5678, 4, 1)] and 4 not in a.use
    finally:
        sys.modules.pop("server", None)
