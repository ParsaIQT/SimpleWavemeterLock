"""A PC with several wavemeters (meters.py): one program reads them all, an engine each, each framed under its own
name here and on the other PCs."""
import math
import socket
from pathlib import Path

import pytest

from conftest import wait_for
from test_wavemeter import MultiLib, a_and_b, multi_hub
from wmlock.__main__ import log_paths, sim_serials
from wmlock.engine import Engine
from wmlock.meters import Meter, Meters
from wmlock.share import Settings, Share
from wmlock.sim import Demo, SimWavemeter
from wmlock.wavemeter import C_NM_THZ, DllSource, Found, NullSource


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("", 0))
        return s.getsockname()[1]


@pytest.fixture
def demo2():
    d = Demo(discovery_port=0, meters=2, serials=[1234, 5678], exposure=0.004,
             wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0))
    yield d
    d.close()


def meters_of(demo, config, share=None):
    """This PC's wavemeters, as the program has them (__main__): the first engine with config, the others made as
    they are found; with share (settings: the network's), an instance on the network each."""
    siblings = set()
    first = Engine(config, lambda sink: demo.make_source(sink, (config.get("wavemeter") or {}).get("serial")),
                   discovery_targets=["127.0.0.1"], extra_devices=[d.key for d in demo.dlcs])
    sh = Share(first, share, siblings=siblings) if share is not None else None

    def make(section, serial, model):
        e = Engine(section, lambda sink: demo.make_source(sink, serial, model), discovery_targets=["127.0.0.1"],
                   extra_devices=[d.key for d in demo.dlcs])
        if share is None:
            return Meter(e)
        return Meter(e, Share(e, Settings(section.setdefault("share", {}), share), siblings=siblings,
                              subscribe=False))

    return Meters(config, Meter(first, sh), demo, make)


def nm(e, n):
    f = e.channels[n].freq if n in e.channels else math.nan
    return C_NM_THZ / f if f > 0 else math.nan


def reads(e, n, wavelength, tol=1e-4) -> bool:
    return abs(nm(e, n) - wavelength) < tol


def test_one_program_reads_each_wavemeter_of_its_pc_an_engine_each(demo2):
    m = meters_of(demo2, {})
    m.start()
    try:
        first = m.items[0].engine
        assert m.poll() == [] or m.items[0].serial  # (the first is given its wavemeter before any other is)
        assert wait_for(lambda: m.items[0].serial == 1234)
        assert wait_for(lambda: m.poll() or len(m.items) == 2)
        second = m.items[1].engine
        assert [x.serial for x in m.items] == [1234, 5678] and m.version == 1 and m.poll() == []
        assert wait_for(lambda: reads(first, 1, 396.959120) and reads(second, 1, 369.524540)
                        and reads(second, 3, 638.610400), 5)
        assert 7 in first.channels and 7 not in second.channels
        assert m.items[1].name == "simulated WS6 5678"
        cfg = m.to_config()
    finally:
        m.stop()
    assert cfg["wavemeter"] == {"serial": 1234, "model": "simulated WS7"}
    assert list(cfg["other_wavemeters"]) == ["5678"]
    assert cfg["other_wavemeters"]["5678"]["wavemeter"] == {"serial": 5678, "model": "simulated WS6"}
    assert set(cfg["other_wavemeters"]["5678"]["channels"]) == {"1", "2", "3"}
    m = meters_of(demo2, cfg)  # started again: each with its settings, the other made at once
    assert [x.serial for x in m.items] == [1234, 5678]
    m.start()
    try:
        assert wait_for(lambda: reads(m.items[0].engine, 1, 396.959120) and reads(m.items[1].engine, 1, 369.524540),
                        5)
    finally:
        m.stop()


def test_settings_given_the_wrong_wavemeter_are_swapped_with_the_other(demo2):
    m = meters_of(demo2, {"channels": {"1": {"name": "397"}}})
    m.start()
    try:
        assert wait_for(lambda: m.items[0].serial and (m.poll() or len(m.items) == 2))
        a, b = m.items
        a.engine.rename(1, "397 cooling")
        b.engine.rename(1, "369 cooling")
        assert wait_for(lambda: reads(a.engine, 1, 396.959120) and reads(b.engine, 1, 369.524540), 5)
        m.swap(a, b)
        assert wait_for(lambda: reads(a.engine, 1, 369.524540) and reads(b.engine, 1, 396.959120), 5)
        assert a.serial == 5678 and b.serial == 1234 and a.engine.dirty and b.engine.dirty
        assert a.engine.channels[1].name == "397 cooling" and 7 in b.engine.channels  # (the settings stay)
        cfg = m.to_config()
    finally:
        m.stop()
    assert cfg["wavemeter"]["serial"] == 5678 and list(cfg["other_wavemeters"]) == ["1234"]
    assert cfg["other_wavemeters"]["1234"]["channels"]["1"]["name"] == "369 cooling"


def test_a_hidden_wavemeter_is_neither_shown_nor_read_until_shown_again(demo2):
    m = meters_of(demo2, {})
    m.start()
    try:
        assert wait_for(lambda: m.items[0].serial and (m.poll() or len(m.items) == 2))
        second = m.items[1]
        second.engine.rename(2, "399")
        m.hide(second)
        assert [x.serial for x in m.items] == [1234] and m.poll() == [] and m.hidden() == [(5678, "simulated WS6 5678")]
        assert wait_for(lambda: not second.engine.source.is_alive(), 3)
        cfg = m.to_config()
        assert cfg["other_wavemeters"]["5678"]["hidden"] and cfg["other_wavemeters"]["5678"]["channels"]["2"]["name"] \
            == "399"
        shown = m.show(5678)
        assert shown is m.items[1] and m.hidden() == [] and shown.engine.channels[2].name == "399"
        assert wait_for(lambda: reads(shown.engine, 1, 369.524540), 5)
        m.hide(m.items[0])  # (never the first: it holds the settings)
        assert len(m.items) == 2
    finally:
        m.stop()


def test_the_first_is_given_the_wavemeter_its_setpoints_know():
    a, b = a_and_b()  # (b reads 600 THz on its channel 1: the setpoint below)
    lib = MultiLib(a, b)
    hub = multi_hub(lib)
    config = {"channels": {"1": {"name": "x", "setpoint_nm": C_NM_THZ / 600.0005}}}
    first = Engine(config, lambda sink: DllSource(sink, interval=0.002, hub=hub, choose=sink.choose_wavemeter))

    def make(section, serial, model):
        return Meter(Engine(section, lambda sink: hub.make_source(sink, serial, model)))

    m = Meters(config, Meter(first), hub, make)
    m.start()
    try:
        assert wait_for(lambda: m.items[0].serial == 5678)
        assert wait_for(lambda: m.poll() or len(m.items) == 2)
        assert m.items[1].serial == 1234 and m.items[1].engine.config["wavemeter"] == {"serial": 1234, "model": "WS7"}
        assert wait_for(lambda: first.channels[1].freq == 600.0 and m.items[1].engine.channels.get(1) is not None
                        and m.items[1].engine.channels[1].freq == 400.0)
    finally:
        m.stop()


def test_which_wavemeter_settings_are_for():
    e = Engine({"channels": {"1": {"setpoint_nm": C_NM_THZ / 600.0}, "2": {"setpoint_nm": C_NM_THZ / 601.0},
                             "3": {}}}, NullSource)
    fa, fb = Found(0, "WS7", 1234), Found(1, "WS6", 5678)
    assert e.choose_wavemeter([(fa, {1: 400.0, 2: 401.0}), (fb, {1: 600.003, 2: 601.0, 3: 602.0})]) == fb
    assert e.choose_wavemeter([(fa, {1: 600.0}), (fb, {2: 601.0})]) is None  # (as many: no telling)
    assert e.choose_wavemeter([(fa, {1: 400.0}), (fb, {1: 600.02})]) is None  # (20 GHz off: not that one)
    assert Engine({}, NullSource).choose_wavemeter([(fa, {1: 400.0}), (fb, {})]) is None


def test_settings_kept_as_their_wavemeters_and_nothing_locked_on_another():
    e = Engine({"channels": {"1": {"name": "397"}}, "reference": {"channel": 6, "anchor_thz": 411.0}}, NullSource)
    e.on_wavemeter("WS7", 1234)
    assert e.config["wavemeter"] == {"serial": 1234, "model": "WS7"} and e.dirty and e.meter["serial"] == 1234
    ch = e.channels[1]
    ch.freq = ch.raw = 755.0
    ch.state = "on"
    e.dirty = False
    e.on_wavemeter("WS7", 1234)
    assert not e.dirty and ch.state == "on"  # (the same: nothing to do)
    e.on_wavemeter("WS6", 5678)  # (swapped)
    assert e.meter == {"serial": 5678, "model": "WS6"} and ch.state == "off"
    assert ch.message == "unlocked: another wavemeter reads this channel now" and math.isnan(ch.freq)
    assert e.reference.anchor == 411.0 and e.reference.error == 0.0
    e.on_wavemeter("WS6", 0)  # (a DLL that does not say: nothing kept)
    assert e.meter["serial"] == 5678


def test_the_other_pcs_see_each_wavemeter_of_this_one_framed_and_adjust_it(demo2):
    port = free_port()
    m = meters_of(demo2, {}, share={"label": "Lab X", "port": port, "lan": False})
    remote_demo = Demo(discovery_port=0, exposure=0.004)
    remote = Engine({}, remote_demo.make_source, discovery_targets=["127.0.0.1"])
    remote_share = Share(remote, {"label": "Lab R", "port": free_port(), "lan": False,
                                  "peers": [f"127.0.0.1:{port}"]})
    m.start()
    remote.start()
    remote_share.start()
    try:
        assert wait_for(lambda: m.items[0].serial and (m.poll() or len(m.items) == 2))
        x1, x2 = m.items[0].share, m.items[1].share
        assert x2.label == f"simulated WS6 5678 · {x2.host}"  # (named by what it is, until named)

        def seen(label):
            return next((p for p in remote_share.snapshot() if p.label == label and p.rows), None)

        assert wait_for(lambda: seen("Lab X") is not None and seen(x2.label) is not None, 10)
        assert seen(x2.label).rows == [1, 2, 3] and seen("Lab X").host == seen(x2.label).host
        assert wait_for(lambda: [p.label for p in x1.snapshot()] == ["Lab R"], 5)  # (its sibling: not another PC's)
        assert all(p.id != x1.id for p in x2.snapshot()) and all(p.id != x2.id for p in x1.snapshot())
        for x in (x1, x2):  # R wants the rows of each
            assert wait_for(lambda x=x: x.peers and next(iter(x.peers.values())).want == "fast", 5)
        rp = next(p for p in remote_share.snapshot() if p.label == "Lab X")
        assert wait_for(lambda: rp.want == "fast", 5)  # X's first wants R's rows...
        rp2 = next(p for p in remote_share.snapshot() if p.label == x2.label)
        assert rp2.want == ""  # ...its second does not: the window has them once
        remote_share.command(rp2.id, 2, "rename", name="399 cooling")
        assert wait_for(lambda: m.items[1].engine.channels[2].name == "399 cooling", 5)
        assert m.items[0].engine.channels[2].name != "399 cooling"
    finally:
        remote_share.stop()
        remote_share.join(2.0)
        remote.stop()
        remote_demo.close()
        m.stop()


def test_logs_and_demo_serials():
    home = Path.home()
    assert log_paths(home / ".simplewavemeterlock.json") == (home / ".simplewavemeterlock-recoveries.jsonl",
                                                            home / ".simplewavemeterlock-dropouts.jsonl")
    assert log_paths(home / "lab.json", 5678) == (home / "lab-wlm5678-recoveries.jsonl",
                                                 home / "lab-wlm5678-dropouts.jsonl")
    a, b = sim_serials("demo-a.json"), sim_serials("demo-b.json")
    assert a == sim_serials("demo-a.json") and a[1] == a[0] + 1 and not set(a) & set(b)
