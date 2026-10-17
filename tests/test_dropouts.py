import json
import logging
import re
import time

import pytest

from conftest import wait_for
from test_engine import hop_engine, quiet_demo  # noqa: F401 (a fixture)
from wmlock.dropouts import Detector, Monitor
from wmlock.modehop import HopParams


# ------------------------------------------------------------------ detector
def test_a_run_below_the_limit_is_a_dropout_spanning_its_samples():
    d = Detector(min_s=0.0005)
    v = [1.0] * 10 + [0.1] * 12 + [1.0] * 10  # 12 samples of 0.25 ms: 3 ms
    assert d.feed(0, 0.00025, v, 0.5) == [(10, 12, 0.1)]


def test_too_short_or_a_single_sample_is_not_a_dropout():
    d = Detector(min_s=0.0005)
    assert d.feed(0, 0.00025, [1.0, 0.1, 1.0, 1.0], 0.5) == []  # one sample: noise
    assert Detector(min_s=0.002).feed(0, 0.00025, [1.0] + [0.1] * 4 + [1.0] * 3, 0.5) == []  # 1 ms < 2 ms


def test_one_sample_above_the_limit_does_not_split_a_dropout():
    d = Detector(min_s=0.0005)
    v = [1.0, 0.1, 0.1, 0.1, 0.9, 0.1, 0.1, 1.0, 1.0]
    assert d.feed(0, 0.001, v, 0.5) == [(1, 6, 0.1)]


def test_a_dropout_carries_over_from_one_batch_to_the_next():
    d = Detector(min_s=0.0005)
    assert d.feed(100, 0.00025, [1.0, 1.0, 0.2, 0.05], 0.5) == []
    assert d.feed(104, 0.00025, [0.1, 0.1, 1.0, 1.0], 0.5) == [(102, 4, 0.05)]


def test_a_gap_ends_a_dropout_going_on():
    d = Detector(min_s=0.0005)
    d.feed(0, 0.00025, [1.0, 0.1, 0.1, 0.1], 0.5)
    assert d.gap(0.00025) == [(1, 3, 0.1)]
    assert d.feed(10, 0.00025, [1.0, 1.0], 0.5) == []


def test_polling_counts_a_single_low_read():
    d = Detector(min_s=0.0, min_n=1)
    assert d.feed(7, 0.005, [0.1], 0.5) == []
    assert d.feed(8, 0.005, [1.0], 0.5) == []
    assert d.feed(9, 0.005, [1.0], 0.5) == [(7, 1, 0.1)]


# ---------------------------------------------------------- monitor, alone
def run_monitor(demo, status, blips, wait=3.0, **attrs):
    shg = demo.laser(demo.SHG)
    got, summaries = [], []
    m = Monitor("127.0.0.1", demo.dlcs[2].port, 1, status=status, report=got.append, summary=summaries.append,
                name="channel 7")
    for k, v in attrs.items():
        setattr(m, k, v)
    m.start()
    try:
        assert wait_for(lambda: m.method, 5)
        time.sleep(0.5)
        for seconds in blips:
            shg.lock.blip(seconds)
            time.sleep(0.4)
        time.sleep(wait)
    finally:
        m.stop()
        m.join(10)
    return m, got, summaries


def test_the_monitor_reads_the_recorder_and_puts_its_settings_back(demo):
    dlc = demo.dlcs[2]
    before = {k: v for k, v in dlc.params.items() if ":recorder:" in k and not callable(v)}
    m, got, summaries = run_monitor(demo, lambda: (True, 0.45, 0), [0.003, 0.0012, 0.0003, 0.05], SUMMARY_S=2.0,
                                    _summary_at=time.monotonic() + 2.0)
    assert m.method == "recorder" and m.resolution == pytest.approx(0.00025, rel=0.05)
    ms = [round(d.duration * 1e3, 2) for d in got]
    assert len(ms) == 3 and ms[0] == pytest.approx(3.0, abs=0.3) and ms[1] == pytest.approx(1.25, abs=0.3)
    assert ms[2] == pytest.approx(50.0, abs=0.3)  # (0.3 ms: one sample, or two: too short)
    assert all(d.method == "recorder" and d.low < 0.05 and d.limit == 0.45 for d in got)
    assert all(abs(d.t - time.time()) < 30 for d in got)
    assert summaries and "the DLC pro's recorder, 0.25 ms samples" in summaries[0]
    after = {k: v for k, v in dlc.params.items() if ":recorder:" in k and not callable(v)}
    assert after == before  # put back as it was found


def test_a_recorder_not_read_while_it_records_is_read_when_full(demo):
    demo.laser(demo.SHG).lock.recorder.busy = True
    m, got, _ = run_monitor(demo, lambda: (True, 0.45, 0), [0.003, 0.002], wait=2.5, RECORD_S=1.5)
    assert m.method == "recorder" and m.when_full
    assert [round(d.duration * 1e3, 1) for d in got] == [pytest.approx(3.0, abs=0.3), pytest.approx(2.0, abs=0.3)]


def test_while_the_lock_should_not_hold_nothing_counts(demo):
    m, got, _ = run_monitor(demo, lambda: (False, 0.45, 0), [0.003, 0.05])
    assert m.method == "recorder" and got == []
    m, got, _ = run_monitor(demo, lambda: (True, None, 0), [0.003, 0.05])  # its level locked not known yet
    assert got == []


def test_without_a_recorder_it_polls_and_says_why(demo, caplog):
    dlc = demo.dlcs[2]
    for k in [k for k in dlc.params if ":recorder:" in k]:
        del dlc.params[k]
    with caplog.at_level(logging.INFO, logger="wmlock.dropouts"):
        m, got, _ = run_monitor(demo, lambda: (True, 0.45, 0), [0.08], wait=6.0)
    assert m.method == "polling" and "no recorder" in m.why_not
    assert len(got) == 1 and got[0].method == "polling" and got[0].duration == pytest.approx(0.08, abs=0.03)
    assert any("by polling" in r.getMessage() and "no recorder" in r.getMessage() for r in caplog.records)


def test_a_recorder_that_does_not_record_is_given_up(demo):
    dlc = demo.dlcs[2]
    dlc.on_set.pop("laser1:recorder:enabled")  # it takes 'enabled', but nothing happens (a trigger awaited, say)
    m, got, _ = run_monitor(demo, lambda: (True, 0.45, 0), [], wait=4.0)
    assert m.method == "polling" and m.why_not.startswith("it does not record (state “idle”")
    assert dlc.params["laser1:recorder:inputs:channel1:low-pass-filter:enabled"] is True  # put back


# ---------------------------------------------------------- through the engine
def test_lock_dropouts_are_logged_in_the_console_and_a_file(quiet_demo, tmp_path, caplog):  # noqa: F811
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    shg = demo.laser(demo.SHG)
    path = tmp_path / "dropouts.jsonl"
    with caplog.at_level(logging.INFO, logger="wmlock.dropouts"):
        e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True), drop_path=path)
        e.DROP_SETTLE_S = 1.0
        try:
            assert wait_for(lambda: ch.hop.trans_locked > 0 and e.monitors[demo.SHG].method == "recorder", 10)
            time.sleep(1.5)  # (settled)
            shg.lock.blip(0.004)
            time.sleep(1.2)
            shg.lock.blip(0.0015)
            assert wait_for(lambda: ch.drops_total == 2, 5)
            lines = [json.loads(x) for x in path.read_text().splitlines()]
            assert [x["duration_ms"] for x in lines] == [pytest.approx(4.0, abs=0.3), pytest.approx(1.5, abs=0.3)]
            assert lines[0]["channel"] == demo.SHG and lines[0]["method"] == "recorder" and lines[0]["low_v"] < 0.1
            assert lines[0]["limit_v"] == pytest.approx(0.5 * ch.hop.trans_locked, rel=0.1)
            said = [r.getMessage() for r in caplog.records if r.levelno == logging.WARNING]
            assert any(re.search(r"channel 7: lock dropout 4\.\d ms: transmission down to", s) for s in said), said
            assert "2 in the last hour" in said[-1]

            time.sleep(1.2)
            for _ in range(4):  # a burst: one line a second in the console, every one in the file
                shg.lock.blip(0.002)
                time.sleep(0.15)
            time.sleep(1.2)
            shg.lock.blip(0.002)
            assert wait_for(lambda: ch.drops_total == 7, 5)
            said = [r.getMessage() for r in caplog.records if r.levelno == logging.WARNING]
            assert len(said) == 4 and "(3 more since the last one said here)" in said[-1], said
            assert len(path.read_text().splitlines()) == 7

            dlc.execute("(param-set! 'laser1:dl:lock:lock-enabled #f)")  # turned off by hand: not a dropout
            assert wait_for(lambda: ch.cavity_on is False, 3)
            time.sleep(1.0)
            assert ch.drops_total == 7
        finally:
            e.stop()
    assert dlc.params["laser1:recorder:inputs:channel1:low-pass-filter:enabled"] is True  # put back


def test_no_monitor_unless_wanted(quiet_demo):  # noqa: F811
    demo, dlc = quiet_demo, quiet_demo.dlcs[2]
    e, ch = hop_engine(demo, demo.SHG, dlc.key, 1, dict(enabled=True, cavity=True, shg=True, dropouts=False))
    try:
        assert demo.SHG not in e.monitors
        e.set_hop_params(demo.SHG, HopParams(enabled=True, cavity=True, shg=True))
        assert demo.SHG in e.monitors
        e.unlink(demo.SHG)
        assert demo.SHG not in e.monitors
    finally:
        e.stop()

