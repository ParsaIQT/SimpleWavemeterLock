"""Laser maps against a simulated external-cavity diode laser, its DLC pro and the wavemeter's detailed view of it
(auto exposure, fringes, patterns), driven synchronously on the bench's own clock: the map talks to a FakeDLCPro
through the real client, in the same thread.

Every run checks the safety rules: the measured current never above the plan's highest (or where it began), the
set current never above current-clip, the piezo within the plan and its limits, every write a step, and nothing
written but current, piezo, temperature and the scan's switch."""
import math
import random
import statistics

import pytest

from wmlock.dlcpro import DecopError, DLCPro
from wmlock.lasermap import (STEP_I, STEP_V, Leg, MapFile, MapPlan, Mapper, MapRunner, contrast, file_name, grid,
                             load, pack, unpack)
from wmlock.modehop import perform
from wmlock.sim import ECDL, FakeDLCPro, SimCCD
from wmlock.wavemeter import WLM_ERRORS, Reading

I0, V0, F0 = 150.0, 70.0, 384230.0


class Loopback(DLCPro):
    """A DLCPro client for a FakeDLCPro in the same thread: no sockets."""

    def __init__(self, fake):
        super().__init__("loopback")
        self.fake = fake

    def command(self, cmd: str) -> str:
        return self.fake.execute(cmd).strip()


class MapBench:
    """A laser on its controller, and the wavemeter's detailed view of it (SimCCD), on the bench's time:
    operations land at once (their answers come back one at a time); a reading takes its exposure, measures the
    laser as it was when it began, and is handed on at its end, if its value changed (as a source does)."""

    def __init__(self, laser=None, current=I0, offset=V0, ff=0.0, scan=False, pd=True, temp=20.0, patterns=True,
                 seed=0, clip=200.0, act_offset=0.0, ta=None, locked=False, noise=0.5):
        self.t = 0.0
        self.laser = laser if laser is not None else ECDL(F0, i_stop=100.0, i_start=103.0, seed=seed)
        cfg = {"current": current, "offset": offset, "scan": scan, "feedforward": ff, "temp": temp, "clip": clip,
               "act_offset": act_offset, "locked": locked}
        self.dlc = FakeDLCPro("SN-M", "map bench", {1: cfg}, serve=False, clock=lambda: self.t)
        self.client = Loopback(self.dlc)
        self.ccd = SimCCD(seed=seed)
        self.t_ref = temp
        self.rng = random.Random(seed)
        self.noise = noise
        self.patterns = patterns
        self.pending = []
        self.later = []    # (bench time, an answer): operations answered late (slow)
        self.slow = {}     # n -> s: the nth read (Get) answered that much later
        self.gets = 0
        self.last = None   # the raw value last handed on
        self.taken = 0     # readings measured
        if pd:
            self.dlc.params["laser1:dl:pd:photodiode"] = lambda: self.laser.photodiode(self.act)
        if ta is not None:
            self.dlc.params["laser1:amp:cc:enabled"] = ta
        for name in ("dl:cc:current-set", "scan:offset"):  # the laser follows each change as it is made
            self.dlc.on_set["laser1:" + name] = self._settle
        self._settle(None)

    def _settle(self, _value):
        self.laser.T = self.dlc.temp_act(1) - self.t_ref
        self.laser.lase(self.V, self.act)

    @property
    def I(self):  # noqa: E743
        return self.dlc.params["laser1:dl:cc:current-set"]

    @property
    def V(self):
        return self.dlc.params["laser1:scan:offset"]

    @property
    def act(self):
        return self.dlc.current_act(1)

    def do(self, op, done, failed):
        try:
            result = perform(self.client, 1, op)
        except DecopError as exc:
            refused = exc
            self.pending.append(lambda: failed(refused))
            return
        if op[0] == "get":
            self.gets += 1
            if self.gets in self.slow:
                self.later.append((self.t + self.slow[self.gets], lambda: done(result)))
                return
        self.pending.append(lambda: done(result))

    def reading(self):
        """The next reading: (raw value, extra, s it took), the laser as it is now."""
        self._settle(None)
        v, i = self.V, self.act
        f = self.laser.freq(v, i)
        comps = self.laser.components(v, i) if f == f else []
        f = f + self.rng.gauss(0.0, self.noise) * 1e-6 if f == f else f
        value, extra, ms = self.ccd.measure(comps, self.laser.power(i), i, f, self.laser.f0, self.patterns)
        return value, extra, ms / 1e3

    def run(self, mapper, until=None, cancel_at=None, limit=5e6, tick=0.01):
        """Drive the map to its end: time passes `tick` at a time (as the engine's ticks), a reading going on all
        along, handed on at its end."""
        runner = MapRunner(mapper, self, self.t, exposure=lambda: self.exposing)
        self.runner = runner
        steps = 0
        value, extra, took = self.reading()
        end, self.exposing = self.t + took, took
        while runner.busy and (until is None or not until(runner)):
            steps += 1
            assert steps < limit, "the map does not end"
            if cancel_at is not None and self.t >= cancel_at and runner.state == "running":
                runner.cancel("stopped")
            if self.pending:
                self.pending.pop(0)()
                continue
            due = [a for a in self.later if a[0] <= self.t]
            if due:
                self.later.remove(due[0])
                due[0][1]()
                continue
            self.t = min(end, self.t + tick)
            if self.t < end:
                runner.tick(self.t)
                continue
            self.taken += 1
            if value != self.last:
                self.last = value
                status = "" if value > 0 else WLM_ERRORS.get(int(value), "error")
                runner.on_reading(Reading(1, value if value > 0 else math.nan, status, self.t, extra))
            else:
                runner.tick(self.t)
            value, extra, took = self.reading()  # (the next one begins: the laser as it is now)
            end, self.exposing = self.t + took, took  # (its exposure, as the wavemeter says it meanwhile)
        return runner

    def writes(self, name):
        return self.dlc.written("laser1:" + name)


def plan(**kw) -> MapPlan:
    d = dict(i_lo=95.0, i_hi=160.0, i_step=0.5, v_lo=60.0, v_hi=80.0, v_step=4.0, reads=2, settle=0.02, dark=3,
             patterns="hops", every=10, repeat=True)
    d.update(kw)
    return MapPlan(**d)


def check_rules(b: MapBench, p: MapPlan, start=(I0, V0)):
    """The safety rules over everything written."""
    d = b.dlc.params
    off = b.dlc.bias[1] + d["laser1:dl:cc:current-offset"]
    ff = d["laser1:dl:cc:feedforward-factor"] if d["laser1:dl:cc:feedforward-enabled"] else 0.0
    i_set, v = start
    top = max(p.i_hi, start[0] + off + ff * start[1])
    for name, value in b.dlc.log:
        assert name in ("laser1:dl:cc:current-set", "laser1:scan:offset", "laser1:dl:tc:temp-set",
                        "laser1:scan:enabled"), name
        if name.endswith("current-set"):
            assert abs(value - i_set) <= STEP_I + 1e-6
            i_set = value
            assert value <= d["laser1:dl:cc:current-clip"] + 1e-9
        elif name.endswith("scan:offset"):
            assert abs(value - v) <= STEP_V + 1e-6
            v = value
            assert min(p.v_lo, start[1]) - 1e-9 <= v <= max(p.v_hi, start[1]) + 1e-9
        assert i_set + off + ff * v <= top + 1e-6, (name, value)


# ------------------------------------------------------------------ the plan
def test_the_grid_has_both_ends_and_steps_no_larger_than_asked():
    assert grid(95.0, 100.0, 0.5) == [95.0 + 0.5 * k for k in range(11)]
    g = grid(0.0, 1.0, 0.3)
    assert g[0] == 0.0 and g[-1] == 1.0 and len(g) == 5 and max(b - a for a, b in zip(g, g[1:])) <= 0.3
    assert grid(5.0, 5.0, 1.0) == [5.0]


def test_a_plan_says_what_is_wrong_with_it():
    assert any("highest current" in s for s in MapPlan().problems())
    assert plan().problems() == []
    assert any("below the highest" in s for s in plan(i_lo=170.0).problems())
    assert any("step" in s for s in plan(i_step=6.0).problems())
    limits = {"clip": 155.0, "offset": 0.0, "ff": 0.0, "v0": 70.0, "pc": (0.0, 140.0), "tc": (15.0, 35.0), "t0": 20.0}
    assert any("current-clip" in s for s in plan().problems(limits))
    limits["clip"] = 200.0
    assert plan().problems(limits) == []
    assert any("beyond its limits" in s for s in plan(v_hi=150.0).problems(limits))
    assert any("temperature" in s for s in plan(t_lo=19.0, t_hi=40.0, t_step=1.0).problems(limits))
    # with the feedforward the set current for the measured one depends on the piezo: the worst case counts
    limits.update(clip=161.0, ff=-0.2)
    assert any("current-clip" in s for s in plan(v_lo=60.0, v_hi=80.0).problems(limits))
    p = plan()
    assert MapPlan.from_dict(p.to_dict()) == p
    s, size = p.estimate(100.0, 4000.0)
    assert 0 < s < 1e5 and size > 0


def test_patterns_are_packed_losslessly():
    rng = random.Random(1)
    for dtype, values in (("H", [rng.randrange(65536) for _ in range(3000)]),
                          ("H", [int(2000 + 1500 * math.cos(x / 7.0)) for x in range(2048)]),
                          ("I", [rng.randrange(1 << 32) for _ in range(500)]),
                          ("d", [rng.random() for _ in range(300)])):
        codec, blob = pack(values, dtype)
        assert unpack(codec, dtype, blob) == values
    smooth = [int(2000 + 1500 * math.cos(x / 7.0)) for x in range(2048)]
    assert len(pack(smooth, "H")[1]) < 0.5 * 2 * 2048  # (fringes compress well)


def test_a_sweep_says_where_it_hops_goes_dark_and_its_fringes_fade():
    leg = Leg()
    events = [leg.add(F0 + k * 1e-5, False, None, 0.9) for k in range(10)]  # 10 MHz a step
    assert events == [""] * 10
    assert leg.add(F0 + 0.006, False, None, 0.9) == "hop"
    assert leg.add(None, True, None, None) == "dark"
    assert leg.add(F0 + 0.0061, False, None, 0.9) == "lasing"
    assert leg.add(F0 + 0.00611, False, 80.0, 0.9) == "spread"
    assert leg.add(F0 + 0.00612, False, None, 0.5) == "fade"
    assert leg.add(None, False, None, None) == "invalid"
    assert contrast([100, 300]) == 0.5 and contrast(None) is None


# ----------------------------------------------------------------- the map
def test_a_map_sweeps_both_ways_keeps_to_its_bounds_and_goes_back(tmp_path):
    b = MapBench(scan=True)
    p = plan()
    out = MapFile(tmp_path / "m.sqlite")
    m = Mapper(p, out, {"laser": {"channel": 3, "name": "bench"}})
    r = b.run(m)
    out.close()
    assert r.state == "done", r.error
    check_rules(b, p)
    assert b.writes("scan:enabled") == [False, True]  # off for the map, on again after
    assert abs(b.act - I0) < 1e-9 and abs(b.V - V0) < 1e-9
    with load(out.path) as f:
        assert f.meta["plan"]["i_hi"] == 160.0 and f.meta["laser"]["channel"] == 3
        assert f.meta["start"]["current_act"] == pytest.approx(I0)
        lines = [ln for ln in f.lines if not ln["repeat"]]
        assert len(lines) == len(p.grid("piezo"))
        assert any(ln["repeat"] for ln in f.lines)
        legs = f.legs()
        for line in lines:
            ups = [pts for ln, leg, pts in legs if ln["id"] == line["id"] and leg == 1]
            downs = [pts for ln, leg, pts in legs if ln["id"] == line["id"] and leg == 2]
            assert ups and downs
            up = ups[0]
            assert up[-1]["i"] == pytest.approx(160.0)  # up to the top...
            first = next(q for q in up if q["f"] is not None)
            assert 102.5 <= first["i"] <= 104.0  # ...lasing from where it starts again (above where it stops)
            down = downs[0]
            last = [q for q in down if q["f"] is not None][-1]
            assert 99.5 <= last["i"] <= 101.0  # sweeping down it lases on to where it stops: turned back soon after
            assert down[-1]["i"] >= last["i"] - 0.5 * (p.dark + 1) - 1e-9
        assert all(95.0 - 1e-9 <= q["i"] <= 160.0 + 1e-9 for q in f.points)
        assert all(q["n"] >= 1 for q in f.points if q["f"] is not None)
        kept = f.db.execute("SELECT COUNT(DISTINCT point_id) FROM patterns").fetchone()[0]
        assert 0 < kept < len(f.points)  # around hops and the dark, and every 10th point
        hop = next(q for q in f.points if q["event"] == "hop")
        assert set(f.patterns(hop["id"])) == {0, 1} and len(f.patterns(hop["id"])[0]) == 4 * SimCCD.PIXELS
        assert f.readings(hop["id"]) and f.events
    assert "back at" in m.result


def mode_changes(pts, laser) -> dict:
    """Where the laser changes mode along a sweep, as read from the map: {(from mode, to mode): mA where the new one
    is first seen}, the mode of each point from its frequency less the simulated laser's own tuning (the median of
    three, past the nonsense two modes give), a change only if the new mode holds for three points."""
    ms = []
    for q in pts:
        if q["f"] is None:
            ms.append(None)
            continue
        ghz = (q["f"] - laser.f0) * 1e3 - laser.ki * (q["i"] - laser.i0) - laser.kv * (q["v"] - laser.v0)
        ms.append(round(ghz / laser.fsr))
    smooth = [statistics.median_low([m for m in ms[max(k - 1, 0):k + 2] if m is not None] or [None])
              if ms[k] is not None else None for k in range(len(ms))]
    out, now = {}, None
    for k, m in enumerate(smooth):
        if m is None:
            continue
        if now is not None and m != now and smooth[k:k + 3] == [m] * len(smooth[k:k + 3]) and len(smooth[k:k + 3]) == 3:
            out[(now, m)] = pts[k]["i"]
        if now is None or smooth[k:k + 3] == [m] * 3:
            now = m
    return out


def test_hysteresis_shows_as_modes_changing_later_sweeping_up_than_down(tmp_path):
    """The simulated laser hops off a mode 0.69 mA later sweeping the current up than it hops back onto it sweeping
    down (its hysteresis, 1.2 GHz of misalignment at 1.75 GHz/mA): the map keeps the two ways apart and shows it."""
    b = MapBench(patterns=False)
    p = plan(v_lo=66.0, v_hi=74.0, v_step=4.0, i_lo=120.0, i_step=0.05, repeat=False, patterns="off")
    out = MapFile(tmp_path / "m.sqlite")
    r = b.run(Mapper(p, out))
    out.close()
    assert r.state == "done", r.error
    diffs = []
    with load(out.path) as f:
        legs = f.legs()
        for line, leg, pts in legs:
            if leg != 1:
                continue
            down = next(q for ln, lg, q in legs if ln["id"] == line["id"] and lg == 2)
            up_changes, down_changes = mode_changes(pts, b.laser), mode_changes(down, b.laser)
            assert len(up_changes) >= 2
            for (m, n), i in up_changes.items():  # (up: onto the next mode down; down: back)
                if (n, m) in down_changes:
                    diffs.append(i - down_changes[(n, m)])
        assert len(diffs) >= 6 and 0.5 < statistics.median(diffs) < 0.9
        assert sum(q["event"] == "hop" for q in f.points) >= len(diffs)


def test_dark_points_end_quickly_once_its_photodiode_tells_dark_from_lasing(tmp_path):
    b = MapBench(patterns=False)
    r = b.run(Mapper(plan(patterns="off", repeat=False), None))
    assert r.state == "done", r.error
    m = r.mapper
    assert m.pd == "dl:pd:photodiode" and m.pd_level() is not None
    b2 = MapBench(patterns=False, pd=False)
    r2 = b2.run(Mapper(plan(patterns="off", repeat=False), None))
    assert r2.state == "done" and r2.mapper.pd == ""
    assert b.t < 0.7 * b2.t  # (without its photodiode, each dark point waits as long as a reading would take)


def test_the_piezo_swept_at_each_current(tmp_path):
    b = MapBench(patterns=False)
    p = plan(sweep="piezo", i_lo=140.0, i_hi=150.0, i_step=2.0, v_lo=50.0, v_hi=90.0, v_step=1.0, patterns="off")
    out = MapFile(tmp_path / "m.sqlite")
    r = b.run(Mapper(p, out))
    out.close()
    assert r.state == "done", r.error
    check_rules(b, p)
    with load(out.path) as f:
        assert {ln["axis"] for ln in f.lines} == {"piezo"}
        for _line, leg, pts in f.legs():
            xs = [q["v"] for q in pts]
            assert xs == sorted(xs, reverse=leg == 2)
            assert len({q["i"] for q in pts}) == 1
        assert sum(q["event"] == "hop" for q in f.points) > 4


def test_with_the_feedforward_on_the_measured_current_is_as_mapped(tmp_path):
    b = MapBench(ff=-0.15, patterns=False, act_offset=0.3)
    p = plan(sweep="both", i_lo=145.0, i_hi=152.0, i_step=1.0, v_lo=60.0, v_hi=80.0, v_step=5.0, patterns="off")
    out = MapFile(tmp_path / "m.sqlite")
    r = b.run(Mapper(p, out))
    out.close()
    assert r.state == "done", r.error
    check_rules(b, p, start=(I0, V0))
    with load(out.path) as f:
        assert {ln["axis"] for ln in f.lines} == {"current", "piezo"}
        for q in f.points:  # what was set has the measured current (set + 0.3 mA + ff times the piezo) as mapped
            assert q["i_set"] + 0.3 - 0.15 * q["v"] == pytest.approx(q["i"], abs=1e-6)
        assert f.meta["start"]["feedforward"] == -0.15
    assert abs(b.act - (I0 + 0.3 - 0.15 * V0)) < 1e-6 and abs(b.V - V0) < 1e-9


def test_the_temperature_stepped_waits_for_it_to_settle_and_goes_back(tmp_path):
    b = MapBench(patterns=False)
    p = plan(t_lo=19.5, t_hi=20.5, t_step=0.5, t_hold=3.0, t_tol=0.01, v_lo=68.0, v_hi=72.0, v_step=4.0,
             i_lo=140.0, i_step=1.0, patterns="off", repeat=False)
    out = MapFile(tmp_path / "m.sqlite")
    r = b.run(Mapper(p, out))
    out.close()
    assert r.state == "done", r.error
    assert b.writes("dl:tc:temp-set") == [19.5, 20.0, 20.5, 20.0]
    with load(out.path) as f:
        temps = sorted({q["temp"] for q in f.points})
        assert temps == [19.5, 20.0, 20.5]
        assert sum("settled" in e["text"] for e in f.events) == 4
        assert f.temps and all(t["act_c"] is not None for t in f.temps)
    assert abs(b.dlc.temp_act(1) - 20.0) < 0.02


def test_stopped_it_puts_everything_back(tmp_path):
    b = MapBench(scan=True, patterns=False)
    p = plan(t_lo=19.0, t_hi=21.0, t_step=1.0, t_hold=1.0, patterns="off")
    out = MapFile(tmp_path / "m.sqlite")
    r = b.run(Mapper(p, out), cancel_at=40.0)
    out.close()
    assert r.state == "failed" and r.error == "stopped" and r.restored
    check_rules(b, p)
    assert abs(b.act - I0) < 1e-9 and abs(b.V - V0) < 1e-9
    assert b.dlc.params["laser1:dl:tc:temp-set"] == 20.0 and b.dlc.params["laser1:scan:enabled"] is True
    with load(out.path) as f:
        assert f.points  # what it had


@pytest.mark.parametrize("what", ["ta", "locked", "clip"])
def test_a_laser_it_must_not_map_is_left_alone(what, tmp_path):
    b = MapBench(ta=True if what == "ta" else None, locked=what == "locked", clip=155.0 if what == "clip" else 200.0)
    r = b.run(Mapper(plan(), None))
    assert r.state == "failed"
    assert {"ta": "TA is on", "locked": "locks it to a cavity", "clip": "current-clip"}[what] in r.error
    assert b.dlc.log == []


def test_file_names_say_when_and_which_laser():
    name = file_name(3, "866 repump", 0.0)
    assert name.startswith("map_19") and name.endswith("_ch3_866-repump.sqlite")


def test_the_command_line_says_what_a_map_holds_and_writes_its_points_as_csv(tmp_path, capsys):
    import csv

    from wmlock.lasermap import main

    b = MapBench()
    out = MapFile(tmp_path / "m.sqlite")
    r = b.run(Mapper(plan(v_lo=68.0, v_hi=72.0, i_lo=140.0, i_step=1.0), out,
                     {"laser": {"channel": 3, "name": "bench", "device": "1.2.3.4", "laser": 1},
                      "wavemeter": {"model": "WS7", "serial": 1234}}))
    out.meta("end", {"outcome": "map done", "ok": True})
    out.close()
    assert r.state == "done", r.error
    assert main([str(out.path), "--csv"]) == 0
    said = capsys.readouterr().out
    assert "channel 3 (bench), 1.2.3.4 laser1" in said and "wavemeter WS7 1234" in said
    assert "plan: current 140 … 160 mA (measured) in 1 mA, piezo 68 … 72 V in 4 V, sweeping current" in said
    assert "lasing" in said and "hops:" in said and "ended: map done" in said
    rows = list(csv.reader(open(tmp_path / "m.csv", encoding="utf-8")))
    with load(out.path) as f:
        assert len(rows) == len(f.points) + 1
    assert rows[0][:5] == ["id", "line_id", "leg", "k", "t"] and "dl:pd:photodiode" in rows[0]


def test_a_wavemeter_that_says_nothing_more_stops_the_map_and_it_is_put_back(tmp_path):
    b = MapBench(patterns=False)
    p = plan(patterns="off")
    out = MapFile(tmp_path / "m.sqlite")
    quiet = []

    def silence(r):  # (its channel gone, say: the wavemeter reads it no more)
        if r.mapper.points > 30:
            quiet.append(True)
        return False

    real = b.reading

    def reading():
        value, extra, took = real()
        return (b.last, extra, took) if quiet else (value, extra, took)  # (nothing new: nothing handed on)

    b.reading = reading
    r = b.run(Mapper(p, out), until=silence)
    out.close()
    assert r.state == "failed" and "no readings of it, 3 points in a row" in r.error and r.restored
    assert abs(b.act - I0) < 1e-9 and abs(b.V - V0) < 1e-9


def test_a_read_answered_too_late_for_its_point_is_dropped(tmp_path):
    """The photodiode's read at one point answered 7 s late (the controller busy, say): the point goes on without it
    after 5 s, and the late answer, coming during another point, is dropped, not taken for anything else."""
    b = MapBench(patterns=False)
    b.slow = {20: 7.0}
    out = MapFile(tmp_path / "m.sqlite")
    r = b.run(Mapper(plan(patterns="off", v_lo=68.0, v_hi=72.0), out))
    out.close()
    assert r.state == "done", r.error
    check_rules(b, plan(patterns="off", v_lo=68.0, v_hi=72.0))
    with load(out.path) as f:
        waited = [q for q in f.points if q["wait"] >= 4.9]
        assert len(waited) == 1 and waited[0]["wait"] < 6.0  # (that one point waited for it, and none other)
        assert [q["id"] for q in f.points if not q["vals"]] == [waited[0]["id"]]
