import math
import random
import re
import statistics

import pytest

from wmlock.reference import ACQUIRING, HOLD, MOVED, TRACKING, Reference, RefParams

F0 = 411.0421  # THz, a 729 nm laser locked to a cavity
F_OTHER = 755.2227  # THz, some other laser on the wavemeter


class Bench:
    """Synthetic wavemeter readings of the reference laser.

    wlm(t): wavemeter error at F0 (MHz), laser(t): the laser's own offset from F0 (MHz),
    valid(t): whether the wavemeter gives a reading at all."""

    def __init__(self, ref, wlm=lambda t: 0.0, laser=lambda t: 0.0, valid=lambda t: True,
                 noise=0.3, rate=5.0, seed=1):
        self.ref, self.wlm, self.laser, self.valid = ref, wlm, laser, valid
        self.noise, self.dt, self.rng = noise, 1.0 / rate, random.Random(seed)
        self.t = 0.0
        self.states = []

    def raw(self, f_true, t):
        """What the wavemeter shows for a laser truly at f_true."""
        return f_true * (1 + self.wlm(t) * 1e-6 / F0)

    def run(self, seconds):
        end = self.t + seconds
        while self.t < end:
            self.t += self.dt
            if self.valid(self.t):
                f = self.raw(F0 + self.laser(self.t) * 1e-6, self.t) + self.rng.gauss(0, self.noise) * 1e-6
            else:
                f = math.nan
            self.ref.update(self.t, f)
            self.ref.tick(self.t)
            self.states.append(self.ref.state)
        return self

    def until(self, cond, limit):
        """Run until cond() holds; returns the time it took, which must be under `limit` s."""
        start = self.t
        while not cond():
            assert self.t - start < limit, f"not reached within {limit} s: {self.ref.state} {self.ref.message!r}"
            self.run(self.dt)
        return self.t - start

    def other_corrected(self):
        """Corrected frequency of the other laser (truly constant), MHz from its true value."""
        return (self.ref.correct(self.raw(F_OTHER, self.t)) - F_OTHER) * 1e6


def make(**kw):
    return Reference(4, RefParams(**kw), now=0.0)


def test_acquires_then_tracks_slow_wavemeter_drift():
    b = Bench(make(), wlm=lambda t: t / 60.0, noise=0.5)  # 1 MHz/min
    assert b.until(lambda: b.ref.state == TRACKING, 40) > Reference.STEADY_S
    t0, start, n = b.t, b.other_corrected(), len(b.states)  # anchored on the reading at t0
    b.run(100)
    t1, e1 = b.t, b.ref.error
    b.run(500)
    assert set(b.states[n:]) == {TRACKING}
    # the raw reading of the other laser drifted by ~18 MHz; the corrected one did not
    assert (b.raw(F_OTHER, b.t) - b.raw(F_OTHER, t0)) * 1e6 == pytest.approx(10 * F_OTHER / F0, rel=0.05)
    assert abs(b.other_corrected() - start) < 0.8  # vs 18 MHz raw: only the anchor fit uncertainty is left
    assert b.ref.error - e1 == pytest.approx(b.wlm(b.t) - b.wlm(t1), abs=0.3)
    assert b.ref.rate * 60 == pytest.approx(1.0, abs=0.15)  # it knows how fast the wavemeter drifts


def test_follows_a_ramp_without_lag():
    """A plain low-pass would trail a 1 MHz/min drift by rate * tau = 0.5 MHz."""
    b = Bench(make(), wlm=lambda t: t / 60.0, noise=0.0)
    b.until(lambda: b.ref.state == TRACKING, 30)
    t0 = b.t
    b.run(300)
    assert b.ref.error == pytest.approx(b.wlm(b.t) - b.wlm(t0), abs=0.02)
    assert b.ref.rate * 60 == pytest.approx(1.0, abs=0.1)


def test_noise_is_averaged_out():
    b = Bench(make(), noise=1.0)
    b.until(lambda: b.ref.state == TRACKING, 60)
    b.run(60)
    errors = []
    for _ in range(300):
        b.run(1)
        errors.append(b.ref.error)
    assert statistics.pstdev(errors) < 0.2
    assert 0.8 < b.ref.sigma < 1.2


def tracking(**kw):
    """A reference that has been tracking for a while (long enough for a settled history, 3 tau)."""
    b = Bench(make(), **kw)
    b.until(lambda: b.ref.state == TRACKING, 30)
    b.run(120)
    return b


def test_single_glitch_is_ignored():
    b = tracking()
    before = b.ref.error
    t0 = b.t
    b.laser = lambda t: 500.0 if t0 < t <= t0 + 0.2 else 0.0
    b.run(5)
    assert set(b.states[-25:]) == {TRACKING}
    assert b.ref.error == pytest.approx(before, abs=0.1)


def test_mode_hop_holds_then_waits_for_the_user():
    b = tracking()
    before, correction, t0 = b.ref.error, b.ref.correction_mhz(F_OTHER), b.t
    b.laser = lambda t: 1500.0 if t > t0 else 0.0
    b.run(1)
    assert b.ref.state == HOLD and re.search(r"jumped \+1[45]\d\d\.\d MHz", b.ref.message)
    b.until(lambda: b.ref.state == MOVED, 30)
    assert b.ref.offset == pytest.approx(1500, abs=2)
    assert b.ref.error == before
    assert b.ref.correction_mhz(F_OTHER) == correction  # the locks never saw the hop
    anchor = b.ref.anchor
    assert b.ref.reanchor(b.t)
    assert b.ref.state == TRACKING and b.ref.error == before
    assert (b.ref.anchor - anchor) * 1e6 == pytest.approx(1500, abs=2)
    b.run(30)
    assert set(b.states[-150:]) == {TRACKING}


def test_mode_hop_that_comes_back_resumes_by_itself():
    b = tracking()
    before, t0 = b.ref.error, b.t
    b.laser = lambda t: 1500.0 if t0 < t < t0 + 40 else 0.0
    b.run(30)
    assert b.ref.state == MOVED
    b.until(lambda: b.ref.state == TRACKING, 30)
    assert "recovered" in b.ref.message
    assert b.ref.error == pytest.approx(before, abs=0.5)


def test_cavity_unlock_sweep_is_ignored():
    b = tracking()
    before, t0, n = b.ref.error, b.t, len(b.states)
    b.laser = lambda t: 400 * math.sin(2 * math.pi * t / 3) if t0 < t < t0 + 30 else 0.0
    b.run(30)
    assert b.ref.state == HOLD and MOVED not in b.states[n:]  # never steady while sweeping
    assert abs(b.ref.error - before) < 0.1  # the odd reading at a sweep's zero crossing is harmless
    b.until(lambda: b.ref.state == TRACKING, 30)
    assert b.ref.error == pytest.approx(before, abs=0.5)


@pytest.mark.parametrize("mhz_per_min, pull", [(120, 0.3), (12, 0.5), (4, 0.5)])
def test_runaway_is_caught_before_it_pulls_the_estimate_far(mhz_per_min, pull):
    """Cavity lost, the laser drifts off. Fast: the gate catches it. Slow: the rate bound does."""
    b = tracking()
    before, t0 = b.ref.error, b.t
    b.laser = lambda t: mhz_per_min / 60 * (t - t0) if t > t0 else 0.0
    b.until(lambda: b.ref.state != TRACKING, 120)
    assert abs(b.ref.error - before) < pull
    b.run(60)  # still running away: never believed again
    assert b.ref.state != TRACKING


def test_no_light_and_silence_put_it_on_hold():
    b = tracking()
    t0 = b.t
    b.valid = lambda t: not t0 < t < t0 + 10
    b.run(1)
    assert b.ref.state == HOLD and b.ref.message == "no signal"
    b.run(9)
    b.until(lambda: b.ref.state == TRACKING, 30)
    # the channel stops being measured altogether
    ref, t = b.ref, b.t
    ref.tick(t + 9)
    assert ref.state == TRACKING
    ref.tick(t + 11)
    assert ref.state == HOLD and ref.message == "no readings"


def test_wavemeter_drift_during_an_outage_is_taken_up():
    b = tracking()
    t0 = b.t
    b.wlm = lambda t: 0.0 if t < t0 else (t - t0) / 60.0  # 1 MHz/min from here on
    b.valid = lambda t: not t0 < t < t0 + 300  # 5 min dark: the wavemeter drifts 5 MHz meanwhile
    b.run(300)
    b.until(lambda: b.ref.state == TRACKING, 60)
    assert "recovered after 5 min" in b.ref.message
    assert b.ref.error == pytest.approx(b.wlm(b.t), abs=0.5)


def test_a_bigger_change_than_the_wavemeter_can_drift_needs_the_user():
    b = tracking()
    t0 = b.t
    b.wlm = lambda t: 0.0 if t < t0 else 20.0
    b.valid = lambda t: not t0 < t < t0 + 30
    b.run(30)
    b.until(lambda: b.ref.state == MOVED, 30)
    assert b.ref.offset == pytest.approx(20, abs=1)
    assert b.ref.accept_as_drift(b.t)
    assert b.ref.state == TRACKING and b.ref.error == pytest.approx(20, abs=1)
    assert not b.ref.reanchor(b.t)  # only while MOVED


def test_restart_holds_the_saved_estimate_until_confirmed():
    saved = tracking(wlm=lambda t: 12.0).ref
    saved.anchor = F0  # pretend the anchor was the laser's true frequency
    saved.error = 12.0
    cfg = saved.to_config()
    cfg["confirmed_at"] -= 3600  # an hour ago
    ref = Reference.from_config(cfg, now=0.0)
    assert ref.state == HOLD and ref.correction_mhz(F0) == pytest.approx(-12.0, abs=1e-3)
    b = Bench(ref, wlm=lambda t: 14.0)  # drifted 2 MHz in an hour: fine
    b.until(lambda: ref.state == TRACKING, 30)
    assert ref.error == pytest.approx(14.0, abs=0.5)
    ref = Reference.from_config(cfg, now=0.0)
    b = Bench(ref, wlm=lambda t: 400.0)  # 400 MHz away: not plausible
    b.until(lambda: ref.state == MOVED, 30)


def test_offset_model_corrects_every_channel_by_the_same_amount():
    b = Bench(make(scale=False))
    b.until(lambda: b.ref.state == TRACKING, 30)
    t0 = b.t
    b.wlm = lambda t: min(t - t0, 480) / 60.0  # 8 MHz at 1 MHz/min
    b.run(600)
    assert b.ref.correction_mhz(F0) == pytest.approx(-8.0, abs=0.3)
    assert b.ref.correction_mhz(F_OTHER) == pytest.approx(b.ref.correction_mhz(F0), abs=1e-6)
    b.ref.p.scale = True
    assert b.ref.correction_mhz(F_OTHER) == pytest.approx(-8.0 * F_OTHER / F0, abs=0.6)


def test_new_reference_continues_the_old_correction():
    b = tracking()
    t0 = b.t
    b.wlm = lambda t: min(t - t0, 600) / 60.0  # 10 MHz at 1 MHz/min
    old = b.run(700).ref
    before = old.correction_mhz(F_OTHER)
    assert before == pytest.approx(-10 * F_OTHER / F0, abs=0.5)
    new = Reference(2, RefParams(), carry=old.eps_offset(), now=0.0)
    assert new.state == ACQUIRING and new.active
    assert new.correction_mhz(F_OTHER) == before
    f_new = 346.0  # the new reference laser, read with the same wavemeter error
    for i in range(100):
        new.update(i * 0.2, f_new * (1 + 10e-6 / F0))
    assert new.state == TRACKING
    assert new.correction_mhz(F_OTHER) == pytest.approx(before, abs=1e-3)


def test_typed_anchor_measures_the_wavemeter_error():
    b = tracking(wlm=lambda t: 30.0)
    assert b.ref.error == pytest.approx(0, abs=0.5)  # anchored on its own reading
    b.ref.set_anchor(F0, b.t)
    assert b.ref.state == ACQUIRING and b.ref.active  # old correction kept meanwhile
    b.until(lambda: b.ref.state == TRACKING, 30)
    assert b.ref.error == pytest.approx(30.0, abs=0.5)
    assert b.other_corrected() == pytest.approx(0, abs=1.0)  # now absolutely calibrated


def test_fresh_reference_applies_no_correction_until_acquired():
    ref = make()
    assert not ref.active and ref.correct(F_OTHER) == F_OTHER
    assert RefParams.from_dict({"scale": 0, "tau": "10", "junk": 1}) == RefParams(tau=10.0, scale=False)


def test_preview_predicts_the_correction_after_a_change():
    b = tracking(wlm=lambda t: 0.0)
    t0 = b.t
    b.wlm = lambda t: min(t - t0, 300) / 60.0  # +5 MHz
    b.run(400)
    ref, f = b.ref, b.raw(F_OTHER, b.t)
    assert ref.preview(f) == ref.correct(f)
    off = ref.preview(f, scale=False)
    ref.p.scale = False
    assert ref.correct(f) == pytest.approx(off, abs=1e-12)
    ref.p.scale = True
    new_anchor = ref.anchor + 100e-6
    expected = ref.preview(f, anchor=new_anchor)
    ref.set_anchor(new_anchor, b.t)
    b.until(lambda: ref.state == TRACKING, 30)
    assert (ref.correct(b.raw(F_OTHER, b.t)) - expected) * 1e6 == pytest.approx(0, abs=0.5)
