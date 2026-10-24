import math

import pytest

from wmlock.lock import Lock, LockParams


def run_plant(lock, k_ghz_per_v, f_err0_mhz, steps, u0=70.0):
    """Static plant: error = f_err0 + K * (u - u0)."""
    lock.engage(u0)
    u, errs = u0, []
    for _ in range(steps):
        e = f_err0_mhz + k_ghz_per_v * 1e3 * (u - u0)
        errs.append(e)
        new = lock.update(e)
        if new is not None:
            u = new
    return u, errs


@pytest.mark.parametrize("k, ki", [(0.35, 1.0), (-0.3, -1.0)])
def test_integrator_converges_for_either_sign(k, ki):
    lock = Lock(LockParams(ki=ki, max_step=10))
    _, errs = run_plant(lock, k, 200.0, 60)
    assert abs(errs[-1]) < 0.01
    assert lock.active


@pytest.mark.parametrize("err0, max_step", [(5.0, 0.25), (100.0, 0.25), (1000.0, 0.25), (-40.0, 10.0)])
def test_a_lock_that_runs_away_reverses_its_gains_once_and_locks(err0, max_step):
    """Its gains of the wrong sign for the laser (its frequency falls as the offset rises): it runs away, faster and
    faster (or as fast as Max step lets it, from far), and is caught at it: back where the error was smallest, its
    gains reversed, and locked."""
    lock = Lock(LockParams(ki=1.0, max_step=max_step, excursion=5.0))
    u, errs = run_plant(lock, -0.3, err0, 200)
    assert lock.active and lock.reversed and abs(errs[-1]) < 0.01, lock.message
    assert (lock.p.kp, lock.p.ki) == (0.0, -1.0)
    assert max(abs(e) for e in errs) < abs(err0) + max(30.0, min(abs(err0), 300.0)) + 100.0
    assert lock.note.startswith("it ran away (") and "its gains are reversed (P 0, I -1)" in lock.note


def test_negative_gains_where_the_frequency_rises_with_the_offset_are_reversed_too():
    """A channel's settings stay with the channel: a laser put on one set up for a laser that tuned the other way."""
    lock = Lock(LockParams(ki=-0.5, max_step=0.25))
    _, errs = run_plant(lock, 0.4, 3.0, 150)
    assert lock.reversed and lock.p.ki == 0.5 and abs(errs[-1]) < 0.01
    assert "its frequency rises as the offset rises, so its gains are reversed (P 0, I 0.5)" in lock.note


def test_running_away_with_its_gains_either_way_unlocks_it_where_it_was_nearest():
    """The laser does not follow the offset (the wrong laser's piezo, say), and drifts away fast: reversed once, it runs
    away all the same; unlocked, put back where the error was smallest, its gains as they were."""
    lock = Lock(LockParams(ki=1.0, max_step=0.25))
    lock.engage(70.0)
    e, out = 4.0, []
    while lock.active and len(out) < 100:
        e *= 1.25  # (whatever the offset does)
        out.append(lock.update(e))
    assert not lock.active and lock.message.startswith("unlocked: it ran away with its gains either way")
    assert lock.p.ki == 1.0 and not lock.reversed
    assert out[-1] == pytest.approx(lock.u) and 69.5 < out[-1] <= 70.0


def test_a_hop_or_noise_is_not_taken_for_running_away():
    lock = Lock(LockParams(ki=0.5, max_step=0.25, capture=0))
    _, errs = run_plant(lock, 0.35, 0.0, 30)
    for k in range(20):  # it hops 250 MHz: one jump, then the lock brings it back
        e = (250.0 if k < 3 else 250.0 * 0.5 ** (k - 2)) + (-1) ** k * 4.0
        lock.update(e)
    assert not lock.reversed and lock.p.ki == 0.5 and lock.active
    rng = __import__("random").Random(3)
    lock = Lock(LockParams(ki=0.5))
    lock.engage(70.0)
    for _ in range(300):  # noisy readings, the lock holding it
        lock.update(rng.gauss(0.0, 8.0))
    assert not lock.reversed and lock.active


def test_wrong_sign_past_its_first_readings_unlocks_at_excursion_limit():
    """Watched for running away only for its first readings after it is engaged: later on, a laser that comes to tune
    the other way is unlocked at Max excursion, as ever."""
    lock = Lock(LockParams(ki=1.0, max_step=0.25, excursion=2.0, capture=0))
    lock.engage(70.0)
    for _ in range(Lock.WATCH_N):
        lock.update(0.0)
    u, k = 70.0, -0.3
    for _ in range(200):
        new = lock.update(100.0 + k * 1e3 * (u - 70.0))
        if new is None:
            break
        u = new
    assert not lock.active and "would leave" in lock.message and not lock.reversed
    assert 68.0 <= u <= 72.0


def test_step_is_slew_limited_and_integrator_does_not_wind_up():
    lock = Lock(LockParams(ki=5.0, max_step=0.1, capture=0))
    lock.engage(50.0)
    steps = [lock.update(2000.0) for _ in range(5)]
    assert steps == pytest.approx([49.9, 49.8, 49.7, 49.6, 49.5])
    # error gone: with no windup the output stays put instead of overshooting
    assert lock.update(0.0) == pytest.approx(49.5)


def test_capture_range_holds_outliers_then_unlocks():
    lock = Lock(LockParams(capture=100.0))
    lock.engage(10.0)
    assert lock.update(500.0) is None and lock.active
    assert lock.update(10.0) is not None  # a good reading resets the count
    for _ in range(2):
        assert lock.update(-500.0) is None and lock.active
    assert lock.update(-500.0) is None
    assert not lock.active and "capture" in lock.message


def test_proportional_term_and_nan():
    lock = Lock(LockParams(kp=2.0, ki=0.0, max_step=0, v_min=-10))
    lock.engage(0.0)
    assert lock.update(500.0) == pytest.approx(-1.0)  # -kp * 0.5 GHz
    assert lock.update(float("nan")) is None
    lock.disengage("x")
    assert lock.update(1.0) is None


def test_absolute_limits():
    lock = Lock(LockParams(ki=1.0, max_step=1.0, v_min=0.0, v_max=140.0, excursion=0, capture=0))
    lock.engage(139.5)
    assert lock.update(-500.0) == pytest.approx(140.0)
    assert lock.update(-500.0) is None and not lock.active


def test_params_roundtrip_ignores_unknown_keys():
    p = LockParams(kp=1.5, capture=10)
    assert LockParams.from_dict({**p.to_dict(), "bogus": 1}) == p
    assert LockParams.from_dict(None) == LockParams()
    assert math.isclose(LockParams.from_dict({"ki": "0.25"}).ki, 0.25)
