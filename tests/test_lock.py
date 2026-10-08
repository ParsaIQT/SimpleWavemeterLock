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


def test_wrong_sign_unlocks_at_excursion_limit():
    lock = Lock(LockParams(ki=1.0, max_step=0.25, excursion=2.0, capture=0))
    u, _ = run_plant(lock, -0.3, 100.0, 200)
    assert not lock.active
    assert "would leave" in lock.message
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
