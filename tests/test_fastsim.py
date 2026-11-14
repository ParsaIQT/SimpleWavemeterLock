"""Many model lasers at once (fastsim.Batch: lasermodel.LaserModel's rule for a whole batch of lasers in lockstep,
what the learned recovery is trained on) against the simulator's own, a point at a time (sim.ModelLaser)."""
import math
import random

import pytest

np = pytest.importorskip("numpy")

from wmlock import fastsim, trials  # noqa: E402
from wmlock.sim import ModelLaser  # noqa: E402


@pytest.mark.parametrize("sided", [False, True])
def test_many_lasers_at_once_follow_the_models_rule_point_for_point(sided):
    """fastsim.Batch against sim.ModelLaser along random paths (the current and the piezo moved, time passing, the
    chip's modes drifting): the same external order at every point, the same frequency, the same margins."""
    points = 0
    for seed in range(12):
        rng = random.Random(seed)
        base = [trials.lab493, trials.lab650][seed % 2]() if seed % 3 == 0 else trials.random_model(seed)
        true = trials.unlike(base, np.random.default_rng(seed), 1.5)
        clock = [0.0]
        las = ModelLaser(true, clock=lambda: clock[0], sided=sided)
        v, i = true.v0 + rng.uniform(-3, 3), true.i0 + rng.uniform(-1, 1)
        las.lase(v, i)
        b = fastsim.Batch.of([true], sided=sided)
        b.start(i, v)
        assert b.order[0] == las._state[0]
        for _ in range(250):
            r = rng.random()
            if r < 0.5:
                i = min(max(i + rng.uniform(-0.2, 0.2), true.i0 - 3), true.i0 + 3)
            elif r < 0.6:
                v += rng.uniform(-0.5, 0.5)
            elif r < 0.65:
                true.offset += rng.uniform(-0.02, 0.02) * true.dc
                b.offset[0] = true.offset
            dt = rng.choice([0.0, 0.002, 0.02, 0.15, 1.0])
            clock[0] += dt
            las.lase(v, i)
            b.step(i, v, dt)
            assert b.order[0] == las._state[0], (seed, points)
            assert b.ghz[0] == pytest.approx(las._ghz, abs=1e-9)
            points += 1
        down, up = b.margins(i, v)
        for mine, its in zip((down[0], up[0]), las.holds(v, i)):
            assert (math.isinf(mine) and math.isinf(its)) or mine == pytest.approx(its, abs=1e-9)
    assert points == 3000


def test_the_piezo_that_keeps_its_mode_on_target_as_its_current_moves():
    b = fastsim.Batch.of([trials.lab493(), trials.lab650(), trials.random_model(3)])
    i_ref, v_ref = b.i0 + np.array([0.4, -0.7, 0.1]), b.v0 + np.array([2.0, -1.0, 0.5])
    b.start(i_ref, v_ref)
    for d in (-2.0, -0.3, 1.1, 2.5):
        v = b.piezo_for(i_ref + d, i_ref, v_ref)
        assert b.mode_ghz(b.order, i_ref + d, v) == pytest.approx(b.mode_ghz(b.order, i_ref, v_ref), abs=1e-9)
        assert np.all(np.abs(v - v_ref) < 6.0)
