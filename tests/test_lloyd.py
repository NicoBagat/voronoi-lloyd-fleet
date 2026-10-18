from dataclasses import replace

import numpy as np
import pytest

from voronoi_lloyd_fleet.core.geometry import rectangle_grid, voronoi_cells
from voronoi_lloyd_fleet.core.lloyd import lloyd_step, lloyd_velocity
from voronoi_lloyd_fleet.sim.metrics import cost_increases
from voronoi_lloyd_fleet.sim.runner import IdealLloydConfig, run_ideal_lloyd
from voronoi_lloyd_fleet.sim.world import RectangleWorld, initial_positions

SMALL = IdealLloydConfig(
    world=RectangleWorld(0.0, 6.0, 0.0, 4.0, h=0.1),
    n_robots=5,
    init_box=(0.0, 2.0, 0.0, 2.0),
    kp=1.0,
    dt=0.1,
    t_final=30.0,
    n_runs=1,
    base_seed=0,
    residual_tol=0.05,
)


def test_step_moves_each_agent_toward_its_centroid_by_the_exact_factor():
    p = np.array([[0.0, 0.0], [2.0, 1.0]])
    c = np.array([[1.0, 1.0], [2.0, 3.0]])
    nxt = lloyd_step(p, c, kp=2.0, dt=0.1)
    assert np.linalg.norm(nxt - c, axis=1) == pytest.approx(np.exp(-0.2) * np.linalg.norm(p - c, axis=1))
    # For a small step, the displacement matches the continuous-time velocity.
    dt = 1e-6
    assert (lloyd_step(p, c, 2.0, dt) - p) / dt == pytest.approx(lloyd_velocity(p, c, 2.0), rel=1e-5)


@pytest.mark.parametrize("kp, dt", [(0.0, 0.1), (-1.0, 0.1), (1.0, 0.0)])
def test_step_rejects_non_positive_gain_or_step(kp, dt):
    with pytest.raises(ValueError):
        lloyd_step(np.zeros((1, 2)), np.ones((1, 2)), kp, dt)


def test_centroidal_configuration_is_a_fixed_point():
    g = rectangle_grid(0.0, 4.0, 0.0, 2.0, 0.1)
    p = np.array([[1.0, 0.5], [3.0, 0.5], [1.0, 1.5], [3.0, 1.5]])  # centres of the 4 quadrants
    cells = voronoi_cells(g.points, g.weights, p)
    assert cells.centroids == pytest.approx(p)
    assert lloyd_step(p, cells.centroids, 1.0, 0.1) == pytest.approx(p)


@pytest.mark.parametrize("seed", range(5))
def test_cost_is_a_lyapunov_function_and_the_fleet_converges(seed):
    p0 = initial_positions(SMALL.world, SMALL.n_robots, np.random.default_rng(seed), SMALL.init_box)
    traj = run_ideal_lloyd(SMALL, p0)
    assert cost_increases(traj.cost) == 0
    assert traj.cost[-1] < 0.5 * traj.cost[0]
    assert traj.residual[-1] < SMALL.residual_tol


def test_monotonicity_holds_even_for_a_large_step():
    # The exact discretization moves agents only part of the way to their
    # centroids, so the cost can't increase whatever dt is.
    big = replace(SMALL, dt=2.0, t_final=40.0)
    p0 = initial_positions(big.world, big.n_robots, np.random.default_rng(7), big.init_box)
    assert cost_increases(run_ideal_lloyd(big, p0).cost) == 0
