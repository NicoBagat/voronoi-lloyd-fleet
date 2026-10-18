from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from voronoi_lloyd_fleet.sim.metrics import convergence_time, cost_increases
from voronoi_lloyd_fleet.sim.runner import load_config, run_ideal_lloyd
from voronoi_lloyd_fleet.sim.world import RectangleWorld, initial_positions

CONFIG = Path(__file__).resolve().parents[1] / "configs" / "e0_ideal_lloyd.toml"
WORLD = RectangleWorld(0.0, 6.0, 0.0, 4.0, h=0.1)


def test_shipped_config_loads():
    cfg = load_config(CONFIG)
    assert cfg.n_robots == 8
    assert cfg.world.area == pytest.approx(200.0)
    assert cfg.init_box == (0.0, 4.0, 0.0, 4.0)
    cfg.world.grid()  # h divides the world


def test_same_seed_same_run_different_seed_different_run():
    cfg = replace(load_config(CONFIG), world=WORLD, n_robots=4, init_box=None, t_final=2.0)
    a = initial_positions(cfg.world, 4, np.random.default_rng(11))
    b = initial_positions(cfg.world, 4, np.random.default_rng(11))
    c = initial_positions(cfg.world, 4, np.random.default_rng(12))
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)
    assert np.array_equal(run_ideal_lloyd(cfg, a).positions, run_ideal_lloyd(cfg, b).positions)


def test_initial_positions_stay_in_the_box():
    p = initial_positions(WORLD, 200, np.random.default_rng(0), box=(1.0, 2.0, 0.5, 1.5))
    assert np.all((p[:, 0] >= 1.0) & (p[:, 0] <= 2.0) & (p[:, 1] >= 0.5) & (p[:, 1] <= 1.5))
    with pytest.raises(ValueError):
        initial_positions(WORLD, 3, np.random.default_rng(0), box=(5.0, 7.0, 0.0, 1.0))


def test_metrics():
    times = np.array([0.0, 0.1, 0.2, 0.3])
    assert convergence_time(times, np.array([1.0, 0.5, 0.04, 0.03]), 0.05) == pytest.approx(0.2)
    assert np.isnan(convergence_time(times, np.array([1.0, 0.9, 0.8, 0.7]), 0.05))
    assert cost_increases(np.array([10.0, 9.0, 9.0, 9.5, 8.0])) == 1
