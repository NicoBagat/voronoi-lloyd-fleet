"""Simulation of the ideal Lloyd baseline (M0) and its configuration.

M0: holonomic agents, exact and instantly shared positions, uniform density,
convex space. Later milestones add one non-ideality each.
"""

import tomllib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from voronoi_lloyd_fleet.core.geometry import voronoi_cells
from voronoi_lloyd_fleet.core.lloyd import lloyd_step
from voronoi_lloyd_fleet.sim.world import RectangleWorld


@dataclass(frozen=True)
class IdealLloydConfig:
    world: RectangleWorld
    n_robots: int
    init_box: tuple[float, float, float, float] | None
    kp: float
    dt: float
    t_final: float
    n_runs: int
    base_seed: int
    residual_tol: float


@dataclass(frozen=True)
class Trajectory:
    """times (K+1,), positions (K+1, n, 2), cost (K+1,), residual (K+1,).

    residual[k] is the largest distance from a robot to its cell centroid at
    times[k]; it is zero exactly at a centroidal Voronoi configuration.
    """

    times: np.ndarray
    positions: np.ndarray
    cost: np.ndarray
    residual: np.ndarray


def load_config(path: str | Path) -> IdealLloydConfig:
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    w, fleet, ctrl, mc = raw["world"], raw["fleet"], raw["control"], raw["monte_carlo"]
    box = fleet.get("init_box")
    return IdealLloydConfig(
        world=RectangleWorld(w["xmin"], w["xmax"], w["ymin"], w["ymax"], w["h"]),
        n_robots=int(fleet["n_robots"]),
        init_box=tuple(box) if box is not None else None,
        kp=float(ctrl["kp"]),
        dt=float(ctrl["dt"]),
        t_final=float(ctrl["t_final"]),
        n_runs=int(mc["n_runs"]),
        base_seed=int(mc["base_seed"]),
        residual_tol=float(mc["residual_tol"]),
    )


def run_ideal_lloyd(config: IdealLloydConfig, positions0: np.ndarray) -> Trajectory:
    """Integrate the Lloyd law from positions0 with exact, shared positions.

    The run is deterministic: all randomness is in positions0.
    """
    grid = config.world.grid()
    n_steps = round(config.t_final / config.dt)
    positions = np.empty((n_steps + 1, *np.shape(positions0)))
    cost = np.empty(n_steps + 1)
    residual = np.empty(n_steps + 1)
    p = np.asarray(positions0, dtype=float)
    for k in range(n_steps + 1):
        cells = voronoi_cells(grid.points, grid.weights, p)
        positions[k] = p
        cost[k] = cells.cost
        residual[k] = np.linalg.norm(p - cells.centroids, axis=1).max()
        if k < n_steps:
            p = lloyd_step(p, cells.centroids, config.kp, config.dt)
    times = config.dt * np.arange(n_steps + 1)
    return Trajectory(times=times, positions=positions, cost=cost, residual=residual)
