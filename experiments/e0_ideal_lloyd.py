"""E0 / M0: ideal Lloyd baseline over seeded Monte Carlo runs.

Writes results/e0_ideal_lloyd.csv (one row per run) and
figures/e0_ideal_lloyd.png (final cells and paths of the first run; the
normalized cost of every run).

    python experiments/e0_ideal_lloyd.py [--config configs/e0_ideal_lloyd.toml]
"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from voronoi_lloyd_fleet.core.geometry import voronoi_cells  # noqa: E402
from voronoi_lloyd_fleet.sim.metrics import convergence_time, cost_increases  # noqa: E402
from voronoi_lloyd_fleet.sim.runner import load_config, run_ideal_lloyd  # noqa: E402
from voronoi_lloyd_fleet.sim.world import initial_positions  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=ROOT / "configs" / "e0_ideal_lloyd.toml")
    parser.add_argument("--out-dir", type=Path, default=ROOT)
    args = parser.parse_args()

    cfg = load_config(args.config)
    rows, trajectories = [], []
    for k in range(cfg.n_runs):
        seed = cfg.base_seed + k
        p0 = initial_positions(cfg.world, cfg.n_robots, np.random.default_rng(seed), cfg.init_box)
        traj = run_ideal_lloyd(cfg, p0)
        trajectories.append(traj)
        rows.append(
            {
                "seed": seed,
                "n_robots": cfg.n_robots,
                "cost_initial": traj.cost[0],
                "cost_final": traj.cost[-1],
                "residual_final": traj.residual[-1],
                "t_conv": convergence_time(traj.times, traj.residual, cfg.residual_tol),
                "cost_increases": cost_increases(traj.cost),
            }
        )

    results = args.out_dir / "results"
    figures = args.out_dir / "figures"
    results.mkdir(parents=True, exist_ok=True)
    figures.mkdir(parents=True, exist_ok=True)
    with open(results / "e0_ideal_lloyd.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    plot(cfg, trajectories, figures / "e0_ideal_lloyd.png")

    t_conv = np.array([r["t_conv"] for r in rows])
    finals = np.array([r["cost_final"] for r in rows])
    print(f"runs: {cfg.n_runs}, robots: {cfg.n_robots}, grid step: {cfg.world.h} m")
    print(f"converged (residual <= {cfg.residual_tol} m): {np.isfinite(t_conv).sum()}/{cfg.n_runs}")
    if np.isfinite(t_conv).any():
        print(f"t_conv median {np.nanmedian(t_conv):.1f} s, range {np.nanmin(t_conv):.1f}-{np.nanmax(t_conv):.1f} s")
    print(f"final cost median {np.median(finals):.2f} m^4, range {finals.min():.2f}-{finals.max():.2f}")
    print(f"steps with a cost increase, all runs: {sum(r['cost_increases'] for r in rows)}")


def plot(cfg, trajectories, path: Path) -> None:
    grid = cfg.world.grid()
    first = trajectories[0]
    cells = voronoi_cells(grid.points, grid.weights, first.positions[-1])
    w = cfg.world

    fig, (ax_map, ax_cost) = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw={"width_ratios": [1.6, 1]})
    ax_map.imshow(
        cells.labels.reshape(grid.shape),
        origin="lower",
        extent=(w.xmin, w.xmax, w.ymin, w.ymax),
        cmap="tab20",
        alpha=0.35,
        interpolation="nearest",
    )
    for i in range(cfg.n_robots):
        path_i = first.positions[:, i]
        ax_map.plot(path_i[:, 0], path_i[:, 1], color="0.3", lw=0.8)
    ax_map.plot(*first.positions[0].T, "o", mfc="white", mec="0.3", ms=4, label="start")
    ax_map.plot(*first.positions[-1].T, "o", color="black", ms=4, label="final")
    ax_map.plot(*cells.centroids.T, "x", color="tab:red", ms=6, label="centroid")
    ax_map.set_xlim(w.xmin, w.xmax)
    ax_map.set_ylim(w.ymin, w.ymax)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.set_title(f"Final Voronoi cells and paths (seed {cfg.base_seed})")
    ax_map.legend(loc="upper right", fontsize=8)

    for traj in trajectories:
        ax_cost.plot(traj.times, traj.cost / traj.cost[0], color="tab:blue", lw=0.8, alpha=0.6)
    ax_cost.set_yscale("log")
    ax_cost.set_xlabel("t [s]")
    ax_cost.set_ylabel("J(t) / J(0)")
    ax_cost.set_title(f"Coverage cost, {len(trajectories)} runs")
    ax_cost.grid(True, which="both", alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
