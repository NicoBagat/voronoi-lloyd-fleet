# voronoi-lloyd-fleet

> Voronoi–Lloyd coverage control for a fleet of logistics robots with estimated positions. Individual exam project for *Intelligent Distributed Systems* (University of Trento).

## Overview

A fleet of mobile robots spreads over a warehouse-like area using Lloyd's algorithm: each robot moves toward the centroid of its Voronoi cell. Here the robots don't know their positions exactly:

- **Base:** each robot estimates its own pose with an extended Kalman filter, from wheel odometry and range measurements to fixed anchors. Voronoi cells are computed from the *estimated* positions that neighbours exchange over a lossy, delayed, range-limited wireless link. Coverage is scored at the true positions.
- **Stretch goal:** the fleet also estimates a demand density over the area (for example, where pick-ups occur), by distributed consensus on local observations. That density weights the coverage.

The evaluation is a Monte Carlo study of coverage quality and convergence under estimation error, packet loss, communication delay and limited range.

- [docs/proposal.md](docs/proposal.md): problem formulation, approach, models, evaluation plan, references.
- [docs/roadmap.md](docs/roadmap.md): milestones M0–M6, experiments E1–E7, scope tiers, working rules.

## Hardware

None: the results come from simulation. A small validation on AgileX LIMO robots may be added later.

## Tech Stack

- **Language:** Python 3.12
- **Framework / SDK:** none. A stand-alone simulator. The algorithm core has no I/O, so it can later be wrapped in ROS2 nodes.
- **Build system:** `pyproject.toml` with an editable install
- **Key libraries:** numpy, matplotlib, pytest; scipy or shapely only where a later module needs them

## Getting Started

```bash
python3 -m venv .venv
.venv/bin/pip install -e ".[dev]"
.venv/bin/python -m pytest
.venv/bin/python experiments/e0_ideal_lloyd.py
```

The last command runs the ideal Lloyd baseline (20 seeded runs, a few minutes) and writes `results/e0_ideal_lloyd.csv` and `figures/e0_ideal_lloyd.png`. Every run is reproducible from the configuration in `configs/` and its seed.

## Project Structure

```
docs/          # proposal and roadmap
src/voronoi_lloyd_fleet/
  core/        # pure functions, no I/O: grid Voronoi geometry, exact polygon cells (test oracle), Lloyd law
  sim/         # world, runner, metrics
configs/       # one configuration per experiment
experiments/   # one script per experiment, writing results/ and figures/
tests/
```

Planned additions, milestone by milestone, are listed in the [roadmap](docs/roadmap.md): estimators and consensus in `core/`, the communication model in `sim/`.

## Status

`development`: milestone M0 (skeleton, ideal Lloyd baseline, tests, CI) implemented on its branch; next is M1 (geometry with range limits, obstacles, collision margins and densities).
