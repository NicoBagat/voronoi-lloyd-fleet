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
- **Build system:** planned: `pyproject.toml` with an editable install (`pip install -e ".[dev]"`)
- **Key libraries:** planned: numpy, matplotlib, pytest; scipy or shapely only where a module needs them

## Getting Started

Not available yet: no code has been written. Setup and test commands arrive with milestone M0.

## Project Structure

Planned:

```
docs/          # proposal and roadmap
src/voronoi_lloyd_fleet/
  core/        # geometry, Lloyd law, estimators, consensus (pure functions)
  sim/         # world, communication model, Monte Carlo runner
configs/       # one configuration per experiment
experiments/   # one script per experiment, writing results and figures
tests/
```

## Status

`planning`: proposal and roadmap written (2026-10-18); next milestone M0 (skeleton and ideal Lloyd).
