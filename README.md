# voronoi-lloyd-fleet

> Voronoi–Lloyd coverage control for a fleet of logistics robots with estimated positions. Exam project for *Intelligent Distributed Systems* (University of Trento).

## Overview

A fleet of mobile robots spreads over a warehouse-like area using Lloyd's algorithm: each robot moves toward the centroid of its Voronoi cell. Here the robots don't know their positions exactly:

- **Base:** each robot estimates its own pose with an extended Kalman filter, from odometry and range measurements. Voronoi cells are computed from the *estimated* positions that neighbours exchange over a lossy, delayed wireless link.
- **Extension:** the fleet also estimates a demand density over the area (for example, where pick-ups occur), by distributed consensus on local observations. That density weights the coverage.

The evaluation is a Monte Carlo study of coverage quality and convergence under estimation error, packet loss, communication delay and limited range.

## Hardware

None: the results come from simulation. A small validation on AgileX LIMO robots may be added later.

## Tech Stack

- **Language:** Python 3.12
- **Framework / SDK:** none. A stand-alone simulator. The algorithm core has no I/O, so it can later be wrapped in ROS2 nodes.
- **Build system:** planned: `pyproject.toml` with an editable install
- **Key libraries:** planned: numpy, scipy, matplotlib, pytest

## Getting Started

Not available yet: no code has been written.

## Project Structure

Planned:

```
src/voronoi_lloyd_fleet/
  core/        # geometry, Lloyd step, estimators, consensus (pure functions)
  sim/         # world, communication model, Monte Carlo runner
experiments/   # one script per experiment, writing figures and tables
tests/
```

## Status

`planning` — started 2026-10-05
