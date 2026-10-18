# Roadmap

The planned milestones for the project described in [proposal.md](proposal.md). No exam date is fixed yet, so the plan is ordered by dependency, not by calendar.

**Status legend:** ⬜ not started · 🔄 in progress · ✅ done

## Scope tiers

| Tier | Milestones | Experiments | Report |
|---|---|---|---|
| **Core**, required | Phase 0, M0–M4 | E1–E5 | complete on its own |
| **Stretch** | M5 | E6 | adds the density-estimation subsection and its results |
| **Optional** | M6 | E7 | only if time remains |

**Decision point:** M5 starts only once E1–E5 have their figures and tables. Otherwise the density estimation goes into the report's future work.

## Phase 0: formulation and interfaces ⬜

- **Fix the scenario:**
  - warehouse polygon and rack obstacles;
  - number of robots;
  - $R_s$, $R_c$, $T_s$, $T_c$;
  - noise levels;
  - anchor layout.
- **Write a notation sheet** used by both the code and the report. Check every formula taken from the course notes against the original pages.
- **Define the inter-robot message:** robot id, pose estimate, covariance, timestamp. For the stretch tier, add the local information pair of the density parameters.
- **Define the metrics:** the tolerance for convergence time, and the steady-state window.

**Exit:** the scenario and message definitions are written down in `docs/`.

## Milestones

Each milestone adds **one** non-ideality and leaves behind its own unit tests and one figure. A regression can then be traced to a single change.

| ID | Adds | Exit criteria | Tier | Status |
|---|---|---|---|---|
| **M0** | Project skeleton (`pyproject.toml`, `src/` layout, tests, CI). Ideal Lloyd: holonomic robots, exact positions, uniform density, convex space. Grid-based cells and centroids; exact polygon cells as the oracle | the cost is non-increasing along every run (Lyapunov check); grid centroids converge to the polygon centroids as the grid is refined; runs are identical for identical seeds | core | 🔄 (branch `M0-init`) |
| **M1** | Geometry: sensing-range-limited cells; visibility cells with inflated obstacles; collision-aware cells; density-weighted mass and centroid | cells match hand-built cases; robots never enter obstacles; the minimum pairwise distance respects the safety margin | core | ⬜ |
| **M2** | Robot model: unicycle kinematics, look-ahead point mapping, $v/\omega$ saturation, discretization at $T_s$ | look-ahead point tracks a holonomic reference; saturation is respected; the cost still decreases in the ideal setting | core | ⬜ |
| **M3** | Network: decentralized periodic broadcast; delivery limited by range, Bernoulli loss, random delay; staleness handling | message statistics match the configured loss and delay; with $p = 1$, no delay and large $R_c$, the M2 result is reproduced | core | ⬜ |
| **M4** | Localization: per-robot EKF (odometry prediction, anchor-range updates); cells built from estimates; coverage scored at true positions | NEES within the χ² bounds on the static test case; with zero noise, the M3 result is reproduced | core | ⬜ |
| **M5** | Demand density: radial-basis model, local event observations, distributed WLS/KF by consensus (Metropolis–Hastings weights), Lloyd on the estimate | consensus reaches the centralized estimate on a fixed connected graph; the density error decreases with observations | stretch | ⬜ |
| **M6** | Cooperative localization with inter-robot ranges: naive vs consistent fusion; EKF vs UKF | the naive filter shows NEES above the bounds, and the consistent filter stays within them | optional | ⬜ |

**Alongside M0:** a Monte Carlo runner with one configuration file per experiment, an explicit seed per run, CSV output, and figures regenerated from the CSV.

## Experiments

| ID | Varied | Main metrics | Needs | Tier |
|---|---|---|---|---|
| E1 | fleet size $n$ | normalized cost over time, convergence time | M2 | core |
| E2 | broadcast/update period $T_c$ | convergence time | M3 | core |
| E3 | delivery probability $p \in [0.3, 1]$ | convergence time (median and spread) | M3 | core |
| E4 | ratio $R_c / R_s$ around 2 | coverage error vs exact cells | M3 | core |
| E5 | perfect vs estimated positions (two noise levels) | cost at true positions, position-error CDF, NEES | M4 | core |
| E6 | true vs estimated vs uniform density | cost under the true density | M5 | stretch |
| E7 | cooperative localization variants; EKF vs UKF | NEES, RMSE | M6 | optional |

**Protocol:**
- at least 20 runs per setting;
- the same initial configurations across settings (paired comparisons);
- medians, interquartile ranges and CDFs.

## Report

| Section | Written after |
|---|---|
| I. Introduction and problem formulation | Phase 0 |
| II. Adopted models (communication, robot and sensors) | M3 (communication), M4 (sensors) |
| III. Solution | M1–M4 (+ M5 for the stretch tier) |
| IV. Implementation details | M4 |
| V. Results (at least 2 pages) | E1–E5 (+ E6) |
| VI. Conclusions | last |

**Delivery:** the report goes in at least 48 hours before the exam discussion, together with a short presentation of the results.

## Working rules

- **Branches:** one branch per milestone (for example `m0-ideal-lloyd`), merged into `main` through a pull request after the full test suite passes.
- **Reproducibility:** every figure in the report is produced by a script in `experiments/` from a configuration and a seed.
- **Core purity:** the algorithm core performs no I/O and has no global state, so it can later run inside a ROS2 node unchanged.
