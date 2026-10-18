# Project Proposal: Voronoi–Lloyd Coverage for a Logistics-Robot Fleet with Estimated Positions

*Exam project for* Intelligent Distributed Systems *(University of Trento). Individual project. Status: proposal, October 2026.*

## 1. Motivation

A fleet of mobile robots in a warehouse should spread out so that every point of the floor, or every place where pick-ups happen, is close to some robot. Voronoi-based coverage control with Lloyd's algorithm does this with purely local rules: each robot moves toward the centroid of its own Voronoi cell, and it only needs its neighbours' positions to compute that cell [1].

The textbook law assumes two things that real fleets don't have:

- **exact positions:** real robots *estimate* their pose, wheel odometry drifts, and ranging is noisy;
- **instant sharing:** neighbours' positions arrive over a wireless network that delays messages, loses them and has a limited range.

The project asks **how much each of these non-idealities degrades coverage, and where the controller stops working.**

## 2. Problem formulation

- **Mission space and robots:** a polygonal mission space $Q \subset \mathbb{R}^2$ with rack obstacles $O$, and $n$ robots at positions $p_i$.
- **Voronoi cell of robot $i$:**

  $$V_i = \{\, q \in Q : \lVert q - p_i \rVert \le \lVert q - p_j \rVert \;\; \forall j \ne i \,\}$$

- **Mass and centroid** under a density $\varphi(q)$ (uniform in the base problem):

  $$M_{V_i} = \int_{V_i} \varphi(q)\, dq, \qquad C_{V_i} = \frac{1}{M_{V_i}} \int_{V_i} \varphi(q)\, q \, dq$$

- **Coverage cost:**

  $$J_{\mathrm{cov}}(p, V) = \sum_{i=1}^{n} \int_{V_i} \varphi(q)\, \lVert q - p_i \rVert^2 \, dq$$

- **Lloyd control law:** $\dot p_i = -k_p\,(p_i - C_{V_i})$ with $k_p > 0$. It drives the fleet to a centroidal Voronoi configuration, a local minimum of $J_{\mathrm{cov}}$, which serves as the Lyapunov function [1].

**What changes in this project.** Robot $i$ knows only:
- its own estimate $\hat p_i$;
- the possibly stale estimates $\hat p_j$ of the neighbours whose messages reached it.

It computes its cell from these. Coverage, however, is always *scored* at the true positions. The gap between the two is what the project measures.

## 3. Approach

### 3.1 Base: coverage from estimated poses

1. **Localization.** Each robot runs an extended Kalman filter on its pose $(x, y, \theta)$. The prediction comes from wheel-encoder odometry. The updates come from range measurements to a few fixed anchors (UWB-like beacons) [5].
2. **Cells.** Each robot builds its own cell from its estimate and the latest neighbour messages:
   - **range-limited** to the sensing disk of radius $R_s$. Neighbours within $R_c > 2R_s$ are sufficient for this cell;
   - **visibility-limited**, with obstacles inflated by the robot footprint;
   - **collision-aware**, with the bisector shifted when two robots get too close [2, 3].
3. **Motion.** The robots are unicycles (differential drive). Lloyd is applied to a look-ahead point in front of the wheel axle, which behaves as a single integrator. The commands $v$ and $\omega$ are saturated.
4. **Communication.** Each robot broadcasts its estimate, covariance and timestamp periodically. Nothing is centralized.

### 3.2 Stretch goal: an estimated demand density

The density $\varphi(q)$ (where pick-ups happen) is unknown. It is modelled as a weighted sum of radial basis functions, $\varphi(q) = \sum_k \theta_k K_k(q)$. Each robot observes events near itself, so its local measurements are linear in $\theta$. The fleet estimates $\theta$ by distributed weighted least squares, or by a distributed Kalman filter in information form if $\varphi$ drifts. Both run as average consensus on information matrices and vectors, with Metropolis–Hastings weights on the time-varying communication graph [6, 7]. Lloyd then uses $\hat\varphi$.

### 3.3 Optional: cooperative localization

Inter-robot range measurements can improve localization. Fused naively, they make the estimates **overconfident**, because the robots' errors become correlated. The optional work compares naive fusion with a consistent scheme [4, 8] by normalized estimation error squared (NEES) [5].

## 4. Models

**Communication system.**
- **Medium:** Wi-Fi (IEEE 802.11), with contention-based access (CSMA/CA). It gives random access delays, packet losses and a limited range.
- **In simulation, each link is modelled as:**
  - delivery only within $R_c$;
  - Bernoulli delivery with probability $p$;
  - a delay of transmission time + random back-off + processing time;
  - a broadcast period $T_c$.
- **Receivers** keep the latest timestamped message per neighbour and drop entries older than a staleness limit.

**Robot and sensors.**
- **Kinematics:** unicycle, $\dot x = v\cos\theta$, $\dot y = v\sin\theta$, $\dot\theta = \omega$, with velocity limits.
- **Sensors:** wheel encoders with additive noise on the increments; anchor ranges with Gaussian noise. Anchor updates may arrive intermittently.
- **Discretization:** the control period is $T_s$, and the process noise is discretized consistently with $T_s$.

## 5. Evaluation plan

**Metrics:**
- **coverage:** $J_{\mathrm{cov}}$ evaluated at the *true* positions (and the true $\varphi$), normalized by the value ideal Lloyd reaches from the same initial configuration;
- **convergence time:** until the normalized cost (or the largest distance from a robot to its centroid) is within tolerance;
- **steady state:** the cost and its jitter over the final window;
- **estimation:** position-error CDFs and NEES.

**Experiments:**

| # | Varied | Tier |
|---|---|---|
| E1 | fleet size $n$ | core |
| E2 | broadcast/update period $T_c$ (a quality-of-service vs quality-of-control trade-off is expected) | core |
| E3 | delivery probability $p \in [0.3, 1]$ | core |
| E4 | ratio $R_c / R_s$ around 2 | core |
| E5 | perfect vs EKF-estimated positions, at two noise levels | core |
| E6 | true vs consensus-estimated vs uniform density | stretch |
| E7 | naive vs consistent cooperative localization; EKF vs UKF | optional |

**Protocol:**
- at least 20 Monte Carlo runs per setting;
- the same initial configurations reused across settings (paired comparisons with sign and Wilcoxon tests);
- results reported as medians, interquartile ranges and CDFs;
- every run seeded and reproducible.

## 6. Implementation

- **Language:** Python 3.12 with numpy and matplotlib, plus pytest for tests. scipy or shapely only where a module needs them.
- **Why not MATLAB:**
  - the algorithm core can later run inside ROS2 nodes unchanged;
  - runs are reproducible from a seed;
  - no licence is needed.
- **Structure:**
  - a pure algorithm core: geometry, Lloyd law, estimators, consensus. It does no I/O and holds no global state;
  - a simulator layer around it: world, network model, Monte Carlo runner;
  - one script per experiment.
- **Cells** are computed on a grid over the mission space, which handles range limits, visibility, collision margins and any density in one mechanism. Exact polygon clipping is the test oracle for the convex, uniform case.

## 7. Deliverables

- **A report** (double column, 6–8 pages):
  - introduction and problem formulation;
  - adopted models (communication system, robot and sensors);
  - solution;
  - implementation;
  - results (at least two pages);
  - conclusions.
- **This repository:** every figure and table can be reproduced with one command.
- **A presentation** of the results for the exam discussion.

The milestone plan is in [roadmap.md](roadmap.md).

## 8. Scope, risks and limits

- **Scope:** the core tier (base approach, E1–E5) is complete on its own. The density estimation (E6) starts only once E1–E5 have their figures and tables.
- **Risks:**
  - the grid resolution trades accuracy for run time, and is fixed early from a convergence test;
  - Lloyd converges only to local optima, which depend on the initial configuration (mitigated by the paired design);
  - the look-ahead mapping does not control the robot's heading.
- **Limit:** the results are simulation-only.

## 9. Future directions

- Wrap one robot's step function as a ROS2 node.
- Replace the simulated network with DDS plus injected loss, delay and range limits.
- Replace the point robots with Gazebo differential-drive models.
- Then move toward robots that share their workspace with people [2].

## References

- D. Fontanelli, *Intelligent Distributed Systems*, course notes, University of Trento, 2023/24 (beta). Mainly:
  - Ch.5 (QoS and delays) and Ch.7 (wireless);
  - Ch.13 (Kalman and extended Kalman filters) and Ch.15 (distributed estimation with relative measurements);
  - Ch.16 (discretization and unicycle odometry);
  - Ch.17 (linear consensus) and Ch.18 (distributed WLS and Kalman filtering);
  - Ch.19 (distributed control: Voronoi coverage and communication effects).
- [1] J. Cortés, S. Martínez, T. Karatas, F. Bullo, "Coverage control for mobile sensing networks," *IEEE Transactions on Robotics and Automation*, 20(2):243–255, 2004.
- [2] M. Boldrer, L. Palopoli, D. Fontanelli, "Lloyd-based approach for robots navigation in human-shared environments," *Proc. IEEE/RSJ IROS*, pp. 6982–6989, 2020.
- [3] M. Boldrer, L. Palopoli, D. Fontanelli, "Graph connectivity control of a mobile robot network with mixed dynamic multi-tasks," *IEEE Robotics and Automation Letters*, 6(2):1934–1941, 2021.
- [4] S. S. Kia, S. Rounds, S. Martínez, "Cooperative localization for mobile agents: a recursive decentralized algorithm based on Kalman-filter decoupling," *IEEE Control Systems Magazine*, 36(2):86–101, 2016.
- [5] Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001.
- [6] L. Xiao, S. Boyd, "Fast linear iterations for distributed averaging," *Systems & Control Letters*, 53(1):65–78, 2004.
- [7] L. Xiao, S. Boyd, S. Lall, "A scheme for robust distributed sensor fusion based on average consensus," *Proc. IPSN*, pp. 63–70, 2005.
- [8] D. Fontanelli, D. Macii, P. Nazemzadeh, L. Palopoli, "Collaborative localization of robotic wheeled walkers using interlaced extended Kalman filters," *Proc. IEEE I2MTC*, 2016.
