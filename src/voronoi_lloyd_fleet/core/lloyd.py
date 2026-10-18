"""Lloyd's coverage law for single-integrator agents, p_dot_i = -k_p (p_i - C_i)."""

import numpy as np


def lloyd_velocity(positions: np.ndarray, centroids: np.ndarray, kp: float) -> np.ndarray:
    """Velocity command of the proportional Lloyd law."""
    if kp <= 0:
        raise ValueError(f"kp must be positive, got {kp}")
    return -kp * (np.asarray(positions, dtype=float) - centroids)


def lloyd_step(positions: np.ndarray, centroids: np.ndarray, kp: float, dt: float) -> np.ndarray:
    """Advance the Lloyd law by dt with the centroids held constant over the step.

    This is the exact zero-order-hold discretization of p_dot = -kp (p - C):
    p(t + dt) = C + exp(-kp dt) (p(t) - C). Every agent moves a fraction
    1 - exp(-kp dt) of the way to its centroid, which can't increase the
    coverage cost for any dt > 0.
    """
    if kp <= 0:
        raise ValueError(f"kp must be positive, got {kp}")
    if dt <= 0:
        raise ValueError(f"dt must be positive, got {dt}")
    positions = np.asarray(positions, dtype=float)
    return centroids + np.exp(-kp * dt) * (positions - centroids)
