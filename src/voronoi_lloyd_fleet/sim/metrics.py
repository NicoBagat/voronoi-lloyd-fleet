"""Performance indices computed from a simulated trajectory."""

import numpy as np


def convergence_time(times: np.ndarray, residual: np.ndarray, tol: float) -> float:
    """First time the residual (largest robot-to-centroid distance) is within tol.

    Returns NaN if the run never gets there.
    """
    hits = np.flatnonzero(np.asarray(residual) <= tol)
    return float(times[hits[0]]) if hits.size else float("nan")


def cost_increases(cost: np.ndarray, rel_tol: float = 1e-12) -> int:
    """Number of steps where the coverage cost went up by more than rel_tol * cost[0]."""
    cost = np.asarray(cost, dtype=float)
    return int(np.count_nonzero(np.diff(cost) > rel_tol * cost[0]))
