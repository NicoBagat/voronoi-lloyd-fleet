"""Grid-based Voronoi geometry: cells, masses, centroids and the coverage cost.

The mission space is sampled at the centres of a regular grid of squares of
side h. Each sample carries a weight, the area of its square times the density
there, so every integral over a cell becomes a sum over the samples assigned
to it:

    M_i = sum of w_q over q in V_i
    C_i = (1 / M_i) * sum of w_q * q over q in V_i
    J   = sum over q of w_q * min_i ||q - p_i||^2

Later restrictions of the cell (sensing range, visibility, collision margins)
and non-uniform densities only mask or reweight samples, so they reuse these
functions unchanged.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Grid:
    """Sample points of a rectangular mission space.

    points: (N, 2) sample positions, row-major over `shape`.
    weights: (N,) area represented by each sample (h**2 for a uniform density).
    shape: (ny, nx), so `values.reshape(shape)` gives an image of the space.
    """

    points: np.ndarray
    weights: np.ndarray
    shape: tuple[int, int]


@dataclass(frozen=True)
class Cells:
    """Voronoi cells of a configuration, evaluated on a grid.

    labels: (N,) index of the agent each sample belongs to.
    masses: (n,) cell masses M_i.
    centroids: (n, 2) cell centroids C_i. An empty cell (possible only when
        two agents coincide) gets its agent's own position, so it doesn't move.
    cost: coverage cost J(p, V(p)), each sample served by its nearest agent.
    """

    labels: np.ndarray
    masses: np.ndarray
    centroids: np.ndarray
    cost: float


def rectangle_grid(xmin: float, xmax: float, ymin: float, ymax: float, h: float) -> Grid:
    """Sample the rectangle [xmin, xmax] x [ymin, ymax] at the centres of h-squares.

    The side lengths must be integer multiples of h, so that the samples tile
    the rectangle exactly and the total weight equals its area.
    """
    if h <= 0:
        raise ValueError(f"grid step must be positive, got {h}")
    width, height = xmax - xmin, ymax - ymin
    if width <= 0 or height <= 0:
        raise ValueError("rectangle must have positive width and height")
    nx, ny = round(width / h), round(height / h)
    if not (np.isclose(nx * h, width) and np.isclose(ny * h, height)):
        raise ValueError(f"grid step {h} does not divide the rectangle {width} x {height}")
    xs = xmin + (np.arange(nx) + 0.5) * h
    ys = ymin + (np.arange(ny) + 0.5) * h
    gx, gy = np.meshgrid(xs, ys, indexing="xy")
    points = np.column_stack([gx.ravel(), gy.ravel()])
    weights = np.full(len(points), h * h)
    return Grid(points=points, weights=weights, shape=(ny, nx))


def squared_distances(points: np.ndarray, positions: np.ndarray) -> np.ndarray:
    """(N, n) matrix of squared distances between samples and agents."""
    dx = points[:, 0, None] - positions[None, :, 0]
    dy = points[:, 1, None] - positions[None, :, 1]
    return dx * dx + dy * dy


def voronoi_cells(points: np.ndarray, weights: np.ndarray, positions: np.ndarray) -> Cells:
    """Assign each sample to its nearest agent and integrate mass, centroid and cost.

    Ties (samples equidistant from two agents) go to the lower agent index.
    """
    positions = np.asarray(positions, dtype=float)
    n = len(positions)
    d2 = squared_distances(points, positions)
    labels = np.argmin(d2, axis=1)
    cost = float(np.dot(weights, d2[np.arange(len(points)), labels]))
    masses = np.bincount(labels, weights=weights, minlength=n)
    first_x = np.bincount(labels, weights=weights * points[:, 0], minlength=n)
    first_y = np.bincount(labels, weights=weights * points[:, 1], minlength=n)
    centroids = positions.copy()
    nonempty = masses > 0
    centroids[nonempty, 0] = first_x[nonempty] / masses[nonempty]
    centroids[nonempty, 1] = first_y[nonempty] / masses[nonempty]
    return Cells(labels=labels, masses=masses, centroids=centroids, cost=cost)
