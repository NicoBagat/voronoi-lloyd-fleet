import numpy as np
import pytest

from voronoi_lloyd_fleet.core.geometry import rectangle_grid, voronoi_cells
from voronoi_lloyd_fleet.core.polygon import exact_cells

W, H = 4.0, 3.0
DOMAIN = np.array([[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]])


def test_grid_tiles_the_rectangle():
    g = rectangle_grid(0.0, W, 0.0, H, 0.1)
    assert g.shape == (30, 40)
    assert len(g.points) == 30 * 40
    assert g.weights.sum() == pytest.approx(W * H)
    assert g.points.min(axis=0) == pytest.approx([0.05, 0.05])
    assert g.points.max(axis=0) == pytest.approx([W - 0.05, H - 0.05])


@pytest.mark.parametrize("h", [0.0, -0.1, 0.3])  # 0.3 doesn't divide W = 4
def test_grid_rejects_bad_steps(h):
    with pytest.raises(ValueError):
        rectangle_grid(0.0, W, 0.0, H, h)


def test_single_agent_mass_centroid_and_cost():
    h = 0.1
    g = rectangle_grid(0.0, W, 0.0, H, h)
    p = np.array([[0.7, 2.2]])
    cells = voronoi_cells(g.points, g.weights, p)
    assert cells.masses[0] == pytest.approx(W * H)
    assert cells.centroids[0] == pytest.approx([W / 2, H / 2])
    # Exact: integral of ||q - p||^2 over the rectangle. The midpoint rule misses
    # h^2/12 per axis per unit area, so the grid value is low by exactly W H h^2 / 6.
    c = np.array([W / 2, H / 2])
    exact = W * H * (W**2 + H**2) / 12 + W * H * np.sum((p[0] - c) ** 2)
    assert cells.cost + W * H * h**2 / 6 == pytest.approx(exact, rel=1e-12)


def test_two_symmetric_agents_split_the_rectangle_in_halves():
    g = rectangle_grid(0.0, W, 0.0, H, 0.1)
    p = np.array([[0.5, H / 2], [W - 0.5, H / 2]])  # bisector is x = W / 2
    cells = voronoi_cells(g.points, g.weights, p)
    assert cells.masses == pytest.approx([W * H / 2, W * H / 2])
    assert cells.centroids == pytest.approx(np.array([[W / 4, H / 2], [3 * W / 4, H / 2]]))


def test_coincident_agents_leave_an_empty_cell_that_does_not_move():
    g = rectangle_grid(0.0, W, 0.0, H, 0.1)
    p = np.array([[1.0, 1.0], [1.0, 1.0], [3.0, 2.0]])
    cells = voronoi_cells(g.points, g.weights, p)
    assert cells.masses[1] == 0.0
    assert cells.centroids[1] == pytest.approx(p[1])
    assert cells.masses.sum() == pytest.approx(W * H)


def _random_positions(seed, n=6):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(0.2, W - 0.2, n), rng.uniform(0.2, H - 0.2, n)])


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_grid_converges_to_exact_polygon_cells(seed):
    p = _random_positions(seed)
    masses_ex, centroids_ex, cost_ex = exact_cells(p, DOMAIN)
    errors = []
    for h in (0.1, 0.05, 0.025):
        g = rectangle_grid(0.0, W, 0.0, H, h)
        cells = voronoi_cells(g.points, g.weights, p)
        errors.append(np.abs(cells.centroids - centroids_ex).max())
        # Only samples along the bisectors can be misassigned, so the mass error
        # is O(h); the midpoint rule makes the cost error O(h^2) (observed ~0.1 h^2).
        assert np.abs(cells.masses - masses_ex).max() < h
        assert cells.cost == pytest.approx(cost_ex, rel=0.5 * h**2)
    assert errors[-1] < 0.01
    assert errors[-1] < 0.5 * errors[0]  # refining the grid by 4x at least halves the error
