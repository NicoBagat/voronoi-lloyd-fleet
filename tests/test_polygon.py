import numpy as np
import pytest

from voronoi_lloyd_fleet.core.polygon import (
    clip_halfplane,
    exact_cells,
    polygon_area_centroid,
    polygon_second_moment,
    voronoi_cell_polygon,
)

UNIT_SQUARE = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])


def test_area_and_centroid_of_known_shapes():
    area, c = polygon_area_centroid(UNIT_SQUARE)
    assert area == pytest.approx(1.0)
    assert c == pytest.approx([0.5, 0.5])
    triangle = np.array([[0.0, 0.0], [3.0, 0.0], [0.0, 3.0]])
    area, c = polygon_area_centroid(triangle)
    assert area == pytest.approx(4.5)
    assert c == pytest.approx([1.0, 1.0])


def test_second_moment_of_unit_square():
    assert polygon_second_moment(UNIT_SQUARE, [0.5, 0.5]) == pytest.approx(1.0 / 6.0)
    # Parallel-axis rule: moving the point by d adds area * d^2.
    assert polygon_second_moment(UNIT_SQUARE, [0.0, 0.0]) == pytest.approx(1.0 / 6.0 + 0.5)


def test_clip_keeps_the_inside_half():
    half = clip_halfplane(UNIT_SQUARE, np.array([1.0, 0.0]), 0.5)  # x <= 0.5
    area, c = polygon_area_centroid(half)
    assert area == pytest.approx(0.5)
    assert c == pytest.approx([0.25, 0.5])
    assert len(clip_halfplane(UNIT_SQUARE, np.array([1.0, 0.0]), -1.0)) == 0


def test_cells_partition_the_domain_and_contain_their_agent():
    rng = np.random.default_rng(3)
    domain = np.array([[0.0, 0.0], [5.0, 0.0], [5.0, 2.0], [0.0, 2.0]])
    p = np.column_stack([rng.uniform(0, 5, 7), rng.uniform(0, 2, 7)])
    masses, _, _ = exact_cells(p, domain)
    assert masses.sum() == pytest.approx(10.0)
    for i in range(len(p)):
        cell = voronoi_cell_polygon(i, p, domain)
        # p_i is inside its convex cell: on the left of every counter-clockwise edge.
        edges = np.roll(cell, -1, axis=0) - cell
        rel = p[i] - cell
        assert np.all(edges[:, 0] * rel[:, 1] - edges[:, 1] * rel[:, 0] >= -1e-12)
