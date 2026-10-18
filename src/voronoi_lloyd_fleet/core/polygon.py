"""Exact Voronoi cells of a convex domain, by half-plane clipping.

Used as the test oracle for the grid geometry in the convex, uniform-density
case. The cell of agent i is the domain clipped by the half-planes
{q : (p_j - p_i) . q <= (|p_j|^2 - |p_i|^2) / 2}, one per other agent j.

Polygons are (k, 2) arrays of vertices in counter-clockwise order.
"""

import numpy as np


def clip_halfplane(polygon: np.ndarray, a: np.ndarray, b: float) -> np.ndarray:
    """Keep the part of a convex polygon where a . q <= b (one Sutherland–Hodgman pass)."""
    if len(polygon) == 0:
        return polygon
    side = polygon @ a - b  # <= 0 inside
    kept = []
    for k in range(len(polygon)):
        p, q = polygon[k], polygon[(k + 1) % len(polygon)]
        sp, sq = side[k], side[(k + 1) % len(polygon)]
        if sp <= 0:
            kept.append(p)
        if (sp < 0 < sq) or (sq < 0 < sp):
            t = sp / (sp - sq)
            kept.append(p + t * (q - p))
    return np.array(kept).reshape(-1, 2)


def voronoi_cell_polygon(i: int, positions: np.ndarray, domain: np.ndarray) -> np.ndarray:
    """Exact Voronoi cell of agent i inside a convex domain polygon."""
    positions = np.asarray(positions, dtype=float)
    p_i = positions[i]
    cell = np.asarray(domain, dtype=float)
    for j, p_j in enumerate(positions):
        if j == i:
            continue
        a = p_j - p_i
        if not a.any():
            continue  # coincident agents: the bisector is undefined, so it doesn't clip
        cell = clip_halfplane(cell, a, 0.5 * (p_j @ p_j - p_i @ p_i))
    return cell


def _edge_terms(polygon: np.ndarray):
    x, y = polygon[:, 0], polygon[:, 1]
    xn, yn = np.roll(x, -1), np.roll(y, -1)
    return x, y, xn, yn, x * yn - xn * y


def polygon_area_centroid(polygon: np.ndarray) -> tuple[float, np.ndarray]:
    """Area and centroid of a simple polygon (shoelace formulas)."""
    x, y, xn, yn, cross = _edge_terms(polygon)
    area = 0.5 * cross.sum()
    cx = ((x + xn) * cross).sum() / (6.0 * area)
    cy = ((y + yn) * cross).sum() / (6.0 * area)
    return float(area), np.array([cx, cy])


def polygon_second_moment(polygon: np.ndarray, point: np.ndarray) -> float:
    """Integral of ||q - point||^2 over a simple polygon."""
    x, y, xn, yn, cross = _edge_terms(polygon)
    polar_origin = (cross * (x * x + x * xn + xn * xn + y * y + y * yn + yn * yn)).sum() / 12.0
    area, centroid = polygon_area_centroid(polygon)
    point = np.asarray(point, dtype=float)
    # Expand ||q - p||^2 = ||q||^2 - 2 p.q + ||p||^2 and integrate each term.
    return float(polar_origin - 2.0 * area * (point @ centroid) + area * (point @ point))


def exact_cells(positions: np.ndarray, domain: np.ndarray):
    """Exact masses, centroids and coverage cost of all agents' cells (uniform density)."""
    positions = np.asarray(positions, dtype=float)
    masses, centroids, cost = [], [], 0.0
    for i in range(len(positions)):
        cell = voronoi_cell_polygon(i, positions, domain)
        area, centroid = polygon_area_centroid(cell)
        masses.append(area)
        centroids.append(centroid)
        cost += polygon_second_moment(cell, positions[i])
    return np.array(masses), np.array(centroids), cost
