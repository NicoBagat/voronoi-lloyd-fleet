"""Mission-space description for the simulator."""

from dataclasses import dataclass

import numpy as np

from voronoi_lloyd_fleet.core.geometry import Grid, rectangle_grid


@dataclass(frozen=True)
class RectangleWorld:
    """A convex, obstacle-free rectangular mission space sampled with step h."""

    xmin: float
    xmax: float
    ymin: float
    ymax: float
    h: float

    @property
    def area(self) -> float:
        return (self.xmax - self.xmin) * (self.ymax - self.ymin)

    def grid(self) -> Grid:
        return rectangle_grid(self.xmin, self.xmax, self.ymin, self.ymax, self.h)

    def polygon(self) -> np.ndarray:
        """Boundary vertices in counter-clockwise order."""
        return np.array(
            [
                [self.xmin, self.ymin],
                [self.xmax, self.ymin],
                [self.xmax, self.ymax],
                [self.xmin, self.ymax],
            ]
        )


def initial_positions(
    world: RectangleWorld,
    n: int,
    rng: np.random.Generator,
    box: tuple[float, float, float, float] | None = None,
) -> np.ndarray:
    """Draw n positions uniformly in `box` = (x0, x1, y0, y1), the whole world by default."""
    x0, x1, y0, y1 = box if box is not None else (world.xmin, world.xmax, world.ymin, world.ymax)
    if not (world.xmin <= x0 < x1 <= world.xmax and world.ymin <= y0 < y1 <= world.ymax):
        raise ValueError(f"initial box {box} is not a non-empty part of the world")
    return np.column_stack([rng.uniform(x0, x1, n), rng.uniform(y0, y1, n)])
