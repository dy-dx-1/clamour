"""
Abstract class contract for creating new maps.

Maps are 2D occupancy grids in metres with RESOLUTION-sized square cells.
Cell (ix, iy) covers [ix*RESOLUTION, (ix+1)*RESOLUTION) along x (same for y) and is stored at grid[iy, ix]
(rows are y), so `imshow(grid, origin='lower')` draws the map the right way up.
"""
from abc import ABC, abstractmethod
import numpy as np

RESOLUTION = 0.01  # m per cell

class BaseMap(ABC):
    @property
    @abstractmethod
    def grid(self) -> np.ndarray:
        """
        2D uint8 occupancy grid: 0 is free, 1 is an obstacle that blocks both motion and ranging.
        Make obstacles several cells thick: line_of_sight samples every half cell and could clip a 1-cell corner.
        TODO: in the future, values between 0 and 1 could attenuate ranging instead of blocking it.
        """

    @property
    def size(self) -> tuple[float, float]:
        """(width, height) of the map in metres."""
        rows, cols = self.grid.shape
        return cols * RESOLUTION, rows * RESOLUTION

    def are_free(self, points) -> np.ndarray:
        """Bool array telling which (N, 2) points (m) fall in free cells. Points outside the map are not free."""
        cells = np.floor(np.asarray(points, dtype=float) / RESOLUTION).astype(int)
        ix, iy = cells[:, 0], cells[:, 1]
        rows, cols = self.grid.shape
        inside = (ix >= 0) & (ix < cols) & (iy >= 0) & (iy < rows)
        free = np.zeros(len(cells), dtype=bool)
        free[inside] = self.grid[iy[inside], ix[inside]] == 0
        return free

    def is_free(self, x: float, y: float) -> bool:
        return bool(self.are_free([(x, y)])[0])

    def line_of_sight(self, p1, p2) -> bool:
        """True if the 2D segment p1 -> p2 (m) crosses no obstacle. Both ends must be in free space."""
        p1, p2 = np.asarray(p1, dtype=float), np.asarray(p2, dtype=float)
        n_samples = int(np.ceil(np.linalg.norm(p2 - p1) / (RESOLUTION / 2))) + 1
        points = p1 + np.linspace(0.0, 1.0, max(n_samples, 2))[:, None] * (p2 - p1)
        return bool(self.are_free(points).all())

    @staticmethod
    def fill_rect(grid: np.ndarray, x0: float, y0: float, x1: float, y1: float, value: int = 1) -> None:
        """Set the rectangle [x0, x1) x [y0, y1) (m) of grid to value. Helper for map constructors."""
        grid[round(y0 / RESOLUTION):round(y1 / RESOLUTION), round(x0 / RESOLUTION):round(x1 / RESOLUTION)] = value
