import numpy as np
from .map_contract import BaseMap, RESOLUTION

class SquareMap(BaseMap):
    """
    Square room of side_length x side_length metres, enclosed by walls.
    With obstacles, adds an interior wall at mid-width from the bottom wall up to 60% of the side, creating NLOS zones.
    """
    WALL_THICKNESS = 0.1  # m

    def __init__(self, side_length: float, obstacles: bool):
        s, t = side_length, self.WALL_THICKNESS
        cells = round(s / RESOLUTION)
        grid = np.zeros((cells, cells), dtype=np.uint8)
        for rect in ((0, 0, s, t), (0, s - t, s, s), (0, 0, t, s), (s - t, 0, s, s)):  # bottom, top, left, right
            self.fill_rect(grid, *rect)
        if obstacles:
            self.fill_rect(grid, s / 2 - t / 2, 0, s / 2 + t / 2, 0.6 * s)
        self._grid = grid

    @property
    def grid(self) -> np.ndarray:
        return self._grid
