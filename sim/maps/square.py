import numpy as np
from map_contract import BaseMap

class SquareMap(BaseMap): 
    """
    Square map of area side_length * side_length 
    """
    def __init__(self, side_length, obstacles:bool): 
        self._grid = np.zeros((side_length, side_length), dtype=int)

    @property
    def grid(self):
        return self._grid