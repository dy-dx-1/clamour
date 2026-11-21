"""
Abstractclass contract for creating new maps 

All maps must have cm-level resolution. 
"""
from abc import ABC, abstractmethod
import numpy as np

class BaseMap(ABC):
    @property
    @abstractmethod
    def grid(self)->np.ndarray: 
        """
        Returns a grid of centimeter-resolution where 0 is a free slot on the map and 1 is an obstacle that fully blocks ranging. 
        TODO: in the future could add 0.5 for an obstacle that let's the ranging pass but attenuates it? 
        """