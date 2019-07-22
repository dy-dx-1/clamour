from math import sqrt
from pypozyx import Coordinates
from typing import Union


class Room:
    def __init__(self,  label: str, x: float, y: float, x_lim: tuple, y_lim: tuple):
        """x, y are the center's coordinates;
        x_lim, y_lim are the limits of the room relative to its own center."""

        self.label = label
        self.x = x
        self.y = y
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.neighbors = []

    def add_neighbor(self, neighbor: 'Room') -> None:
        self.neighbors.append(neighbor)

    def within_bounds(self, coordinates: Coordinates) -> bool:
        return (self.x + self.x_lim[0] <= coordinates.x <= self.x + self.x_lim[1]) \
            and (self.y + self.y_lim[0] <= coordinates.y <= self.y + self.y_lim[1])

    def within_neighbor_bounds(self, coordinate: Coordinates) -> Union[str, None]:
        for neighbor in self.neighbors:
            if neighbor.within_bounds(coordinate):
                return neighbor.label

        return None

    def distance(self, coordinates: Coordinates):
        return sqrt((self.x - coordinates.x)**2 + (self.y - coordinates.y)**2)

    def __repr__(self):
        return "(" + str(self.label) + ": " + str(self.x) + ", " + str(self.y) + ")"
