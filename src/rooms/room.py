import numpy as np
from math import sqrt, sin, cos
from pypozyx import Coordinates
from typing import Union


class Room:
    def __init__(self, label: str, neighbors: list, x: int, y: int, x_lim: tuple, y_lim: tuple, theta: float = 0.0):
        """x, y are the center's coordinates;
        x_lim, y_lim are the limits of the room relative to its own center."""

        self.label = label
        self.x = x
        self.y = y
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.theta = theta  # Orientation of the local X-axis of the room relative to the global X-axis
        self.neighbors = neighbors
        self.transformation_matrix = self.calculate_transformation_matrix()

    def calculate_transformation_matrix(self) -> np.ndarray:
        """The matrix for transforming from local to world coordinates."""

        rotation = np.array([[cos(self.theta), -sin(self.theta), 0], [sin(self.theta), cos(self.theta), 0], [0, 0, 1]])
        translation = np.array([[1, 0, self.x], [0, 1, self.y], [0, 0, 1]])
        return np.matmul(rotation, translation)

    def add_neighbor(self, neighbor: 'Room') -> None:
        self.neighbors.append(neighbor)

    def within_bounds(self, world_coordinates: Coordinates) -> bool:
        local_coordinates = self.transform_to_local_coordinates(world_coordinates)
        return (-self.x_lim[0] <= local_coordinates.x <= self.x_lim[1]) \
            and (-self.y_lim[0] <= local_coordinates.y <= self.y_lim[1])

    def transform_to_local_coordinates(self, world_coordinates: Coordinates):
        # World coordinates need to be transformed to 2D homogeneous (x, y, 1)

        homogeneous_xy_coordinates = Coordinates(world_coordinates[0], world_coordinates[1], 1)
        return Coordinates(*np.matmul(np.linalg.inv(self.transformation_matrix), homogeneous_xy_coordinates))

    def within_neighbor_bounds(self, coordinate: Coordinates, rooms: dict) -> Union[str, None]:
        for neighbor in self.neighbors:
            if rooms[neighbor].within_bounds(coordinate):
                return rooms[neighbor].label

        return None

    def distance(self, coordinates: Coordinates):
        return sqrt((self.x - coordinates.x)**2 + (self.y - coordinates.y)**2)

    def __repr__(self):
        return "(" + str(self.label) + ": " + str(self.x) + ", " + str(self.y) + ")"
