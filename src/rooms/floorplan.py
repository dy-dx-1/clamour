from .room import Room
from pypozyx import Coordinates


class NonexistentRoomException(Exception):
    pass


DEFAULT_ROOMS = [Room('A', 0, 0, (-1, 1), (-1, 1)),
                 Room('B', 7, 8, (-1, 1), (-1, 1)),
                 Room('C', 2, 4, (-1, 1), (-1, 1))]


class Floorplan:
    def __init__(self, rooms: list = None):
        self.rooms = self.rooms_dict_from_list(rooms if rooms is not None else DEFAULT_ROOMS)
        self.paths = []

    def add_path_from_labels(self, labels: tuple) -> None:
        """Both rooms must exist within self.rooms"""

        if all([label in self.rooms for label in labels]):
            self.paths.append(labels)
            self.rooms[labels[0]].add_neighbor(self.rooms[labels[1]])
            self.rooms[labels[1]].add_neighbor(self.rooms[labels[0]])
        else:
            raise NonexistentRoomException()

    def closest_room_label(self, coordinates: Coordinates) -> str:
        distances = {label: room.distance(coordinates) for label, room in self.rooms.items()}
        return min(distances, key=distances.get)

    def __repr__(self) -> str:
        return str(self.paths)

    @staticmethod
    def rooms_dict_from_list(rooms: list) -> dict:
        return {room.label: room for room in rooms}
