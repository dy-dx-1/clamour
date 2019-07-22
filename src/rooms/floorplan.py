from room import Room
from pypozyx import Coordinates


class NonexistentRoomException(Exception):
    pass


class Floorplan:
    def __init__(self, rooms: list):
        self.rooms = self.rooms_dict_from_list(rooms)
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

    def __repr__(self):
        return str(self.paths)

    @staticmethod
    def rooms_dict_from_list(rooms: list) -> dict:
        return {room.label: room for room in rooms}
