import csv
from .room import Room
from pypozyx import Coordinates


class NonexistentRoomException(Exception):
    pass


class Floorplan:
    def __init__(self):
        self.rooms = self.rooms_dict_from_list(self.load_rooms_from_csv())
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
        return str(self.rooms)

    @staticmethod
    def load_rooms_from_csv() -> list:
        room_list = []
        with open('rooms/rooms.csv') as r:
            reader = csv.reader(r, delimiter=';')
            next(reader)  # We don't want to read the header
            for room in reader:
                room_list.append(Room(label=room[0], x=int(room[1]), y=int(room[2]), x_lim=(int(room[3]), int(room[4])),
                                      y_lim=(int(room[5]), int(room[6])), theta=float(room[7])))
        return room_list

    @staticmethod
    def rooms_dict_from_list(rooms: list) -> dict:
        return {room.label: room for room in rooms}
