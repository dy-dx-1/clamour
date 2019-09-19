import csv
import math
from .room import Room


class RoomLoader:
    @staticmethod
    def load_rooms_from_csv() -> list:
        with open('rooms/chambord_rooms.csv') as r:
            reader = csv.reader(r, delimiter=';')
            next(reader)  # We don't want to read the header, so we skip it
            return [RoomLoader.create_room(room) for room in reader]

    @staticmethod
    def create_room(room_data: list) -> Room:
        label = room_data[0]
        x = int(room_data[5]*10)
        y = int(room_data[6]*10)
        x_lim = (int(room_data[3]*10), int(room_data[3]*10))
        y_lim = (int(room_data[4]*10), int(room_data[4]*10))
        theta = RoomLoader.orientation_from_corners(room_data[7:])

        return Room(label, x, y, x_lim, y_lim, theta)

    @staticmethod
    def orientation_from_corners(corners: list) -> float:
        diff_x = int(corners[2]) - int(corners[4])
        diff_y = int(corners[3]) - int(corners[5])

        return math.atan2(diff_y, diff_x)
