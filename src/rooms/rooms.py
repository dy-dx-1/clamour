from room import Room
from floorplan import Floorplan

from pypozyx import Coordinates


def main():
    rooms = [Room('A', 0, 0, (-1, 1), (-2, 3)),
             Room('B', 7, 8, (-2, 0), (-1, 4)),
             Room('C', 2, 4, (0, 1), (-2, 1))]

    floor_plan = Floorplan(rooms)

    floor_plan.add_path_from_labels(('A', 'C'))

    test_coords = Coordinates(1, 1)
    print(floor_plan.closest_room_label(test_coords))


if __name__ == '__main__':
    main()
