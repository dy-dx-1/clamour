from .room import Room
from .floorplan import FloorPlan


def main():
    rooms = [Room('A', 0, 0, (-1, 1), (-2, 3)),
             Room('B', 7, 8, (-2, 0), (-1, 4)),
             Room('C', 2, 4, (0, 1), (-2, 1))]

    floor_plan = FloorPlan(rooms)

    floor_plan.rooms['A'].add_neighbor(floor_plan.rooms['B'])
    floor_plan.rooms['B'].add_neighbor(floor_plan.rooms['C'])

    floor_plan.generate_paths()

    print(floor_plan)


if __name__ == '__main__':
    main()
