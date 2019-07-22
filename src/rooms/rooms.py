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


class FloorPlan:
    def __init__(self, rooms: list):
        self.rooms = self.rooms_dict_from_list(rooms)
        self.paths = []

    def generate_paths(self) -> None:
        for label, room in self.rooms.items():
            for neighbor in room.neighbors:
                self.paths.append((label, neighbor.label))

    def __repr__(self):
        return str(self.paths)

    @staticmethod
    def rooms_dict_from_list(rooms: list) -> dict:
        return {room.label: room for room in rooms}


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
