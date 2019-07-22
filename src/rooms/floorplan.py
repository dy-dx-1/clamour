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
