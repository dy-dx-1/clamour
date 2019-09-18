from enum import Enum
from time import perf_counter


OBSOLESCENCE_DELAY = 20  # nb of seconds beyond which a neighbor becomes irrelevant


# TODO: import properly
class State(Enum):
    INITIALIZATION = 0
    SYNCHRONIZATION = 1
    SCHEDULING = 2
    TASK = 3
    LISTEN = 4


class Neighborhood:
    def __init__(self):
        self.current_neighbors = {}
        self.synced_neighbors = set()
        self.neighbor_synchronization_received = {}
        self.synchronized_active_neighbor_count = 0

    def collect_garbage(self, delay: float = OBSOLESCENCE_DELAY) -> None:
        for id in [id for id, data in self.current_neighbors.items() if data[1] < perf_counter() - delay]:
            del self.current_neighbors[id]

    def is_alone_in_state(self, state: State) -> bool:
        print(self.current_neighbors)
        return len([neighbor for neighbor in self.current_neighbors.values() if neighbor[2] == state]) == 0

    def add_neighbor(self, device_id: int, second_degree_neighbors: list, timestamp: float, state: int) -> None:
        self.current_neighbors[device_id] = (second_degree_neighbors, timestamp, state)

    def add_synced_neighbor(self, device_id: int) -> None:
        if device_id not in self.synced_neighbors:
            print("Adding", device_id, "to synced neibs:", "self.synced_neighbors")
        self.synced_neighbors.add(device_id)

    def remove_synced_neighbor(self, device_id: int) -> None:
        if device_id in self.synced_neighbors:
            print("Removing", device_id, "from synced neibs:", self.synced_neighbors)
        self.synced_neighbors.discard(device_id)

    def are_neighbors_synced(self) -> bool:
        return all([key in self.synced_neighbors for key in self.current_neighbors.keys()])

