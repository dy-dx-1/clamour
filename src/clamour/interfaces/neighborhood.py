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
        self.synced_neighbors = {}
        self.neighbor_synchronization_received = {}
        self.synchronized_active_neighbor_count = 0
        self.changed = False  # Indicates if the neighborhood has changed since the last topology broadcast

    def collect_garbage(self, delay: float = OBSOLESCENCE_DELAY) -> None:
        for id in [id for id, data in self.current_neighbors.items() if data[1] < perf_counter() - delay]:
            del self.current_neighbors[id]

    def is_alone_in_state(self, state: State) -> bool:
        if len(self.current_neighbors) == 0:
            print("No neighbors")

        if state == -1:
            return len(self.current_neighbors) == 0

        return len([neighbor for neighbor in self.current_neighbors.values() if neighbor[2] == state]) == 0

    def add_neighbor(self, device_id: int, timestamp: float, state: int, second_degree_neighbors: list = None) -> None:
        if device_id in self.current_neighbors:
            updated_second_degree_neighbors = second_degree_neighbors or self.current_neighbors[device_id][0]
        else:
            updated_second_degree_neighbors = second_degree_neighbors

        self.current_neighbors[device_id] = (updated_second_degree_neighbors, timestamp, state)
        self.changed = True

    def add_synced_neighbor(self, device_id: int) -> None:
        if device_id not in self.synced_neighbors:
            self.synced_neighbors[device_id] = 1
        else:
            self.synced_neighbors[device_id] += 1

    def remove_synced_neighbor(self, device_id: int) -> None:
        if device_id in self.synced_neighbors:
            self.synced_neighbors[device_id] = 0
        
    def are_neighbors_synced(self) -> bool:
        return all([key in self.synced_neighbors for key in self.current_neighbors.keys()]) \
            and all([times_synced > 5 for times_synced in self.synced_neighbors.values()])

