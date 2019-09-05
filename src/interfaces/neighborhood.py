from time import perf_counter


OBSOLESCENCE_DELAY = 20  # nb of seconds beyond which a neighbor becomes irrelevant


class Neighborhood:
    def __init__(self):
        self.current_neighbors = {}
        self.synced_neighbors = set()
        self.neighbor_synchronization_received = {}
        self.synchronized_active_neighbor_count = 0

    def collect_garbage(self) -> None:
        for id in [id for id, data in self.current_neighbors.items() if data[1] < perf_counter() - OBSOLESCENCE_DELAY]:
            del self.current_neighbors[id]

    def is_alone(self) -> bool:
        return len(self.current_neighbors) == 0

    def add_neighbor(self, device_id: int, second_degree_neighbors: list, timestamp: float) -> None:
        # print('(STEP) Adding neighbor')
        self.current_neighbors[device_id] = (second_degree_neighbors, timestamp)

    def add_synced_neighbor(self, device_id: int) -> None:
        self.synced_neighbors.add(device_id)

    def remove_synced_neighbor(self, device_id: int) -> None:
        self.synced_neighbors.discard(device_id)

    def are_neighbors_synced(self) -> bool:
        return all([key in self.synced_neighbors for key in self.current_neighbors.keys()])

