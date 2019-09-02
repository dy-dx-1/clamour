from time import perf_counter


OBSOLESCENCE_DELAY = 20  # nb of seconds beyond which a neighbor becomes irrelevent


class Neighborhood:
    def __init__(self):
        self.current_neighbors = {}
        self.synced_neighbors = set()
        self.neighbor_synchronization_received = {}
        self.synchronized_active_neighbor_count = 0

    def collect_garbage(self) -> None:
        to_del = [id for id, data in self.current_neighbors.items() if data[1] < perf_counter() - OBSOLESCENCE_DELAY]
        if len(to_del) > 0:
            print("Collecting garbage: ", to_del)
        for id in to_del:
            del self.current_neighbors[id]

    def is_alone(self):
        return len(self.current_neighbors) == 0

    def add_neighbor(self, device_id: int, second_degree_neighbors: list, timestamp: float) -> None:
        # print('(STEP) Adding neighbor')
        self.current_neighbors[device_id] = (second_degree_neighbors, timestamp)

    def add_synced_neighbor(self, device_id: int):
        if device_id not in self.synced_neighbors:
            print(f"Adding {device_id} to synced_neighbors ({self.synced_neighbors})")
        self.synced_neighbors.add(device_id)

    def remove_synced_neighbor(self, device_id: int):
        if device_id in self.synced_neighbors:
            print(f"Removing {device_id} from synced_neighbors ({self.synced_neighbors})")
        self.synced_neighbors.discard(device_id)

    def are_neighbors_synced(self):
        synced = all([key in self.synced_neighbors for key in self.current_neighbors.keys()])
        if synced:
            print(f"Synced neighbors: {self.synced_neighbors}, neighbors: {self.current_neighbors.keys()}")
        return synced

