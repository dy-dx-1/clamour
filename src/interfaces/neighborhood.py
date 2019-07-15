OBSOLESCENCE_DELAY = 20  # nb of seconds beyond which a neighbor becomes irrelevent

class Neighborhood:
    def __init__(self):
        self.current_neighbors = {}
        self.neighbor_synchronization_received = {}
        self.synchronized_active_neighbor_count = 0

    def collect_garbage(self, current_time: float) -> None:
        for id, data in self.current_neighbors.items():
            if data[1] < current_time - OBSOLESCENCE_DELAY:
                del self.current_neighbors[id]
