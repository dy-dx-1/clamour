class Neighborhood:
    def __init__(self):
        self.current_neighbors = {}
        self.neighbor_synchronization_received = {}
        self.synchronized_active_neighbor_count = 0
