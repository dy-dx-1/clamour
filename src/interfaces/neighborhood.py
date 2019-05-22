class Neighborhood():
    def __init__(self):
        self.neighbor_list = []
        self.is_alone = True
        self.synchronized_neighbors = []
        self.current_neighbors = {}
        self.neighbor_synchronization_received = []
        self.synchronized_active_neighbor_count = 0

    def add_anchor_to_neighbors(self, id):
        # An id >= 100 means the device is an anchor
        if id >= 100 and id in self.neighbor_list:
            self.neighbor_list.append(id)
