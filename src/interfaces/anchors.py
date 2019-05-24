class Anchors():
    def __init__(self):
        self.available_anchors = []
        self.discovery_done = False
        self.anchors_list = [] # Might need to be hardcoded later based on anchor positions in the museum.
        self.anchors_dict = {self.anchors_list[i].data[0]: self.anchors_list[i] for i in self.anchors_list}
