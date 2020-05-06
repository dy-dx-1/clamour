from pypozyx import DeviceCoordinates, Coordinates
import csv
import sys


class Anchors:
    def __init__(self):
        self.available_anchors = []
        # self.available_tags = []
        self.floor_height = 18900 - 300
        self.anchors_list = self.load_anchors_from_csv()

        # print("Anchor list loaded (", len(self.anchors_list), ")")

        self.anchors_dict = {anchor.data[0]: anchor for (_, anchor) in enumerate(self.anchors_list)}

    def load_anchors_from_csv(self) -> list:
        with open(sys.path[0]+'/interfaces/anchors.csv') as r:
            reader = csv.reader(r, delimiter=';')
            next(reader)  # We don't want to read the header, so we skip it
            return [self.add_anchor(anc) for anc in reader]

    def add_anchor(self, anchor_data: list) -> DeviceCoordinates:
        label = int(anchor_data[0], base=16)
        lvl = int(anchor_data[1])
        x = int(anchor_data[2])
        y = int(anchor_data[3])
        z = int(anchor_data[4])

        if lvl == 2:
            z += self.floor_height

        return DeviceCoordinates(label, 1, Coordinates(x, y, z))
