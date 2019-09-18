from pypozyx import DeviceCoordinates, Coordinates
import csv


class Anchors:
    def __init__(self):
        self.available_anchors = []
        self.anchors_list = self.load_anchors_from_csv()

        print("Anchor list loaded (", len(self.anchors_list), ")")

        self.anchors_dict = {anchor.data[0]: anchor for (_, anchor) in enumerate(self.anchors_list)}

    def load_anchors_from_csv(self) -> list:
        with open('interfaces/anchors.csv') as r:
            reader = csv.reader(r, delimiter=';')
            next(reader)  # We don't want to read the header, so we skip it
            return [self.add_anchor(anc) for anc in reader]

    @staticmethod
    def add_anchor(anchor_data: list) -> DeviceCoordinates:
        label = int(anchor_data[0], base=16)
        x = int(anchor_data[1])
        y = int(anchor_data[2])
        z = int(anchor_data[3])

        print("Adding anchor:", label, x, y, z)

        return DeviceCoordinates(label, 1, Coordinates(x, y, z))
