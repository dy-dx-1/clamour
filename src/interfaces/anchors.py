from pypozyx import DeviceCoordinates, Coordinates
import csv


class Anchors:
    def __init__(self):
        self.available_anchors = []
        # self.anchors_list = [DeviceCoordinates(0x100B, 1, Coordinates(1700, 5170, 1840)),
        #                      DeviceCoordinates(0x100A, 1, Coordinates(0, 0, 1330)),
        #                      DeviceCoordinates(0x100C, 1, Coordinates(-1780, 4480, 2230)),
        #                      DeviceCoordinates(0x1006, 1, Coordinates(3964, 1989, 764))]
        self.anchors_list = self.load_anchors_from_csv()

        print("Anchor list loaded (",len(self.anchors_list), ")")

        self.anchors_dict = {anchor.data[0]: anchor for (_, anchor) in enumerate(self.anchors_list)}

    def load_anchors_from_csv(self) -> list:
        with open('interfaces/anchors.csv') as r:
            reader = csv.reader(r, delimiter=';')
            next(reader)  # We don't want to read the header, so we skip it
            return [self.add_anchor(anc) for anc in reader]

    def add_anchor(self, anchor_data: list) -> DeviceCoordinates:
        label = hex(int(anchor_data[0]))
        x = int(anchor_data[1])
        y = int(anchor_data[2])
        z = int(anchor_data[3])
        return DeviceCoordinates(label, 1, Coordinates(x, y, z))
