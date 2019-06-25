from pypozyx import DeviceCoordinates, Coordinates


class Anchors:
    def __init__(self):
        self.available_anchors = []
        self.anchors_list = [DeviceCoordinates(0x100B, 1, Coordinates(1700, 5170, 1840)),
                             DeviceCoordinates(0x100A, 1, Coordinates(0, 0, 1330)),
                             DeviceCoordinates(0x100C, 1, Coordinates(-1780, 4480, 2230))]

        self.anchors_dict = {anchor.data[0]: anchor for (_, anchor) in enumerate(self.anchors_list)}
