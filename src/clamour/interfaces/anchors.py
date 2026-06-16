from .containers import Coordinates, DeviceCoordinates
from ..config import ANCHORS

class Anchors:
    def __init__(self):
        self.available_anchors = []
        self.floor_height = 18900 - 300
        self.anchors_list = self.load_anchors_from_config()
        self.anchors_dict = {anchor.network_id: anchor for anchor in self.anchors_list}

    def load_anchors_from_config(self) -> list:
        anchor_list = list() 
        for anc_dict in ANCHORS: 
            if anc_dict['level'] == 2: # NOTE: 2026-06-16, this was from old code with anchors.csv, no idea what floor level does if anything
                z += self.floor_height
            
            anchor_list.append(DeviceCoordinates(network_id = anc_dict['id'],
                              flag = 1, 
                              pos = Coordinates(anc_dict['x'], anc_dict['y'], anc_dict['z']) ) ) 