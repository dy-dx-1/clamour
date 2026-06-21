from .containers import Coordinates
from ..config import ANCHORS

class Anchors:
    def __init__(self):
        self.floor_height = 18900 - 300
        self.anchors_dict = self.load_anchors_from_config() # Dict of ALL deployed anchors {anchor_id: Coordinates()}

    def load_anchors_from_config(self) -> dict:
        anchor_dict = {} 
        for anc_dict in ANCHORS: 
            if anc_dict['level'] == 2: # NOTE: 2026-06-16, this was from old code with anchors.csv, no idea what floor level does if anything
                z += self.floor_height
            # TODO: 2026-06-18, replaced DeviceCoordinates by a simple dict representing anchors. confirm works and then delete this commented part
            #anchor_list.append(DeviceCoordinates(network_id = anc_dict['id'],
            #                  flag = 1, 
            #                  pos = Coordinates(anc_dict['x'], anc_dict['y'], anc_dict['z']) ) ) 
            anchor_dict[anc_dict['id']] = Coordinates(anc_dict['x'], anc_dict['y'], anc_dict['z']) 