from .containers import Coordinates
from ..config import ANCHORS

class Anchors:
    def __init__(self):
        self.floor_height = 18900 - 300
        self.anchors_dict = self.load_anchors_from_config() # Dict of ALL deployed anchors {anchor_id: Coordinates()}

    def load_anchors_from_config(self) -> dict:
        anchor_dict = {} 
        for anc_dict in ANCHORS: 
            if anc_dict['level'] == 2:
                z += self.floor_height
            anchor_dict[anc_dict['id']] = Coordinates(anc_dict['x'], anc_dict['y'], anc_dict['z']) 
        return anchor_dict