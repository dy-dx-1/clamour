from .containers import Coordinates
from ..config import ANCHORS
import numpy as np 

class Anchors:
    def __init__(self):
        self.floor_height = 18900 - 300
        self.anchors_dict = self.load_anchors_from_config() # Dict of ALL deployed anchors {anchor_id: Coordinates()}

    def load_anchors_from_config(self) -> dict[int, Coordinates]:
        anchor_dict = {} 
        for anc_dict in ANCHORS: 
            if anc_dict['level'] == 2:
                z += self.floor_height
            anchor_dict[anc_dict['id']] = Coordinates(anc_dict['x'], anc_dict['y'], anc_dict['z']) 
        return anchor_dict

    def get_centroid_for(self, *anchor_ids:int)->Coordinates:
        """
        Evaluates the centroid for all anchor IDs passed to the function. 
        NOTE: Does not validate anchor ID, only pass anchors or will raise an error!
        """
        points = np.array( [self.anchors_dict[id].data for id in anchor_ids] ) 
        mean = np.mean(points, axis=0)
        return Coordinates(mean[0], mean[1], mean[2])