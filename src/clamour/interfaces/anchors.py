from ..config import ANCHORS
import numpy as np 

class Anchors:
    def __init__(self):
        self.floor_height = 1890 - 30
        self.anchors_dict = self.load_anchors_from_config() # Dict of ALL deployed anchors {anchor_id: (x, y, z)}

    def load_anchors_from_config(self) -> dict[int, tuple[float, float, float]]:
        anchor_dict = {} 
        for anc_dict in ANCHORS: 
            x, y, z = anc_dict['x'], anc_dict['y'], anc_dict['z']
            if anc_dict['level'] == 2:
                z += self.floor_height
            anchor_dict[anc_dict['id']] = (x, y, z)
        return anchor_dict

    def get_centroid_for(self, *anchor_ids: int) -> tuple[float, float, float]:
        """
        Evaluates the centroid for all anchor IDs passed to the function. 
        NOTE: Does not validate anchor ID, only pass anchors or will raise an error!
        """
        points = np.array([self.anchors_dict[id] for id in anchor_ids])
        mean = np.mean(points, axis=0)
        return float(mean[0]), float(mean[1]), float(mean[2])
