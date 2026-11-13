from ..custom_terminal import print 

from .tag import Tag
from .containers import Pose
from .anchors import Anchors

import numpy as np
from typing import Literal

# The fake neighbors / anchors the tag 'sees' 
ACTIVE_TAGS = [100, 101, 102]
VISIBLE_ANCHORS = [1,2,3,4,5,6,7,8]
ANCHORS_DICT = Anchors().anchors_dict
# The fake position 
FAKE_POS = (286, 296, 40)

class SyntheticTag(Tag):
    """
    A synthetic tag for testing stationary IMU behavior outside of the lab
    """
    def __init__(self, tag_id:int):

        self._id = tag_id        # NOTE: It's expected that all ID's used are max 2bytes (<0xFFFF). Should be enforced by config.py
        self._active_tags = {tag_id:1 for tag_id in ACTIVE_TAGS}      # keeps track of nearby tags and when they were last seen 
        self._available_anchors = VISIBLE_ANCHORS # keeps track of currently in-range anchors 
        self._pose = Pose()             # Tag position, orientation, and covariance

        print(f"SYNTHETIC TAG STARTED!", 'ok', 'device') 
        print(f"TAG ID: {self._id}", 'ok', 'device')

    def __enter__(self): 
        return self 
    
    def __exit__(self, exc_type, exc_val, exc_tb): 
        print("SyntheticTag closed!", 'ok', 'device')

    @property
    def tag_id(self)->int:
        return self._id
    
    @property
    def active_tags(self)->set[int]:
        return set(self._active_tags.keys()) 
    
    @property
    def available_anchors(self)->set[int]:
        return set(self._available_anchors) 

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id: int) -> bool:
        return True if device_id<=10 else False 

    def add_neighbor(tag_id: int) -> None: 
        pass 

    def clear_anchors(self) -> None:
        pass

    def reset(self) -> None:
        pass
    
    def get_device_list(self, discovery_type:Literal["all", "anchor", "tag"]) -> set[int]:
        devices = [] 
        if discovery_type=="all" or discovery_type=="anchor": 
            for anchor in VISIBLE_ANCHORS: 
                devices.append(anchor) 
        if discovery_type=="all" or discovery_type=="tag": 
            for tag in self.active_tags: 
                devices.append(tag) 
        return set(devices) 

    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------
    def broadcast(self, payload:list) -> bool: 
        return True 
    
    def receive_data(self) -> tuple[int, bytes]:
        return None, b''  

    ### -------------------------------------------- LOCALIZATION --------------------------------------------

    @property
    def pose(self) -> Pose:
        return self._pose

    @pose.setter
    def pose(self, new_pose: Pose) -> None:
        self._pose = new_pose 

    @property
    def orientation(self) -> list:
        return self._pose.angles 

    def ranging(self, target_id:int) -> tuple[int|None, Pose|None]:
        dist_vect = np.array(FAKE_POS) - np.array(ANCHORS_DICT[target_id])
        return round(np.linalg.norm(dist_vect)), None 