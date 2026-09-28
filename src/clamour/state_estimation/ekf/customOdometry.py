import time
from ...interfaces import Pose

class CustomOdometry:
    def __init__(self, R):
        self._R = R
        self._pose_listener = None

    def update_pose(self, pose: Pose):
        if(self._pose_listener is not None):
            self._pose_listener(self, pose, time.time()) 

    def set_pose_listener(self, callback):
        self._pose_listener = callback
    
    def get_R(self):
        return self._R