import time
from messages import PoseMessage

class CustomOdometry:
    def __init__(self, R, key):
        self._R = R
        self._key = key
        self._pose_listener = None

    def update_pose(self, pose: PoseMessage):
        if(self._pose_listener is not None):
            self._pose_listener(self, pose, time.time()) 

    def set_pose_listener(self, callback):
        self._pose_listener = callback
    
    def get_R(self):
        return self._R

    def get_key(self):
        return self._key