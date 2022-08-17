from messages import PoseMessage
from .types import UpdateType

class CustomOdometryMessage:
    def __init__(self, pose: PoseMessage, odometry_key: str, R, timestamp: float):
        self.pose = pose
        self.odometry_key = odometry_key
        self.R = R
        self.timestamp = timestamp
        self.update_type = UpdateType.CUSTOM_POSE

    @staticmethod
    def save(message):
        """"Pickles the message"""
        return message.__class__, message.__dict__

    @staticmethod
    def load(cls, attributes) -> "UpdateMessage":
        """Unpickles the message"""
        obj = cls.__new__(cls)
        obj.__dict__.update(attributes)
        return obj
