""""This type of message conveys information about a new state measurement.
It is intended to be passed to a ContextManagedQueue as a pickled class + dictionary tuple.
The state information passed within the message will be used to update the device's EKF."""

from pypozyx import Coordinates
from .types import UpdateType


class UpdateMessage:
    def __init__(self, update_type: UpdateType, timestamp: float, measured_yaw: float=0.0,
                 measured_xyz: Coordinates=None, neighbors: list=None, topology: dict=None):
        self.update_type = update_type
        self.measured_xyz = measured_xyz
        self.measured_yaw = measured_yaw
        self.timestamp = timestamp
        self.neighbors = neighbors if neighbors is not None else []
        self.topology = topology if topology is not None else {}

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
