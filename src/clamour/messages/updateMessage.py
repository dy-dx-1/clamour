from ..interfaces import Coordinates
from .types import UpdateType

class UpdateMessage:
    """
    Message containing information about new measurements. 
    It is intended to be passed to a ContextManagedQueue as a pickled class + dictionary tuple. 
    The state information passed within the message will be used to update the device's state estimation.
    
    This message is expected to be used with UpdateType.PEDOMETER, RANGING and TOPOLOGY
    Although not all require the same parameters. 
    - PEDOMETER: update_type, timestamp and yaw 
    - RANGING: All parameters, but note that the last yaw is directly passed, no new info on it though. 
    - TOPOLOGY: All but ranging_data, measured_yaw and neighbors
    
    ARGS:
    - update_type
    - timestamp
    - synchronized_clock
    - offset
    - ranging_data: list of ranging values in format (target_id, range_in_mm) or []
    - measured_yaw
    - slots 
    - neighbors: list or None 
    - topology: dict 
    """
    def __init__(self, update_type: UpdateType, timestamp: float,
                 synchronized_clock: float=0.0, offset: float=0.0,
                 ranging_data: list[tuple]|None = None, measured_yaw: float=0.0,
                 slots: list=None, neighbors: list=None, topology: dict=None):
        self.update_type = update_type
        self.timestamp = timestamp
        self.synchronized_clock = synchronized_clock
        self.offset = offset

        self.ranging_data = ranging_data
        self.measured_yaw = measured_yaw

        self.slots = slots
        self.neighbors = neighbors if neighbors is not None else []
        self.topology = topology if topology is not None else {}

    @staticmethod
    def save(message):
        """"Pickles the message"""
        return message.__class__, message.__dict__

    @staticmethod
    def load(cls, attributes) -> UpdateMessage:
        """Unpickles the message"""
        obj = cls.__new__(cls)
        obj.__dict__.update(attributes)
        return obj