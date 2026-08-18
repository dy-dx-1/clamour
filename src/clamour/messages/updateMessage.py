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

    NOTE: As of 2026-08-17, tentatively separating anchors and tags list, even if same format, to facilitate different covariance info to be added with FG? Maybe there's a cleaner solution apparent when more of the system is constructed. Currently separating them also helps in the EKF to eval if can trilaterate easily.  

    ARGS:
    - update_type
    - timestamp
    - synchronized_clock
    - offset
    - anchors_ranging_data: list of ranging to anchors in format (TargetCoordinates, range_in_mm) 
    - tags_ranging_data: list of ranging to tags in format (TargetCoordinates, range_in_mm) 
    - measured_yaw
    - slots 
    - topology: dict 
    """
    def __init__(self, update_type: UpdateType, timestamp: float,
                 synchronized_clock: float=0.0, offset: float=0.0,
                 anchors_ranging_data: list[tuple[Coordinates, int]]|None = None, tags_ranging_data: list[tuple[Coordinates, int]]|None = None,
                 measured_yaw: float=0.0,
                 slots: list=None, topology: dict=None):
        self.update_type = update_type
        self.timestamp = timestamp
        self.synchronized_clock = synchronized_clock
        self.offset = offset

        self.anchors_ranging_data = anchors_ranging_data
        self.tags_ranging_data = tags_ranging_data
        self.measured_yaw = measured_yaw

        self.slots = slots
        self.topology = topology if topology is not None else {}

    @staticmethod
    def save(message):
        """"Pickles the message"""
        return message.__class__, message.__dict__

    @staticmethod
    def load(cls, attributes) -> 'UpdateMessage':
        """Unpickles the message"""
        obj = cls.__new__(cls)
        obj.__dict__.update(attributes)
        return obj