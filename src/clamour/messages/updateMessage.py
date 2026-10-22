from ..sensor_events import RangeObservation
from .types import UpdateType

class UpdateMessage:
    """
    Message containing information about new measurements. 
    It is intended to be passed to a ContextManagedQueue as a pickled class + dictionary tuple. 
    The state information passed within the message will be used to update the device's state estimation.
    
    This message is expected to be used with UpdateType.PEDOMETER, RANGING and TOPOLOGY.
    The ranging contract is now unified around a single list of RangeObservation
    objects so each measurement knows whether it is from an anchor or a tag.

    ARGS:
    - update_type
    - timestamp_ns: canonical estimator-frame event time in nanoseconds
    - measured_yaw
    - range_observations: list[RangeObservation]
    - slots
    - topology: dict
    - sent_ns: estimator-frame time at which the message was enqueued (diagnostics only)
    """

    def __init__(self, update_type: UpdateType, timestamp_ns: int,
                 measured_yaw: float = 0.0,
                 slots: list | None = None, topology: dict | None = None,
                 range_observations: list[RangeObservation] | None = None,
                 sent_ns: int | None = None):
        self.update_type = update_type

        # Canonical estimator-time value in nanoseconds. Producers should stamp
        # events in estimator time before enqueueing them.
        self.timestamp = int(round(timestamp_ns))

        self.measured_yaw = measured_yaw
        self.range_observations = range_observations or []

        self.slots = slots
        self.topology = topology if topology is not None else {}
        self.sent_ns = sent_ns

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
