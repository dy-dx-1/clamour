from dataclasses import dataclass

from ..interfaces import Pose
from .types import UpdateType


@dataclass(frozen=True)
class RangeObservation:
    """One UWB range with its own acquisition time on the estimator clock."""

    target_id: int
    distance_cm: float
    event_time_ns: int
    is_anchor: bool
    target_pose: Pose | None = None

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
    """

    def __init__(self, update_type: UpdateType, timestamp_ns: int | None = None,
                 synchronized_clock: float = 0.0, offset: float = 0.0,
                 measured_yaw: float = 0.0,
                 slots: list | None = None, topology: dict | None = None,
                 *, arrival_time_ns: int | None = None,
                 source_clock_id: str = "host_monotonic", source_timestamp: int | None = None,
                 time_sigma_ns: int | None = None,
                 range_observations: list[RangeObservation] | None = None):
        self.update_type = update_type

        if timestamp_ns is None:
            raise ValueError("timestamp_ns is required")

        # Canonical estimator-time value in nanoseconds. Producers should stamp
        # events in estimator time before enqueueing them.
        self.timestamp = int(round(timestamp_ns))
        self.arrival_time_ns = arrival_time_ns
        self.source_clock_id = source_clock_id
        self.source_timestamp = source_timestamp
        self.time_sigma_ns = time_sigma_ns

        # Compatibility shim: older code still expects a float-style timestamp
        # attribute, but the estimator-facing contract is nanoseconds.
        self.synchronized_clock = synchronized_clock
        self.offset = offset

        self.measured_yaw = measured_yaw
        self.range_observations = range_observations or []

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
