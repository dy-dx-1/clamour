"""Timestamped sensor measurements shared by acquisition and estimation code.

These types deliberately live outside ``state_estimation`` and ``messages`` so
the estimator coordinator, transport messages, and factor-graph backend do not
import one another merely for type definitions.
"""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .interfaces.containers import Pose


@dataclass(frozen=True)
class RangeObservation:
    """One UWB range with its acquisition time on the estimator clock."""

    target_id: int
    distance_cm: float
    event_time_ns: int
    is_anchor: bool
    target_pose: Pose | None = None


@dataclass(frozen=True)
class TimedIMUSample:
    """One complete IMU sample expressed on the estimator clock."""

    timestamp_ns: int
    acceleration: NDArray[np.float64]
    angular_velocity: NDArray[np.float64]

    def __post_init__(self) -> None:
        if not isinstance(self.acceleration, np.ndarray):
            raise TypeError("IMU acceleration must be a NumPy array")
        if self.acceleration.shape != (3,):
            raise ValueError(f"IMU acceleration must have shape (3,), got {self.acceleration.shape}")
        if not isinstance(self.angular_velocity, np.ndarray):
            raise TypeError("IMU angular velocity must be a NumPy array")
        if self.angular_velocity.shape != (3,):
            raise ValueError(
                "IMU angular velocity must have shape (3,), "
                f"got {self.angular_velocity.shape}"
            )
