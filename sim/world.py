"""
Ground truth of the simulation (SI units) and the abstract radio between devices.

- Anchors are static; tags follow their Trajectory once motion has started (see start_motion).
- A device is reachable from another if it's within UWB range (3D) with 2D line of sight on the map.
- Ranging: noisy GT distance between reachable devices. For a tag target, also returns the pose that tag last published.
- Broadcasts reach the reachable tags that are listening at that moment (inside a listen() call), like the DW1000.
- IMU: idealized body-frame samples (x along the heading, z up) with constant biases and white noise.

The World is served to the tag processes by a central server, one thread per client connection, so public methods
are thread-safe. "Now" is the host monotonic clock (time.monotonic(), s), shared by every process on the machine.
"""
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from maps import BaseMap
from trajectory import TrajectoryState, build_trajectory

# Independent random streams per tag, seeded with (seed, tag_id, stream)
PATH_STREAM = 0
RANGE_NOISE_STREAM = 1
IMU_STREAM = 2
MIN_ANCHORS_FOR_INIT = 4  # Clamour's estimator only initializes from a batch of >= 4 anchor ranges
MAX_START_HEADING_RAD = np.deg2rad(2)  # Clamour's FG assumes tags start facing +x, with a 2 deg prior (see sim_design.md)
GRAVITY = 9.80665         # m/s^2
MG = GRAVITY / 1000       # m/s^2 per mg

@dataclass(frozen=True)
class IMUModel:
    """Idealized IMU shared by every tag; each tag draws its own biases from it."""
    sample_rate_hz: float
    accel_noise_density: float  # m/s^2/sqrt(Hz)
    gyro_noise_density: float   # rad/s/sqrt(Hz)
    accel_bias_mean: np.ndarray # (3,) m/s^2
    accel_bias_std: np.ndarray  # (3,) m/s^2
    gyro_bias_std: float        # rad/s
    clock_rate_error: float     # Relative rate error of the sample/timestamp clock

@dataclass
class _TagIMU:
    rng: np.random.Generator    # Used by the tag's estimator process only, so by a single server thread
    accel_bias: np.ndarray      # (3,) m/s^2
    gyro_bias: np.ndarray       # (3,) rad/s
    start_tick: int             # Timestamp counter value when the IMU is created (32-bit counter)

def clamour_anchors() -> dict[int, tuple[float, float, float]]:
    """Anchors from clamour's config.py, converted from cm to m."""
    from clamour.interfaces.anchors import Anchors  # Applies clamour's floor-level offsets
    return {anchor_id: tuple(c / 100 for c in position) for anchor_id, position in Anchors().anchors_dict.items()}

class World:
    def __init__(self, world_map: BaseMap, anchors: dict, tag_specs: dict, seed: int, uwb_range: float,
                 uwb_noise: float, max_accel: float, segment_length: tuple[float, float], imu: IMUModel):
        self.map = world_map
        self.anchors = {anchor_id: np.asarray(position, dtype=float) for anchor_id, position in anchors.items()}
        for anchor_id, position in self.anchors.items():
            if not world_map.is_free(*position[:2]):
                raise ValueError(f"Anchor {anchor_id} at {tuple(position)} is not in the map's free space")
        self.trajectories = {tag_id: build_trajectory(spec, max_accel, segment_length, world_map,
                                                      np.random.default_rng([seed, tag_id, PATH_STREAM]))
                             for tag_id, spec in tag_specs.items()}
        # Only the tag's TDMA process ranges, so each generator is used by a single server thread
        self._range_noise = {tag_id: np.random.default_rng([seed, tag_id, RANGE_NOISE_STREAM]) for tag_id in tag_specs}
        self.uwb_range = uwb_range
        self.uwb_noise = uwb_noise
        self.imu = imu
        self._imus = {}
        for tag_id in tag_specs:
            rng = np.random.default_rng([seed, tag_id, IMU_STREAM])
            self._imus[tag_id] = _TagIMU(rng=rng,
                                         accel_bias=rng.normal(imu.accel_bias_mean, imu.accel_bias_std),
                                         gyro_bias=rng.normal(0.0, imu.gyro_bias_std, 3),
                                         start_tick=int(rng.integers(0, 2**32)))

        self._motion_start: float | None = None
        self._published_poses = {}                               # {tag_id: last pose the tag published}
        self._inboxes = {tag_id: None for tag_id in tag_specs}   # {tag_id: deque while listening, None otherwise}
        self._message_arrived = threading.Condition()

        now = self.now()
        for tag_id in self.trajectories:
            visible = sum(anchor_id in self.anchors for anchor_id in self.reachable_devices(tag_id, now))
            if visible < MIN_ANCHORS_FOR_INIT:
                print(f"WARNING: tag {tag_id} reaches only {visible} anchors from its start position. "
                      f"Its estimator won't initialize (needs {MIN_ANCHORS_FOR_INIT}).")
            start_heading = self.trajectories[tag_id].state(0.0).heading
            if abs(start_heading) > MAX_START_HEADING_RAD:
                print(f"WARNING: tag {tag_id} starts with heading {np.rad2deg(start_heading):.1f} deg. Clamour assumes "
                      f"tags start facing +x (yaw prior 0 +/- 2 deg), so its IMU integration will be off.")

    @classmethod
    def from_config(cls, cfg) -> "World":
        """
        Build the world from the sim config module. Empty cfg.ANCHORS falls back to clamour's anchors.
        IMU accel biases are drawn from clamour's own prior, so the FG's bias prior is statistically right.
        """
        from clamour.config import IMU_ACCEL_INITIAL_BIAS_MG, IMU_ACCEL_INITIAL_BIAS_STD_MG
        imu = IMUModel(cfg.IMU_SAMPLE_RATE_HZ, cfg.IMU_ACCEL_NOISE_DENSITY, cfg.IMU_GYRO_NOISE_DENSITY,
                       IMU_ACCEL_INITIAL_BIAS_MG * MG, IMU_ACCEL_INITIAL_BIAS_STD_MG * MG, cfg.IMU_GYRO_BIAS_STD,
                       cfg.IMU_CLOCK_RATE_ERROR)
        return cls(cfg.MAP, cfg.ANCHORS or clamour_anchors(), cfg.TAGS, cfg.SEED, cfg.UWB_RANGE, cfg.UWB_NOISE,
                   cfg.TAG_MAX_ACCEL, cfg.RANDOM_SEGMENT_LENGTH, imu)

    @staticmethod
    def now() -> float:
        return time.monotonic()

    ### -------------------------------------------- GROUND TRUTH --------------------------------------------
    def start_motion(self, delay: float = 0.0) -> None:
        """Make every tag start its trajectory after delay seconds. Later calls are ignored."""
        with self._message_arrived:
            if self._motion_start is None:
                self._motion_start = self.now() + delay

    def motion_start(self) -> float | None:
        """Monotonic time at which motion starts, or None if not scheduled yet."""
        return self._motion_start

    def gt_state(self, tag_id: int, t: float) -> TrajectoryState:
        """Ground truth of a tag at monotonic time t (still at its start until motion starts)."""
        start = self._motion_start
        return self.trajectories[tag_id].state(t - start if start is not None else 0.0)

    def position(self, device_id: int, t: float) -> np.ndarray:
        """3D position (m) of an anchor or tag at monotonic time t."""
        anchor = self.anchors.get(device_id)
        return anchor if anchor is not None else self.gt_state(device_id, t).position

    ### -------------------------------------------- ABSTRACT RADIO --------------------------------------------
    def is_device(self, device_id: int) -> bool:
        return device_id in self.anchors or device_id in self.trajectories

    def reachable(self, device_a: int, device_b: int, t: float) -> bool:
        """Within UWB range (3D distance) with 2D line of sight on the map."""
        a, b = self.position(device_a, t), self.position(device_b, t)
        return np.linalg.norm(a - b) <= self.uwb_range and self.map.line_of_sight(a[:2], b[:2])

    def reachable_devices(self, tag_id: int, t: float | None = None) -> set[int]:
        """IDs of every anchor and other tag reachable from tag_id at time t (default: now)."""
        t = self.now() if t is None else t
        return {device_id for device_id in (*self.anchors, *self.trajectories)
                if device_id != tag_id and self.reachable(tag_id, device_id, t)}

    def range(self, requester_id: int, target_id: int):
        """
        Noisy distance (m) from requester to target, now. (None, None) if the target is unknown or unreachable.
        For a tag target, also returns its last published pose (None if it hasn't published yet).
        """
        t = self.now()
        if not self.is_device(target_id) or target_id == requester_id or not self.reachable(requester_id, target_id, t):
            return None, None
        true_distance = np.linalg.norm(self.position(requester_id, t) - self.position(target_id, t))
        distance = true_distance + self._range_noise[requester_id].normal(0.0, self.uwb_noise)
        if distance <= 0:
            return None, None
        return float(distance), self._published_poses.get(target_id)

    def publish_pose(self, tag_id: int, pose) -> None:
        """Pose a tag shares with tags ranging to it. Opaque to the world (SimulationTag decides its format)."""
        self._published_poses[tag_id] = pose

    def broadcast(self, sender_id: int, payload: bytes) -> None:
        """Deliver payload to every reachable tag that is listening right now. Others miss it."""
        t = self.now()
        with self._message_arrived:
            listening = [tag_id for tag_id, inbox in self._inboxes.items() if inbox is not None and tag_id != sender_id]
        receivers = [tag_id for tag_id in listening if self.reachable(sender_id, tag_id, t)]
        with self._message_arrived:
            for tag_id in receivers:
                inbox = self._inboxes[tag_id]
                if inbox is not None:  # May have stopped listening meanwhile
                    inbox.append((sender_id, payload))
            self._message_arrived.notify_all()

    def listen(self, tag_id: int, timeout: float) -> tuple[int, bytes] | None:
        """
        Listen for up to timeout seconds, like a DW1000 listen. Returns the first (sender_id, payload) heard, or None.
        Like the DW1000, one listen yields one frame: anything else heard in the window is dropped.
        """
        with self._message_arrived:
            inbox = self._inboxes[tag_id] = deque()
            self._message_arrived.wait_for(lambda: inbox, timeout)
            self._inboxes[tag_id] = None
        return inbox[0] if inbox else None

    ### -------------------------------------------- IMU --------------------------------------------
    def imu_spec(self, tag_id: int) -> dict:
        """Constants a tag's simulated IMU needs (SI units): sample rate, noise densities, clock rate error, start tick."""
        return {'sample_rate_hz': self.imu.sample_rate_hz,
                'accel_noise_density': self.imu.accel_noise_density,
                'gyro_noise_density': self.imu.gyro_noise_density,
                'clock_rate_error': self.imu.clock_rate_error,
                'start_tick': self._imus[tag_id].start_tick}

    def imu_samples(self, tag_id: int, times) -> tuple[np.ndarray, np.ndarray]:
        """
        IMU samples of a tag at the given monotonic times, in the body frame (x along the heading, z up):
        - accel (N, 3) in m/s^2: specific force, so +g on z at rest, + bias + white noise
        - gyro (N, 3) in rad/s: yaw rate on z, + bias + white noise
        """
        if len(times) == 0:
            return np.empty((0, 3)), np.empty((0, 3))
        imu = self._imus[tag_id]
        states = [self.gt_state(tag_id, t) for t in times]
        heading = np.array([state.heading for state in states])
        force = np.array([state.acceleration for state in states]) + [0.0, 0.0, GRAVITY]  # World frame
        cos, sin = np.cos(heading), np.sin(heading)
        accel = np.column_stack((cos * force[:, 0] + sin * force[:, 1], -sin * force[:, 0] + cos * force[:, 1], force[:, 2]))
        gyro = np.zeros((len(times), 3))
        gyro[:, 2] = [state.yaw_rate for state in states]
        # White noise density -> per-sample std: density * sqrt(sample rate)
        root_rate = np.sqrt(self.imu.sample_rate_hz)
        accel += imu.accel_bias + imu.rng.normal(0.0, self.imu.accel_noise_density * root_rate, accel.shape)
        gyro += imu.gyro_bias + imu.rng.normal(0.0, self.imu.gyro_noise_density * root_rate, gyro.shape)
        return accel, gyro

    def imu_attitude(self, tag_id: int, t: float) -> float:
        """Yaw (rad) of a tag at monotonic time t, relative to its start heading (like a game rotation vector)."""
        yaw = self.gt_state(tag_id, t).heading - self.trajectories[tag_id].state(0.0).heading
        return float(np.arctan2(np.sin(yaw), np.cos(yaw)))
