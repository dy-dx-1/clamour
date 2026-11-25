"""
Ground truth of the simulation (SI units) and the abstract radio between devices.

- Anchors are static; tags follow their Trajectory once motion has started (see start_motion).
- A device is reachable from another if it's within UWB range (3D) with 2D line of sight on the map.
- Ranging: noisy GT distance between reachable devices. For a tag target, also returns the pose that tag last published.
- Broadcasts reach the reachable tags that are listening at that moment (inside a listen() call), like the DW1000.

The World is served to the tag processes by a central server, one thread per client connection, so public methods
are thread-safe. "Now" is the host monotonic clock (time.monotonic(), s), shared by every process on the machine.
"""
import threading
import time
from collections import deque

import numpy as np

from maps import BaseMap
from trajectory import TrajectoryState, build_trajectory

# Independent random streams per tag, seeded with (seed, tag_id, stream)
PATH_STREAM = 0
RANGE_NOISE_STREAM = 1
MIN_ANCHORS_FOR_INIT = 4  # Clamour's estimator only initializes from a batch of >= 4 anchor ranges

def clamour_anchors() -> dict[int, tuple[float, float, float]]:
    """Anchors from clamour's config.py, converted from cm to m."""
    from clamour.interfaces.anchors import Anchors  # Applies clamour's floor-level offsets
    return {anchor_id: tuple(c / 100 for c in position) for anchor_id, position in Anchors().anchors_dict.items()}

class World:
    def __init__(self, world_map: BaseMap, anchors: dict, tag_specs: dict, seed: int, uwb_range: float,
                 uwb_noise: float, max_accel: float, segment_length: tuple[float, float]):
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

    @classmethod
    def from_config(cls, cfg) -> "World":
        """Build the world from the sim config module. Empty cfg.ANCHORS falls back to clamour's anchors."""
        return cls(cfg.MAP, cfg.ANCHORS or clamour_anchors(), cfg.TAGS, cfg.SEED, cfg.UWB_RANGE, cfg.UWB_NOISE,
                   cfg.TAG_MAX_ACCEL, cfg.RANDOM_SEGMENT_LENGTH)

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
