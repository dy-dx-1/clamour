"""
UWB tag interface for the simulation module's World (see sim/sim_design.md) instead of hardware.

Behaves like BitcrazeTag from clamour's point of view:
- Anchors (IDs <= 10) are discovered by get_device_list() and cached until clear_anchors().
- Neighbouring tags are only known from messages heard (add_neighbor), and forgotten after DISCOVERY_TIMEOUT.
- Ranging to a tag also returns the pose that tag last published, in Bitcraze's TWR report layout
  (position + covariance, no angles; None if the tag had no covariance yet).
- The pose lives in a SharedPose and is published to the World from receive_data() (TDMA process), just like a
  Bitcraze tag answers a ranging request with its pose from inside receive_data().
- Calls take about as long as on hardware (emulated UWB timing below), because TDMA slot logic depends on it.

The World works in SI units; this class converts to clamour's integer cm.
"""
import os
import time
from multiprocessing.managers import BaseManager
from typing import Literal

from ..custom_terminal import print
from .tag import Tag
from .containers import Pose, SharedPose
from .anchors import Anchors

ALL_ANCHORS = Anchors().anchors_dict  # The anchors this tag knows of and polls during discovery
WORLD_TYPEID = 'world'                # Must match sim/world_server.py
DISCOVERY_TIMEOUT = 15                # s, same as BitcrazeTag: neighbours silent for longer are dropped

# Emulated UWB timing in seconds. TODO: calibrate against measured Bitcraze timings
DW_LISTEN_TIMEOUT = 0.015
DW_PAUSE_DELAY = 0.005                         # BitcrazeTag's pause after each anchor poll
TWR_SUCCESS_DURATION = 0.004
TWR_FAILURE_DURATION = 2 * DW_LISTEN_TIMEOUT   # Waits for ANSWER and REPORT both time out
POLL_ANSWER_DURATION = 0.004                   # Anchor discovery poll that gets an answer

class _WorldClient(BaseManager):
    pass

_WorldClient.register(WORLD_TYPEID)

def connect_world(world_address):
    """Proxy to the World server (sim/world_server.py). Connect in the process that uses it: proxies don't survive a fork."""
    client = _WorldClient(address=world_address)
    client.connect()
    return getattr(client, WORLD_TYPEID)()

def _sleep_until(deadline: float) -> None:
    remaining = deadline - time.perf_counter()
    if remaining > 0:
        time.sleep(remaining)

class SimulationTag(Tag):
    """
    ARGS:
        - tag_id: this tag's ID, as known by the World
        - world_address: address of the World server (sim/world_server.py)
    """
    def __init__(self, tag_id: int, world_address):
        self._id = tag_id
        self._world_address = world_address
        self._world = None              # World proxy, connected once per process (see world)
        self._world_pid = None
        self._active_tags = dict()      # keeps track of nearby tags and when they were last seen
        self._available_anchors = set() # keeps track of currently in-range anchors
        self._pose = SharedPose()       # Tag position, orientation, and covariance, shared by all processes

        print(f"SIMULATION TAG #{self._id} STARTED, WORLD AT {world_address}", 'ok', 'device')

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("SimulationTag closed!", 'ok', 'device')

    @property
    def world(self):
        """Proxy to the World. Connects on first use in each process, since clamour forks after building the tag."""
        if self._world_pid != os.getpid():
            self._world = connect_world(self._world_address)
            self._world_pid = os.getpid()
        return self._world

    @property
    def tag_id(self) -> int:
        return self._id

    @property
    def active_tags(self) -> set[int]:
        now = time.perf_counter()
        return {tag_id for tag_id, last_seen in self._active_tags.items() if (now - last_seen) < DISCOVERY_TIMEOUT}

    @property
    def available_anchors(self) -> set[int]:
        return self._available_anchors

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id: int) -> bool:
        return device_id <= 10

    def add_neighbor(self, tag_id: int) -> None:
        self._active_tags[tag_id] = time.perf_counter()

    def clear_anchors(self) -> None:
        self._available_anchors.clear()

    def reset(self) -> None:
        pass

    def get_device_list(self, discovery_type: Literal["all", "anchor", "tag"]) -> set[int]:
        devices = set()
        if discovery_type == "all" or discovery_type == "anchor":
            start = time.perf_counter()
            reachable = {anchor_id for anchor_id in self.world.reachable_devices(self._id) if anchor_id in ALL_ANCHORS}
            self._available_anchors |= reachable
            devices |= reachable
            # Each known anchor is polled in turn: an answer or a listen timeout, then the DW pause
            _sleep_until(start + len(reachable) * POLL_ANSWER_DURATION
                         + (len(ALL_ANCHORS) - len(reachable)) * DW_LISTEN_TIMEOUT + len(ALL_ANCHORS) * DW_PAUSE_DELAY)
        if discovery_type == "all" or discovery_type == "tag":
            # Like Bitcraze, tags are discovered from the messages they send (see add_neighbor)
            devices |= self.active_tags
        return devices

    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------
    def broadcast(self, payload: list) -> bool:
        self.world.broadcast(self._id, bytes(payload))
        return True

    def receive_data(self) -> tuple[int, bytes]:
        # Publishing here, from the TDMA process, mirrors a Bitcraze tag answering ranging with its pose in receive_data()
        self.world.publish_pose(self._id, self._pose_report())
        frame = self.world.listen(self._id, DW_LISTEN_TIMEOUT)
        return frame if frame is not None else (None, b'')

    ### -------------------------------------------- LOCALIZATION --------------------------------------------
    @property
    def pose(self) -> Pose:
        return self._pose.read()

    @pose.setter
    def pose(self, new_pose: Pose) -> None:
        self._pose.write(new_pose)

    @property
    def orientation(self) -> list:
        return self.pose.angles

    def ranging(self, target_id: int) -> tuple[int | None, Pose | None]:
        start = time.perf_counter()
        distance_m, report = self.world.range(self._id, target_id)
        _sleep_until(start + (TWR_SUCCESS_DURATION if distance_m is not None else TWR_FAILURE_DURATION))
        if distance_m is None:
            return None, None
        distance_cm = round(distance_m * 100)
        if distance_cm <= 0:
            return None, None
        return distance_cm, (self._pose_from_report(report) if report is not None else None)

    def _pose_report(self) -> tuple[int, ...]:
        """This tag's pose in Bitcraze's TWR report layout: x, y, z, xx, yy, zz, xy, xz, yz (zeros if no covariance)."""
        pose = self.pose
        c = pose.covar
        covar = (0,) * 6 if c is None else (c[0][0], c[1][1], c[2][2], c[0][1], c[0][2], c[1][2])
        return tuple(int(value) for value in (*pose.coordinates, *covar))

    @staticmethod
    def _pose_from_report(report: tuple[int, ...]) -> Pose | None:
        """Same as BitcrazeTag.extract_report_neighbor_info: xx == 0 means the tag had no covariance to share."""
        x, y, z, xx, yy, zz, xy, xz, yz = report
        if xx == 0:
            return None
        return Pose(x, y, z, covar=(xx, yy, zz, xy, xz, yz))
