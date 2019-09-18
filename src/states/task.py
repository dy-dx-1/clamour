import random
from multiprocessing import Lock

from numpy import array, atleast_2d
from pypozyx import (POZYX_3D, POZYX_ANCHOR_SEL_AUTO, POZYX_DISCOVERY_ALL_DEVICES,
                     POZYX_POS_ALG_UWB_ONLY, POZYX_SUCCESS, POZYX_FAILURE, Coordinates, DeviceRange,
                     PozyxSerial, DeviceCoordinates, EulerAngles, SingleRegister)

from interfaces import Anchors, Neighborhood, Timing
from interfaces.timing import FRAME_DURATION, TASK_SLOT_DURATION, TASK_START_TIME
from messages import UpdateMessage, UpdateType
from messenger import Messenger
from pozyx_utils import PozyxDiscoverer

from .constants import State
from .tdmaState import TDMAState


class Task(TDMAState):
    def __init__(self, timing: Timing, anchors: Anchors, neighborhood: Neighborhood,
                 id: int, shared_pozyx: PozyxSerial, shared_pozyx_lock: Lock, messenger: Messenger):
        self.timing = timing
        self.anchors = anchors
        self.id = id
        self.localize = self.ranging
        self.pozyx = shared_pozyx
        self.pozyx_lock = shared_pozyx_lock
        self.neighborhood = neighborhood
        self.messenger = messenger
        self.set_manually_measured_anchors()

    def execute(self) -> State:
        self.discover_devices()
        self.neighborhood.collect_garbage()
        self.select_localization_method()
        self.set_manually_measured_anchors()
        self.localize()

        return self.next()

    def next(self) -> State:
        if self.timing.current_time_in_cycle > (TASK_START_TIME + self.timing.frame_id * FRAME_DURATION +
                                                (self.timing.current_slot_id + 1) * TASK_SLOT_DURATION):
            return State.LISTEN
        else:
            return State.TASK

    def select_localization_method(self) -> None:
        self.localize = self.positioning if len(self.anchors.available_anchors) >= 3 else self.ranging

    def positioning(self) -> None:
        position = Coordinates()
        angles = EulerAngles()

        with self.pozyx_lock:
            status_pos = self.pozyx.doPositioning(position, POZYX_3D, algorithm=POZYX_POS_ALG_UWB_ONLY)
            status_angle = self.pozyx.getEulerAngles_deg(angles)

        if status_pos != POZYX_SUCCESS:
            self.handle_error("positioning (pos)")
        if status_angle != POZYX_SUCCESS:
            self.handle_error("positioning (ranging)")

        if status_pos == status_angle == POZYX_SUCCESS and self.positioning_converges(position):
            self.messenger.send_new_measurement(UpdateType.TRILATERATION, position, angles.heading)

    @staticmethod
    def positioning_converges(coordinates: Coordinates) -> bool:
        return not (coordinates.x == coordinates.y == coordinates.z == 0)

    def ranging(self) -> None:
        ranging_target_id = self.select_ranging_target()
        if ranging_target_id not in self.anchors.anchors_dict:
            device_coordinates = Coordinates()
            with self.pozyx_lock:
                self.pozyx.getCoordinates(device_coordinates)

            self.anchors.anchors_dict[ranging_target_id] = DeviceCoordinates(ranging_target_id, 1, device_coordinates)

        device_range = DeviceRange()
        angles = EulerAngles()

        with self.pozyx_lock:
            status_pos = self.pozyx.doRanging(ranging_target_id, device_range) if ranging_target_id > 0 else POZYX_FAILURE
            status_angle = self.pozyx.getEulerAngles_deg(angles)
        
        if status_pos != POZYX_SUCCESS:
            self.handle_error("ranging (pos)")
        if status_angle != POZYX_SUCCESS:
            self.handle_error("ranging (ranging)")

        yaw = angles.heading

        measured_position = Coordinates(device_range.data[1], 0, 0)
        neighbor_position = array([self.anchors.anchors_dict[ranging_target_id][2],
                                   self.anchors.anchors_dict[ranging_target_id][3],
                                   self.anchors.anchors_dict[ranging_target_id][4]])

        if status_pos == status_angle == POZYX_SUCCESS:
            self.messenger.send_new_measurement(UpdateType.RANGING, measured_position, yaw, atleast_2d(neighbor_position))

    def select_ranging_target(self) -> int:
        """We select a target for doing a range measurement.
        Anchors are prioritized because of their lower uncertainty."""

        if len(self.anchors.available_anchors) > 0:
            return random.choice(self.anchors.available_anchors)
        elif len(self.anchors.available_tags) > 0:
            return random.choice(self.anchors.available_tags)
        else:
            return -1

    def discover_devices(self):  # todo @yanjun: This pozys.doDiscover can not be done in every task slot because it is too time consuming. We do it once in all the FRAME_DURATION * NB_FULL_CYCLES.
        """Discovers the devices available for localization/ranging.
        Prioritizes the anchors because of their smaller measurement uncertainty.
        If there aren't enough anchors, will use tags as well."""

        self.anchors.available_anchors.clear()
        with self.pozyx_lock:
            self.pozyx.clearDevices()

        self.discover(POZYX_DISCOVERY_ALL_DEVICES)
        print("Discovered anchors/tags:", self.anchors.available_anchors)

        anchors = [device for device in self.anchors.available_anchors if self.is_anchor(device)]

        if len(anchors) >= 1:
            self.anchors.available_anchors = anchors
            print("Filtered anchors:", self.anchors.available_anchors)
        else:
            self.anchors.available_tags = self.anchors.available_anchors
            print("Filtered tags:", self.anchors.available_tags)
            self.anchors.available_anchors.clear()


    @staticmethod
    def is_anchor(device_id: int) -> bool:
        return device_id < 0x500

    def discover(self, discovery_type: int) -> None:
        devices = PozyxDiscoverer.get_device_list(self.pozyx, self.pozyx_lock, discovery_type)

        for device_id in devices:
            if device_id not in self.anchors.available_anchors:
                self.anchors.available_anchors.append(device_id)

    def set_manually_measured_anchors(self) -> None:
        with self.pozyx_lock:
            self.pozyx.clearDevices()

        for anchor in self.anchors.anchors_dict.values():
            try:
                with self.pozyx_lock:
                    self.pozyx.addDevice(anchor)
            except:
                print(anchor)

        if len(self.anchors.anchors_dict) > 4:
            with self.pozyx_lock:
                self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors.anchors_dict))

    def handle_error(self, function_name: str) -> None:
        error_code = SingleRegister()

        with self.pozyx_lock:
            self.pozyx.getErrorCode(error_code)
            message = self.pozyx.getErrorMessage(error_code)

        if error_code != 0x0:
            print("Error in", function_name, ":", message)
