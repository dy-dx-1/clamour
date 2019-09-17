import random
from multiprocessing import Lock

from numpy import array, atleast_2d
from pypozyx import (POZYX_3D, POZYX_ANCHOR_SEL_AUTO, POZYX_DISCOVERY_ANCHORS_ONLY, POZYX_DISCOVERY_TAGS_ONLY,
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

    def positioning(self) -> int:
        position = Coordinates()
        dimension = POZYX_3D
        height = 1000
        angles = EulerAngles()

        print("Anchors/tags for positioning:", self.anchors.available_anchors)
        with self.pozyx_lock:
            status_pos = self.pozyx.doPositioning(position, dimension, height, POZYX_POS_ALG_UWB_ONLY)
            status_angle = self.pozyx.getEulerAngles_deg(angles)

        if status_pos != POZYX_SUCCESS:
            self.handle_error("positioning (pos)")
        if status_angle != POZYX_SUCCESS:
            self.handle_error("positioning (ranging)")

        yaw = angles.heading

        position = Coordinates(position.x, position.y, position.z)

        if status_pos == status_angle == POZYX_SUCCESS:
            self.messenger.send_new_measurement(UpdateType.TRILATERATION, position, yaw)
        
        return status_pos and status_angle

    def ranging(self) -> int:
        ranging_target_id = self.select_ranging_target()
        if ranging_target_id not in self.anchors.anchors_dict:
            device_coordinates = Coordinates()
            with self.pozyx_lock:
                self.pozyx.getCoordinates(device_coordinates)

            self.anchors.anchors_dict[ranging_target_id] = DeviceCoordinates(ranging_target_id, 1, device_coordinates)

        device_range = DeviceRange()
        angles = EulerAngles()

        print("Anchors/tags for ranging:", self.anchors.available_anchors)
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
        # print("Ranging status", status_pos, status_angle,
        #       "(nb available anchors:", len(self.anchors.available_anchors), ")")
        if status_pos == status_angle == POZYX_SUCCESS:
            self.messenger.send_new_measurement(UpdateType.RANGING, measured_position, yaw, atleast_2d(neighbor_position))

        return status_pos and status_angle

    def select_ranging_target(self) -> int:
        """We select a target for doing a range measurement.
        Anchors are prioritized because of their lower uncertainty."""

        if len(self.anchors.available_anchors) > 0:
            return random.choice(self.anchors.available_anchors)
        elif len(self.neighborhood.current_neighbors) > 0:
            return random.choice(list(self.neighborhood.current_neighbors))
        else:
            return -1

    def discover_devices(self):  # todo @yanjun: This pozys.doDiscover can not be done in every task slot because it is too time consuming. We do it once in all the FRAME_DURATION * NB_FULL_CYCLES.
        """Discovers the devices available for localization/ranging.
        Prioritizes the anchors because of their smaller measurement uncertainty.
        If there aren't enough anchors, will use tags as well."""

        self.anchors.available_anchors.clear()
        with self.pozyx_lock:
            self.pozyx.clearDevices()

        self.discover(POZYX_DISCOVERY_ANCHORS_ONLY)
        print("Discovered anchors:", self.anchors.available_anchors)

        if len(self.anchors.available_anchors) < 3:
            self.discover(POZYX_DISCOVERY_TAGS_ONLY)
            print("Tags discovered:", self.anchors.available_anchors)

    def discover(self, discovery_type: int) -> None:
        devices = PozyxDiscoverer.get_device_list(self.pozyx, self.pozyx_lock, discovery_type)

        for device_id in devices:
            if device_id not in self.anchors.available_anchors:
                self.anchors.available_anchors.append(device_id)

    def get_coordinates(self, device_id: int) -> DeviceCoordinates:
        device_coordinates = Coordinates()
        self.pozyx.getCoordinates(device_coordinates, device_id)
        return DeviceCoordinates(device_id, 0, device_coordinates)

    def set_manually_measured_anchors(self) -> None:
        """If a discovered anchor's coordinates are known (i.e. were manually measured),
        they will be added to the pozyx."""

        anchors_to_configure = [self.anchors.anchors_dict[anchor_id] for anchor_id in self.anchors.available_anchors
                                if anchor_id in self.anchors.available_anchors]
        print("Anchors to configure:", anchors_to_configure)

        if len(anchors_to_configure) < 3:
            tags_to_configure = [self.get_coordinates(anchor_id) for anchor_id in self.anchors.available_anchors
                                 if anchor_id not in self.anchors.available_anchors]

            anchors_to_configure.append(tags_to_configure)

            print("Tags to configure:", tags_to_configure)

        with self.pozyx_lock:
            self.pozyx.configureAnchors(anchors_to_configure)

        # for anchor_id in self.anchors.available_anchors:
        #     if anchor_id in self.anchors.anchors_dict:
        #         # For this step, only the anchors (not the tags) must be selected to use their predefined position
        #         with self.pozyx_lock:
        #             status = self.pozyx.configureAnchors([self.anchors.anchors_dict[anchor_id]])
        #             # status = self.pozyx.addDevice(self.anchors.anchors_dict[anchor_id])
        #         if status != POZYX_SUCCESS:
        #             self.handle_error("set_manually_measured_anchors (anchors)")
        #     else:
        #         device_coordinates = Coordinates()
        #         with self.pozyx_lock:
        #             status_coord = self.pozyx.getCoordinates(device_coordinates, anchor_id)
        #             status_device = self.pozyx.addDevice(DeviceCoordinates(anchor_id, 1, device_coordinates))
        #         if status_coord != POZYX_SUCCESS:
        #             self.handle_error("set_manually_measured_anchors (tags_coord)")
        #         if status_device != POZYX_SUCCESS:
        #             self.handle_error("set_manually_measured_anchors (tags_device)")

        # if len(self.anchors.available_anchors) > 4:
        #     with self.pozyx_lock:
        #         self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors.available_anchors))

    def handle_error(self, function_name: str) -> None:
        error_code = SingleRegister()

        with self.pozyx_lock:
            self.pozyx.getErrorCode(error_code)
            message = self.pozyx.getErrorMessage(error_code)

        if error_code != 0x0:
            print("Error in", function_name, ":", message)
