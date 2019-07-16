import random
from multiprocessing import Lock

from numpy import array, atleast_2d
from pypozyx import (POZYX_3D, POZYX_ANCHOR_SEL_AUTO, POZYX_DISCOVERY_ANCHORS_ONLY, POZYX_DISCOVERY_TAGS_ONLY,
                     POZYX_POS_ALG_UWB_ONLY, POZYX_SUCCESS, Coordinates, DeviceList, DeviceRange,
                     PozyxSerial, SingleRegister, DeviceCoordinates, EulerAngles)

from interfaces import Anchors, Neighborhood, Timing
from interfaces.timing import FRAME_DURATION, TASK_SLOT_DURATION, TASK_START_TIME
from messages import UpdateMessage, UpdateType
from messenger import Messenger

from .constants import State, TAG_ID_MASK
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
        self.broadcast_positioning_result(self.localize())

        return self.next()
        
    def next(self) -> State:
        if self.timing.current_time_in_cycle > (TASK_START_TIME + self.timing.frame_id * FRAME_DURATION +
                                                (self.timing.current_slot_id + 1) * TASK_SLOT_DURATION):
            return State.LISTEN
        else:
            return State.TASK

    def select_localization_method(self) -> None:
        self.localize = self.positioning if len(self.anchors.available_anchors) >= 4 else self.ranging

    def positioning(self) -> int:
        position = Coordinates()
        dimension = POZYX_3D
        height = 1000
        angles = EulerAngles()

        with self.pozyx_lock:
            status = self.pozyx.doPositioning(position, dimension, height, POZYX_POS_ALG_UWB_ONLY)
            status &= self.pozyx.getEulerAngles_deg(angles)
        yaw = angles[0]
        
        scaled_position = Coordinates(position.x/10, position.y/10, position.z/10)

        if status == POZYX_SUCCESS:
            self.messenger.send_new_measurement(UpdateType.TRILATERATION, scaled_position, yaw)

        return status

    def ranging(self) -> int:
        ranging_target_id = self.select_ranging_target()
        if ranging_target_id not in self.anchors.anchors_dict:
            device_coordinates = Coordinates()
            with self.pozyx_lock:
                self.pozyx.getCoordinates(device_coordinates)
            
            self.anchors.anchors_dict[ranging_target_id] = DeviceCoordinates(ranging_target_id, 1, device_coordinates)

        device_range = DeviceRange()
        angles = EulerAngles()

        with self.pozyx_lock:
            status = self.pozyx.doRanging(ranging_target_id, device_range) if ranging_target_id > 0 else None
            status &= self.pozyx.getEulerAngles_deg(angles)
        yaw = angles[0]


        measured_position = Coordinates(device_range.data[1]/10, 0, 0)
        neighbor_position = array([self.anchors.anchors_dict[ranging_target_id][2]/10,
                                   self.anchors.anchors_dict[ranging_target_id][3]/10,
                                   self.anchors.anchors_dict[ranging_target_id][4]/10])
        
        if status == POZYX_SUCCESS:
            self.messenger.send_new_measurement(UpdateType.RANGING, measured_position, yaw, atleast_2d(neighbor_position))

        return status


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

        with self.pozyx_lock:
            self.pozyx.clearDevices()
        
        self.discover(POZYX_DISCOVERY_ANCHORS_ONLY)

        if len(self.anchors.available_anchors) < 3:
            self.discover(POZYX_DISCOVERY_TAGS_ONLY)

    def discover(self, discovery_type: int) -> None:
        with self.pozyx_lock:
            discovery_status = self.pozyx.doDiscovery(discovery_type=discovery_type)
        
        if discovery_status == POZYX_SUCCESS:
            devices = self.get_devices()

            for device_id in devices:
                if device_id not in self.anchors.available_anchors:
                    self.anchors.available_anchors.append(device_id)

    def get_devices(self) -> DeviceList:
        size = SingleRegister()
        with self.pozyx_lock:
            status = self.pozyx.getDeviceListSize(size)
        
        devices = DeviceList(list_size=size[0])

        if status == POZYX_SUCCESS and size[0] > 0:
            with self.pozyx_lock:
                status &= self.pozyx.getDeviceIds(devices)
        else:
            print("No anchors available.")

        return devices

    def set_manually_measured_anchors(self) -> None:
        """If a discovered anchor's coordinates are known (i.e. were manually measured),
        they will be added to the pozyx."""

        with self.pozyx_lock:
            self.pozyx.clearDevices()

        for anchor_id in self.anchors.available_anchors:
            if anchor_id in self.anchors.anchors_dict:
                # For this step, only the anchors (not the tags) must be selected to use their predefined position
                with self.pozyx_lock:
                    self.pozyx.addDevice(self.anchors.anchors_dict[anchor_id])
            else:
                device_coordinates = Coordinates()
                with self.pozyx_lock:
                    self.pozyx.getCoordinates(device_coordinates)
                    self.pozyx.addDevice(DeviceCoordinates(anchor_id, 1, device_coordinates))
        
        if len(self.anchors.available_anchors) > 4:
            with self.pozyx_lock:
                self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors.available_anchors))

    def broadcast_positioning_result(self, positioning_result) -> None:
        """Commented function because broadcast via socket deactivated anyway,
        and possible version incompatibility issues"""
        pass
        # message_data = [self.id & TAG_ID_MASK] + self.extended_kalman_filter.x.toList() + \
        #                [self.localize, self.extended_kalman_filter.dt] + \
        #                 self.last_measurement + self.last_measurement_data.flatten().tolist() + \
        #                [self.acceleration.x/10, self.acceleration.y/10, self.acceleration.z/10,
        #                 self.timing.frame_id, self.timing.current_slot_id, positioning_result]
        #
        # message = struct.pack("%sf" % len(message_data), *message_data)
        # self.socket.send(message)
