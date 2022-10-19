import random
from multiprocessing import Lock
from struct import error as StructError
from time import perf_counter

from numpy import array, atleast_2d
from pypozyx import (POZYX_3D, POZYX_ANCHOR_SEL_AUTO, POZYX_DISCOVERY_ALL_DEVICES,
                     POZYX_POS_ALG_UWB_ONLY, POZYX_SUCCESS, Coordinates, DeviceRange,
                     PozyxSerial, EulerAngles, SingleRegister, Data)

from interfaces import Anchors, Neighborhood, Timing, SlotAssignment
from messages import UpdateMessage, UpdateType
from messenger import Messenger
from pozyx_utils import PozyxDiscoverer

from .constants import State
from .tdmaState import TDMAState


class Task(TDMAState):
    def __init__(self, timing: Timing, anchors: Anchors, neighborhood: Neighborhood,
                 id: int, shared_pozyx: PozyxSerial, shared_pozyx_lock: Lock, messenger: Messenger,
                 slot_assignment: SlotAssignment):
        self.timing = timing
        self.anchors = anchors
        self.id = id
        self.localize = self.ranging
        self.pozyx = shared_pozyx
        self.pozyx_lock = shared_pozyx_lock
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.messenger = messenger
        self.set_manually_measured_anchors()
        self.frame_id_done_discover = -1
        self.neighborUpdateFrequency = 5 # every five frames, do discovery and update neighbor information

    def execute(self) -> State:
        if self.frame_id_done_discover != self.timing.frame_id and not self.timing.frame_id % self.neighborUpdateFrequency: # do discovery at first slot of every $neighborUpdateFrequency frames
            self.frame_id_done_discover = self.timing.frame_id
            self.discover_devices()
            self.neighborhood.collect_garbage()
            self.select_localization_method()
            self.set_manually_measured_anchors()

        if self.timing.enough_time_left():
            self.localize()
            #self.testTDMA()

        if self.neighborhood.changed:
            self.messenger.broadcast_topology_message()  # Broadcast topology change to other devices
            self.messenger.send_topology_update(self.timing.logical_clock.clock, self.timing.logical_clock.offset, self.neighborhood.current_neighbors)
            self.neighborhood.changed = False

        return self.next()

    def testTDMA(self):
        tosend = [255] * 8
        temp = self.slot_assignment.pure_send_list.copy()
        for ele in temp:
            if ele<0:
                temp.remove(ele)
        for i in range(min(len(temp), 8)):
            tosend[i] = temp[i]
        if self.timing.current_slot_id == 0:
            tosend[-1] = (0 if tosend[-1]==255 else tosend[-1])
            #print(self.id, " b2 at slot", 0)
            with self.pozyx_lock:
                self.pozyx.sendData(destination=0, data=Data(tosend, 'BBBBBBBBB'))
        else:
            #print(self.id, " b2 at slot", self.timing.current_slot_id-1)
            tosend[-1] = (self.timing.current_slot_id-1 if tosend[-1]==255 else tosend[-1])
            print(tosend)
            with self.pozyx_lock:
                self.pozyx.sendData(destination=0, data=Data(tosend, 'BBBBBBBBB'))
        print(self.timing.frame_id, self.timing.current_slot_id, self.timing.get_full_cycle_duration(),self.timing.current_time_in_cycle)

    def next(self) -> State:
        if self.timing.in_cycle():
            return State.TASK if self.timing.in_taskslot(self.slot_assignment.pure_send_list) else State.LISTEN
        else:
            print("Go to sync")
            return State.SYNCHRONIZATION

    def select_localization_method(self) -> None:
        self.localize = self.positioning if len(self.anchors.available_anchors) >= 3 else self.ranging

    def positioning(self) -> None:
        position = Coordinates()
        angles = EulerAngles()

        try:
            with self.pozyx_lock:
                print("USing anchors: ", self.anchors.anchors_dict)
                print("USing anchors: ", self.anchors.anchors_list)
                status_pos = self.pozyx.doPositioning(position, POZYX_3D, algorithm=POZYX_POS_ALG_UWB_ONLY)
                status_angle = self.pozyx.getEulerAngles_deg(angles)
        except StructError as s:
            status_pos, status_angle = 0, 0
            print(str(s))

        if status_pos != POZYX_SUCCESS:
            self.handle_error("positioning (pos)")
        if status_angle != POZYX_SUCCESS:
            self.handle_error("positioning (ranging)")

        if status_pos == status_angle == POZYX_SUCCESS and self.positioning_converges(position):
            self.messenger.send_ekf_update(UpdateType.TRILATERATION, self.timing.logical_clock.clock, self.timing.logical_clock.offset,
                                           position, angles.heading, topology=self.neighborhood.current_neighbors)

    @staticmethod
    def positioning_converges(coordinates: Coordinates) -> bool:
        return not (coordinates.x == coordinates.y == coordinates.z == 0.0)

    def ranging(self) -> None:
        ranging_target_id = self.select_ranging_target()

        if ranging_target_id is not None:
            ref_coordinates = Coordinates()

            if ranging_target_id not in self.anchors.anchors_dict:
                try:
                    with self.pozyx_lock:
                        self.pozyx.getCoordinates(ref_coordinates)
                except StructError as s:
                    print(str(s))
            else:
                ref_coordinates = self.anchors.anchors_dict[ranging_target_id].pos

            measured_position = DeviceRange()
            angles = EulerAngles()

            try:
                with self.pozyx_lock:
                    status_pos = self.pozyx.doRanging(ranging_target_id, measured_position)
                    status_angle = self.pozyx.getEulerAngles_deg(angles)
            except StructError as s:
                status_angle, status_pos = 0, 0
                print(s)

            if status_pos == POZYX_SUCCESS:
                measured_position = Coordinates(measured_position.data[1], 0, 0)

            neighbor_position = array([ref_coordinates.x, ref_coordinates.y, ref_coordinates.z])

            if status_pos == status_angle == POZYX_SUCCESS:
                self.messenger.send_ekf_update(UpdateType.RANGING, self.timing.logical_clock.clock, self.timing.logical_clock.offset,
                                               measured_position, angles.heading, neighbors=atleast_2d(neighbor_position),
                                               topology=self.neighborhood.current_neighbors)

    def select_ranging_target(self) -> int:
        """We select a target for doing a range measurement.
        Anchors are prioritized because of their lower uncertainty."""

        if len(self.anchors.available_anchors) > 0:
            return random.choice(self.anchors.available_anchors)

    def discover_devices(self):
        """Discovers the devices available for localization/ranging.
        Prioritizes the anchors because of their smaller measurement uncertainty.
        If there aren't enough anchors, will use tags as well."""

        self.anchors.available_anchors.clear()

        with self.pozyx_lock:
            self.pozyx.clearDevices()

        self.discover(POZYX_DISCOVERY_ALL_DEVICES)

        new_anchors, new_tags = [], []
        for device in self.anchors.available_anchors:
            print("Found device: ", device)
            if PozyxDiscoverer.is_anchor(device):
                print("It's an anchor!")
                new_anchors.append(device)
            else:
                print("It's a tag!")
                new_tags.append(device)

        self.anchors.available_anchors = new_anchors
        self.update_neighborhood(new_tags)

    def update_neighborhood(self, new_tags: list) -> None:
        if set(new_tags) != set(self.neighborhood.current_neighbors.keys()):
            self.neighborhood.current_neighbors.clear()
            for tag in new_tags:
                self.neighborhood.add_neighbor(tag, perf_counter(), State.TASK)
                self.neighborhood.changed = True

    def discover(self, discovery_type: int) -> None:
        devices = PozyxDiscoverer.get_device_list(self.pozyx, self.pozyx_lock, discovery_type)

        for device_id in devices:
            if device_id not in self.anchors.available_anchors:
                self.anchors.available_anchors.append(device_id)

    def set_manually_measured_anchors(self) -> None:
        with self.pozyx_lock:
            self.pozyx.clearDevices()

        for anchor in self.anchors.available_anchors:
            print("Creating pozyx anchor list: ", anchor)
            print("From loaded anchor: ",  self.anchors.anchors_dict)
            print(self.anchors.anchors_list)
            if anchor in self.anchors.anchors_dict:
                with self.pozyx_lock:
                    self.pozyx.addDevice(self.anchors.anchors_dict[anchor])

        if len(self.anchors.available_anchors) > 3:
            with self.pozyx_lock:
                self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors.available_anchors))

    def handle_error(self, function_name: str) -> None:
        error_code = SingleRegister()

        try:
            with self.pozyx_lock:
                self.pozyx.getErrorCode(error_code)
                message = self.pozyx.getErrorMessage(error_code)
        except StructError as s:
            message = ""
            print(str(s))

        if error_code != 0x0:
            print("Error in", function_name, ":", message)
