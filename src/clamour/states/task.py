import random
from multiprocessing.synchronize import Lock # multiprocessing.Lock
from time import perf_counter

from numpy import array, atleast_2d
import struct

from ..custom_terminal import print 
from ..interfaces import Tag, Coordinates, Anchors, Neighborhood, Timing, SlotAssignment
from ..messages import UpdateMessage, UpdateType
from ..messenger import Messenger

from .constants import State
from .tdmaState import TDMAState


class Task(TDMAState):
    def __init__(self, timing: Timing, anchors: Anchors, neighborhood: Neighborhood,
                 id: int, shared_tag: Tag, shared_tag_lock: Lock, messenger: Messenger,
                 slot_assignment: SlotAssignment):
        self.timing = timing
        self.anchors = anchors
        self.id = id
        self.localize = self.ranging
        self.tag = shared_tag
        self.tag_lock = shared_tag_lock
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
            with self.tag_lock:
                self.tag.sendData(destination=0, payload=struct.pack('<BBBBBBBBB', *tosend))
        else:
            tosend[-1] = (self.timing.current_slot_id-1 if tosend[-1]==255 else tosend[-1])
            print(f"Task.testTDMA(): tosend: {tosend}", 'info', 'tdma')
            with self.tag_lock:
                self.tag.sendData(destination=0, payload=struct.pack('<BBBBBBBBB', *tosend))
        print(f"Task.testTDMA(): {self.timing.frame_id} {self.timing.current_slot_id} {self.timing.get_full_cycle_duration()} {self.timing.current_time_in_cycle}", 'info', 'tdma')

    def next(self) -> State:
        if self.timing.in_cycle():
            return State.TASK if self.timing.in_taskslot(self.slot_assignment.pure_send_list) else State.LISTEN
        else:
            print("Task.next(): Moving to SYNC state", 'info', 'tdma')
            return State.SYNCHRONIZATION

    def select_localization_method(self) -> None:
        self.localize = self.positioning if len(self.tag.available_anchors) >= 3 else self.ranging

    def positioning(self) -> None:
        with self.tag_lock:
            print(f"Task.positioning(): Attempting positioning with anchors: {self.anchors.anchors_dict}", 'info', 'loc')
            position = self.tag.doPositioning()
            angles = self.tag.getOrientation() 

        if (position is not None) and (angles is not None): 
            print(f"Task.positioning(): Successful positioning", 'ok', 'loc')
        else: 
            if position is None: 
                self.handle_error("Tag.doPositioning()")
            if angles is None:
                self.handle_error("Tag.getOrientation()")

        if (not ((position is None) or (angles is None))) and self.positioning_converges(position):
            self.messenger.send_ekf_update(UpdateType.TRILATERATION, self.timing.logical_clock.clock, self.timing.logical_clock.offset,
                                           position, angles.heading, topology=self.neighborhood.current_neighbors)

    @staticmethod
    def positioning_converges(coordinates: Coordinates) -> bool:
        return not (coordinates.x == coordinates.y == coordinates.z == 0.0)

    def ranging(self) -> None:
        ranging_target_id = self.select_ranging_target()

        if ranging_target_id is not None:
            with self.tag_lock: 
                if ranging_target_id not in self.anchors.anchors_dict:
                    ref_coordinates = self.tag.getCoordinates()
                else:
                    ref_coordinates = self.anchors.anchors_dict[ranging_target_id]

                measured_position = self.tag.doRanging(ranging_target_id) 
                angles = self.tag.getOrientation() 

            neighbor_position = array([ref_coordinates.x, ref_coordinates.y, ref_coordinates.z])

            if not ((measured_position is None) or (angles is None)): # These will be None if there were errors when getting the ranging/angles 
                self.messenger.send_ekf_update(UpdateType.RANGING, self.timing.logical_clock.clock, self.timing.logical_clock.offset,
                                               measured_position, angles.heading, neighbors=atleast_2d(neighbor_position),
                                               topology=self.neighborhood.current_neighbors)

    def select_ranging_target(self) -> int:
        """We select a target for doing a range measurement.
        Anchors are prioritized because of their lower uncertainty."""

        if len(self.tag.available_anchors) > 0:
            return random.choice(self.tag.available_anchors)

    def discover_devices(self):
        """Discovers the devices available for localization/ranging.
        Prioritizes the anchors because of their smaller measurement uncertainty.
        If there aren't enough anchors, will use tags as well."""

        with self.tag_lock:
            self.tag.clearAnchors() # Internal tag list automatically discards old tags 
            devices = self.tag.get_device_list("all")

        new_tags = []
        for device in devices:
            if self.tag.is_anchor(device):
                print(f"Task.discover_devices(): Found device: {device}, it's an ANCHOR!", 'info', 'tdma')
                with self.tag_lock: 
                    self.tag.addAnchor(device)
            else:
                print(f"Task.discover_devices(): Found device: {device}, it's a TAG!", 'info', 'tdma')
                new_tags.append(device)

        self.update_neighborhood(new_tags)

    def update_neighborhood(self, new_tags: list) -> None:
        if set(new_tags) != set(self.neighborhood.current_neighbors.keys()):
            self.neighborhood.current_neighbors.clear()
            for tag in new_tags:
                self.neighborhood.add_neighbor(tag, perf_counter(), State.TASK)
                self.neighborhood.changed = True

    def set_manually_measured_anchors(self) -> None:
        if len(self.tag.available_anchors) > 3:
            with self.tag_lock:
                self.tag.configureAnchorSelection(number_of_anchors = len(self.tag.available_anchors))

    def handle_error(self, function_name: str) -> None: 
        """
        Prints the current error on the device with the function where it happened
        """
        with self.tag_lock: 
            self.tag.printCurrentError(function_name) 