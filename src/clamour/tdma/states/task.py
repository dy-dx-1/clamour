from multiprocessing.synchronize import Lock
from time import perf_counter, sleep 
from typing import TYPE_CHECKING

from ...custom_terminal import print 
from ...interfaces import Tag, Coordinates, Anchors
from ..neighborhood import Neighborhood
from ..slot_assignment import SlotAssignment
from ..timing import Timing

if TYPE_CHECKING:
    from ...messenger import Messenger

from .constants import State
from .tdmaState import TDMAState


class Task(TDMAState):
    """
    Performs actual localization/ranging work 
    - If current slot is for this tag, position it 
    - May also discover neighbors and broadcast topology updates 
    - Uses enough_time_left() to avoid running out of time 

    Relevant timing constants: 
    TASK_SLOT_DURATION is the length of each task slot.
    NB_TASK_SLOTS is the number of task slots in a frame.
    TASK_FRAME_DURATION is the total length of one task frame.
    NB_TASK_FRAMES_PER_CYCLE defines how many task frames are included before
    returning to synchronization.
    MAX_RANGING_DELAY limits how late into a slot the node will still do work.
    """
    def __init__(self, timing: Timing, anchors: Anchors, neighborhood: Neighborhood, 
                 shared_tag: Tag, shared_tag_lock: Lock, messenger: "Messenger",
                 slot_assignment: SlotAssignment):
        self.timing = timing
        self.anchors = anchors # TODO Remove, unused 
        self.tag = shared_tag
        self.tag_lock = shared_tag_lock
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.messenger = messenger
        self.frame_id_done_discover = -1
        self.neighborUpdateFrequency = 5 # every five frames, do discovery and update neighbor information
        self.ranging_references = {'anchors':{}, 'tags':{}}     # Keeps track of what anchors/tags we previously used for ranging to allow variety in selection 

    def execute(self) -> State:
        if self.frame_id_done_discover != self.timing.frame_id and not self.timing.frame_id % self.neighborUpdateFrequency: # do discovery at first slot of every $neighborUpdateFrequency frames
            self.frame_id_done_discover = self.timing.frame_id
            self.discover_devices()
            self.neighborhood.collect_garbage()

        if self.timing.enough_time_left():
            self.collect_ranges() 
            #self.testTDMA()

        if self.neighborhood.changed:
            self.messenger.broadcast_topology_message()  # Broadcast topology change to other devices
            self.messenger.send_topology_update(self.timing.logical_clock.clock, self.timing.logical_clock.offset, self.neighborhood.current_neighbors)
            self.neighborhood.changed = False

        return self.next()

    def testTDMA(self):
        tosend = [0xFF] * 8
        temp = self.slot_assignment.pure_send_list.copy()
        for ele in temp:
            if ele<0:
                temp.remove(ele)
        for i in range(min(len(temp), 8)):
            tosend[i] = temp[i]
        if self.timing.current_slot_id == 0:
            tosend[-1] = (0 if tosend[-1]==255 else tosend[-1])
            with self.tag_lock:
                self.tag.broadcast(payload=tosend) # NOTE: previously there were 9 B's, but I don't think it matches format of tosend (26June26)
        else:
            tosend[-1] = (self.timing.current_slot_id-1 if tosend[-1]==255 else tosend[-1])
            print(f"Task.testTDMA(): tosend: {tosend}", 'info', 'tdma')
            with self.tag_lock:
                self.tag.broadcast(payload=tosend)
        print(f"Task.testTDMA(): {self.timing.frame_id} {self.timing.current_slot_id} {self.timing.get_full_cycle_duration()} {self.timing.current_time_in_cycle}", 'info', 'tdma')

    def next(self) -> State:
        if self.timing.in_cycle():
            return State.TASK if self.timing.in_taskslot(self.slot_assignment.pure_send_list) else State.LISTEN
        else:
            print("Task.next(): Moving to SYNC state", 'info', 'tdma')
            return State.SYNCHRONIZATION

    def collect_ranges(self): 
        """
        Ranges with appropriate anchors/neighbors through an intelligent selection policy. 
        Sends the collected info to the state estimator for positioning. 
        """
        anchor_zs = []
        tag_zs = []  
        for target_id in self.select_ranging_targets():
            z, target_pos = self.tag.ranging(target_id)  # target_pos also holds covar and will only be returned and used if target_id is another tag
            if z: # Successful measurement, fetch position and add 
                if self.tag.is_anchor(target_id): 
                    anchor_zs.append( (target_id, z) )   # (id, range_measure_in_mm) 
                else: 
                    if target_pos: # If None, we didn't get covariance info, so won't pass it to estimator
                        tag_zs.append( (target_id, target_pos, z) ) 
            sleep(0.000001) # Short break to ensure exchange finished. Should be more than enough.
        # Packing in an update message and sending to estimator 
        self.messenger.send_range_update(clock=self.timing.logical_clock.clock, 
                                         offset=self.timing.logical_clock.offset,
                                         anchors_ranging_data=anchor_zs,
                                         tags_ranging_data=tag_zs,
                                         yaw = self.tag.orientation.heading,
                                         topology= self.neighborhood.current_neighbors)

    def select_ranging_targets(self)->set:
        """
        Selects the best anchors/neighbors to use when ranging to balance computation requirements and information gathered.
        Returns a tuple of IDs for ranging targets 
        """
        # 2026-08-14 NOTE in the future this is a major part of what I'll be modifying for my thesis
        # for now, simply doing what clamour was doing before, i.e. get anchor ranges if >=3 anchors and 1 neighbor/anchor if not
        if len(self.tag.available_anchors)>=3: 
            return self.tag.available_anchors
        else:
            return set([self.select_single_ranging_target()]) 

    def select_single_ranging_target(self) -> int|None:
        """Selects a target for doing a range measurement.
        Anchors are prioritized because of their lower uncertainty.
        When multiple devices are available, we always select the 
        least used to reduce overall uncertainty.""" 
        if len(self.tag.available_anchors) > 0:
            # Amongs all available anchors, pick the one who has the lowest use in the dict of ranging references 
            anchor_refs = self.ranging_references['anchors']
            selected_anchor = min(self.tag.available_anchors, 
                             key=lambda id: anchor_refs.get(id, 0))
            anchor_refs[selected_anchor] = anchor_refs.get(selected_anchor, 0) + 1 
            return selected_anchor
        elif len(self.tag.active_tags) > 0: 
            tag_refs = self.ranging_references['tags']
            selected_tag = min(self.tag.active_tags, 
                             key=lambda id: tag_refs.get(id, 0))
            tag_refs[selected_tag] = tag_refs.get(selected_tag, 0) + 1 
            return selected_tag
        else:
            return None 

    def discover_devices(self):
        """Discovers the devices available for localization/ranging."""

        with self.tag_lock:
            self.tag.clear_anchors() # Internal tag list automatically discards old tags 
            devices = self.tag.get_device_list("all")

        new_tags = []
        for device in devices:
            if self.tag.is_anchor(device):
                print(f"Task.discover_devices(): Found device: {device}, it's an ANCHOR!", 'info', 'tdma')
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
