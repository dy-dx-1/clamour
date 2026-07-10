import random
from time import sleep
from multiprocessing.synchronize import Lock
from typing import TYPE_CHECKING

from .constants import State
from .tdmaState import TDMAState
from ..neighborhood import Neighborhood

from ...custom_terminal import print 
from ...interfaces import Tag, Anchors

if TYPE_CHECKING:
    from ...messenger import Messenger

class Initialization(TDMAState):
    """
    Initial state, only runs once to: 
    - Discover nearby devices and build initial neighborhood 
    - Move straight into SYNC 
    """
    def __init__(self, neighborhood: Neighborhood, anchors: Anchors, 
                 tag: Tag, messenger: "Messenger",
                 multiprocess_communication_queue, shared_tag_lock: Lock):
        self.neighborhood = neighborhood
        self.anchors = anchors
        self.tag = tag
        self.tag_lock = shared_tag_lock
        self.messenger = messenger
        self.multiprocess_communication_queue = multiprocess_communication_queue

    def execute(self) -> State:
        sleep(abs(random.gauss(0.02, 0.05)))
        self.discover_neighbors()
        return self.next()

    def next(self) -> State:
        print("Initialization.next(): Entering synchronization...", 'info', 'tdma')
        return State.SYNCHRONIZATION

    def discover_neighbors(self):
        self.clear_known_devices()
        with self.tag_lock:
            devices = self.tag.get_device_list(discovery_type = "tag")

        print(f"Initialization.discover_neighbors(): Tags discovered: {devices}", 'info', 'tdma')

        self.messenger.update_topology(State.SYNCHRONIZATION, devices)  # Put state to Sync for next phase

    def clear_known_devices(self):
        with self.tag_lock:
            self.tag.clear_anchors()
        # NOTE 2026-06-21: I deleted clearing of some properties anchors.discovery_done and neighborhood.neighbor_list that don't seem to exist anywhere else in code 