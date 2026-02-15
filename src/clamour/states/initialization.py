import random
from multiprocessing import Lock
from interfaces.tag import Tag
from pypozyx import PozyxSerial, Data    ### TODO REMOVE DATA DEPENDENCY OR ADAPT 
from pypozyx.definitions.constants import (POZYX_DISCOVERY_TAGS_ONLY)
from time import sleep

from interfaces import Anchors, Neighborhood
from messenger import Messenger
from pozyx_utils import PozyxDiscoverer

from .constants import State
from .tdmaState import TDMAState


class Initialization(TDMAState):
    def __init__(self, neighborhood: Neighborhood, anchors: Anchors, 
                 id: int, tag: Tag, messenger: Messenger,
                 multiprocess_communication_queue, shared_tag_lock: Lock):
        self.neighborhood = neighborhood
        self.anchors = anchors
        self.id = id
        self.tag = tag
        self.tag_lock = shared_tag_lock
        self.messenger = messenger
        self.multiprocess_communication_queue = multiprocess_communication_queue

    def execute(self) -> State:
        sleep(abs(random.gauss(0.02, 0.05)))
        self.discover_neighbors()
        return self.next()

    def next(self) -> State:
        print("Entering synchronization...")
        return State.SYNCHRONIZATION

    def clear_tag_buffer(self):
        print(self.tag.sendData(destination=self.id, data=Data([0], 'i')))
        sleep(0.25)
        for _ in range(50):
            print(self.messenger.obtain_message_from_tag())
            sleep(0.05)

    def discover_neighbors(self):
        self.clear_known_devices()
        devices = PozyxDiscoverer.get_device_list(self.pozyx, self.pozyx_lock, POZYX_DISCOVERY_TAGS_ONLY)

        print("Tags discovered: ", devices)

        self.messenger.update_topology(State.SYNCHRONIZATION, devices)  # Put state to Sync for next phase

    def clear_known_devices(self):
        with self.tag_lock:
            self.tag.clearDevices()

        self.neighborhood.neighbor_list = []
        self.anchors.available_anchors = []
        self.anchors.discovery_done = False
