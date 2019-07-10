from pypozyx import PozyxSerial
from pypozyx.definitions.constants import (POZYX_DISCOVERY_ALL_DEVICES, POZYX_SUCCESS)

from interfaces import Anchors, Neighborhood
from messenger import Messenger

from .constants import State
from .tdmaState import TDMAState, print_progress


class Initialization(TDMAState):
    def __init__(self, neighborhood: Neighborhood, anchors: Anchors, 
                 id: int, pozyx: PozyxSerial, messenger: Messenger,
                 multiprocess_communication_queue):
        self.neighborhood = neighborhood
        self.anchors = anchors
        self.id = id
        self.pozyx = pozyx
        self.messenger = messenger
        self.multiprocess_communication_queue = multiprocess_communication_queue

    def execute(self) -> State:
        self.multiprocess_communication_queue.put("initialization")
        self.discover_neighbors()
        return self.next()

    def next(self) -> State:
        print("Entering synchronization...")
        return State.SYNCHRONIZATION

    def discover_neighbors(self):  # todo @yanjun: By receiving msgs? If in this case, who is sending? If by pozyx.doDiscovery, then need to do at different time. I used sleeping for a period related with id to randomize this process?
        self.clear_known_devices()
        
        # We scan the network for messages an arbitrary number of times
        for _ in range(1000):
            if self.messenger.receive_new_message():
                self.messenger.update_neighbor_dictionary()
                self.messenger.message_box.pop()  # Discard the message, it will not be needed afterwards
        
        self.reset_discovery_settings()

    def clear_known_devices(self):
        self.neighborhood.neighbor_list = []
        self.anchors.available_anchors = []

    def reset_discovery_settings(self):
        self.pozyx.clearDevices()

        if self.pozyx.doDiscovery(discovery_type=POZYX_DISCOVERY_ALL_DEVICES) == POZYX_SUCCESS: # todo @yanjun: update neighbood tags here.
            self.pozyx.printDeviceList()
            self.anchors.discovery_done = False
