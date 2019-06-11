from pypozyx import PozyxSerial
from pypozyx.definitions.constants import (POZYX_DISCOVERY_ALL_DEVICES, POZYX_SUCCESS)

from interfaces import Anchors, Neighborhood
from messenger import Messenger

from .constants import State
from .tdmaState import TDMAState, print_progress


class Initialization(TDMAState):
    def __init__(self, neighborhood: Neighborhood, anchors: Anchors, 
                 id: int, pozyx: PozyxSerial, messenger: Messenger):
        self.neighborhood = neighborhood
        self.anchors = anchors
        self.id = id
        self.pozyx = pozyx
        self.messenger = messenger

    @print_progress
    def execute(self) -> State:
        self.discover_neighbors()
        return self.next()

    def next(self) -> State:
        return State.SYNCHRONIZATION

    def discover_neighbors(self):
        self.clear_known_devices()
        
        # We scan the network for messages an arbitrary number of times
        for _ in range(1000):
            sender_id, data, _ = self.messenger.obtain_message_from_pozyx()

            if self.messenger.is_new_message(sender_id, data):
                self.messenger.update_neighbor_dictionary()
                self.neighborhood.is_alone = False
        
        self.reset_discovery_settings()

    def clear_known_devices(self):
        self.neighborhood.neighbor_list = []
        self.anchors.available_anchors = []

    def reset_discovery_settings(self):
        self.pozyx.clearDevices()

        if self.pozyx.doDiscovery(discovery_type=POZYX_DISCOVERY_ALL_DEVICES) == POZYX_SUCCESS:
            self.pozyx.printDeviceList()
            self.anchors.discovery_done = False
