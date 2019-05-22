from .constants import State
from .tdmaState import TDMAState
from interfaces import Neighborhood, Anchors
from messages import MessageBox
from messageFactory import MessageFactory

from pypozyx import PozyxSerial, RXInfo
from pypozyx.definitions.constants import POZYX_DISCOVERY_ALL_DEVICES, POZYX_SUCCESS
from pypozyx.structures.generic import Data


class Initialization(TDMAState):
    def __init__(self, neighborhood: Neighborhood, message_box: MessageBox, 
                anchors: Anchors, id: int, pozyx: PozyxSerial):
        self.neighborhood = neighborhood
        self.message_box = message_box
        self.anchors = anchors
        self.id = id
        self.pozyx = pozyx

    def execute(self) -> State:
        self.discover_neighbors()
        return self.next()

    def next(self) -> State:
        return State.SYNCHRONIZATION

    def discover_neighbors(self):
        self.clear_known_devices()
        
        # We scan the network for messages an arbitrary number of times
        for _ in range(1000):
            sender_id, data = self.obtain_message_from_pozyx()

            if message_handler.is_new_message(sender_id, data):
                self.update_neighbor_dictionary()
                self.neighborhood.add_anchor_to_neighbors(sender_id)
                self.neighborhood.is_alone = False
        
        self.reset_discovery_settings()

    def clear_known_devices(self):
        self.neighborhood.neighbor_list = []
        self.anchors.available_anchors = []

    def obtain_message_from_pozyx(self):
        info = RXInfo()
        data = Data([0], 'i')
        self.pozyx.getRxInfo(info)
        self.pozyx.readRXBufferData(data)

        return info[0], data[0]

    def update_neighbor_dictionary(self):
        self.message_box.current_message = MessageFactory.create(self.message_box.last_received_message_data)
        self.message_box.current_message.decode()
        self.neighborhood.current_neighbors[self.message_box.last_received_message_id] = (self.message_box.last_received_message_id,
                                                                                        self.message_box.current_message.message_type,
                                                                                        self.message_box.current_message)
        self.neighborhood.synchronized_active_neighbor_count.append(len(self.neighborhood.current_neighbors))

    def reset_discovery_settings(self):
        self.pozyx.clearDevices()

        if self.pozyx.doDiscovery(discovery_type=POZYX_DISCOVERY_ALL_DEVICES) == POZYX_SUCCESS:
            self.pozyx.printDeviceList()
            self.anchors.discovery_done = False
