from .constants import State
from .tdmaState import TDMAState
from interfaces import Neighborhood, SlotAssignment, Timing
from interfaces.timing import Timing, FCYCLE, SLOT_FOR_RESET, tdmaExcStartTime
from messages import MessageBox, MessageFactory, UWBCommunicationMessage

from pypozyx import PozyxSerial, RXInfo
from pypozyx.definitions.constants import POZYX_DISCOVERY_ALL_DEVICES, POZYX_SUCCESS
from pypozyx.structures.generic import Data

class Listen(TDMAState):
    def __init__(self, neighborhood: Neighborhood, slot_assignment: SlotAssignment,
                 timing: Timing, message_box: MessageBox, pozyx: PozyxSerial):
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.timing = timing
        self.message_box = message_box
        self.pozyx = pozyx

    def execute(self) -> State:
        self.timing.update_frame_id()
        self.timing.update_slot_id()
        next_state = self.next()
        
        if next_state == State.LISTEN:
            self.listen_for_messages()

        return next_state

    def next(self) -> State:
        if (self.timing.current_time_in_cycle < FCYCLE - SLOT_FOR_RESET) and (self.timing.current_time_in_cycle > tdmaExcStartTime):
            if self.timing.current_slot_id in self.slot_assignment.pure_send_list:
                return State.TASK
            else:
                return State.LISTEN
        else:
            return State.SYNCHRONIZATION

    def listen_for_messages(self):
        sender_id, data = self.obtain_message_from_pozyx()

        if message_handler.is_new_message(sender_id, data):
            self.update_neighbor_dictionary()
            if isinstance(self.message_box.current_message, UWBCommunicationMessage):
                # TODO: when the device is in wait state, it may still perform actions that do not require interaction,
                #       such as counting steps using a pedometer
                pass
        

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
