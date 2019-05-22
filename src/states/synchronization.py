from .constants import State, JUMP_THRESHOLD
from .tdmaState import TDMAState
from interfaces import Neighborhood, SlotAssignment, Timing
from messages import MessageBox, MessageFactory, UWBSynchronizationMessage, SynchronisationMessage

from timing import THRESHOLD_SYNTIME, syncTimeLen, COMM_DELAY

from time import perf_counter
from numpy import std, mean
from pypozyx import PozyxSerial, RXInfo
from pypozyx.definitions.constants import POZYX_SUCCESS
from pypozyx.structures.generic import Data, SingleRegister


class Synchronization(TDMAState):
    def __init__(self, neighborhood: Neighborhood, slot_assignment: SlotAssignment,
                timing: Timing, message_box: MessageBox, id: int, pozyx: PozyxSerial):
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.timing = timing
        self.message_box = message_box
        self.id = id
        self.pozyx = pozyx

    def execute(self) -> State:
        self.timing.synchronization_offset_mean = 0 if len(self.timing.clock_differential_stat) == 0  \
                                                    else mean(self.timing.clock_differential_stat)
        
        self.synchronize()
        self.broadcast_synchronization_message()

        if self.timing.synchronization_offset_mean > THRESHOLD_SYNTIME:
            self.timing.synchronized = True

        next_state = self.next()
        if next_state == State.SCHEDULING:
            self.reset_scheduling()
            self.timing.clock_differential_dev = std(self.timing.clock_differential_stat)
            self.reset_timing_offsets()

        return next_state

    def next(self) -> State:
        if self.timing.current_time_in_cycle > syncTimeLen and self.timing.synchronized:
            return State.SCHEDULING
        else:
            return State.SYNCHRONIZATION

    def broadcast_synchronization_message(self):
        message = UWBSynchronizationMessage()
        message.synchronized_clock = int(round(self.timing.logical_clock.get_updated_clock()*100000))
        message.encode()

        self.pozyx.sendData(Data([message.data], "I"))

    def synchronize(self):
        # We listen for synchronization messages an arbitrary number of times
        for _ in range(10):
            sender_id, data, status = self.obtain_message_from_pozyx()
            if status == POZYX_SUCCESS and message_handler.is_new_message(sender_id, data):
                message = MessageFactory.create(data)
                if isinstance(message, UWBSynchronizationMessage):
                    message.decode()
                    self.update_offset(sender_id, message)
                else:
                    print("Wrong message type")
                self.update_neighbor_dictionary()
            else:
                self.handle_error()

    def reset_scheduling(self):
        self.slot_assignment.block = [-1] * len(self.slot_assignment.block)
        self.slot_assignment.send_list = [-1] * len(self.slot_assignment.send_list)
        self.slot_assignment.receive_list = [-1] * len(self.slot_assignment.receive_list)
        self.slot_assignment.pure_send_list = []
        self.message_box.next_message = []
        self.neighborhood.synchronized_active_neighbor_count = len(self.neighborhood.current_neighbors) + 1
        self.slot_assignment.update_free_slots()

    def reset_timing_offsets(self):
        self.timing.clock_differential_stat = []
        self.timing.synchronization_offset_mean = 20
        self.timing.synchronized = False
    
    def obtain_message_from_pozyx(self):
        info = RXInfo()
        data = Data([0], 'i')
        status = self.pozyx.readRXBufferData(data)
        self.pozyx.getRxInfo(info)

        return info[0], data[0], status

    def update_offset(self, sender_id: int, message: UWBSynchronizationMessage):
        sync_msg = SynchronisationMessage(sender_id=sender_id, clock=self.timing.logical_clock.getLogicalTime(), neibLogical=message.syncClock/100000)
        sync_msg.offset += COMM_DELAY

        if abs(sync_msg.offset) > JUMP_THRESHOLD:
            self.timing.logical_clock.correct_logical_offset(sync_msg.offset)
        else:
            self.collaborative_offset_compensation(sync_msg)
    
    def collaborative_offset_compensation(self, message: SynchronisationMessage):
        self.neighborhood.neighbor_synchronization_received[message.sender_id] = message
        self.timing.clock_differential.append(message.offset)
        self.timing.clock_differential_stat.append(message.offset)

        if len(self.neighborhood.neighbor_synchronization_received) >= len(self.neighborhood.current_neighbors):
            total_offset = 0
            for _, individual_offset in self.neighborhood.neighbor_synchronization_received:
                total_offset += individual_offset
            
            offset_corection = total_offset / (len(self.neighborhood.neighbor_synchronization_received) + 1)
            self.timing.logical_clock.correct_logical_offset(offset_corection)

            self.neighborhood.neighbor_synchronization_received = {}
            self.timing.clock_differential = []

        self.timing.received_frequency_sample.append(perf_counter())


    def handle_error(self):
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code)
        print("Error occured: " + status)

    def update_neighbor_dictionary(self):
        self.message_box.current_message = MessageFactory.create(self.message_box.last_received_message_data)
        self.message_box.current_message.decode()
        self.neighborhood.current_neighbors[self.message_box.last_received_message_id] = (self.message_box.last_received_message_id,
                                                                                        self.message_box.current_message.message_type,
                                                                                        self.message_box.current_message)
        self.neighborhood.synchronized_active_neighbor_count.append(len(self.neighborhood.current_neighbors))
