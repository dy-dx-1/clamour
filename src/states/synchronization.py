from time import perf_counter

from numpy import mean, std
from pypozyx import POZYX_SUCCESS

from interfaces import Neighborhood, SlotAssignment, Timing
from messages import (MessageFactory, SynchronisationMessage,
                      UWBSynchronizationMessage)
from messenger import Messenger
from timing import COMMUNICATION_DELAY, THRESHOLD_SYNTIME, SYNCHRONIZATION_PERIOD

from .constants import JUMP_THRESHOLD, State
from .tdmaState import TDMAState


class Synchronization(TDMAState):
    def __init__(self, neighborhood: Neighborhood, slot_assignment: SlotAssignment,
                timing: Timing, messenger: Messenger, id: int):
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.timing = timing
        self.id = id
        self.messenger = messenger

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
        if self.timing.current_time_in_cycle > SYNCHRONIZATION_PERIOD and self.timing.synchronized:
            return State.SCHEDULING
        else:
            return State.SYNCHRONIZATION

    def broadcast_synchronization_message(self):
        time = int(round(self.timing.logical_clock.get_updated_clock()*100000))
        self.messenger.broadcast_synchronization_message(time)

    def synchronize(self):
        # We listen for synchronization messages an arbitrary number of times
        for _ in range(10):
            sender_id, data, status = self.messenger.obtain_message_from_pozyx()
            if status == POZYX_SUCCESS and self.messenger.is_new_message(sender_id, data):
                message = MessageFactory.create(data)
                if isinstance(message, UWBSynchronizationMessage):
                    message.decode()
                    self.update_offset(sender_id, message)
                else:
                    print("Wrong message type")
                self.messenger.update_neighbor_dictionary()
            else:
                self.messenger.handle_error()

    def reset_scheduling(self):
        self.slot_assignment.block = [-1] * len(self.slot_assignment.block)
        self.slot_assignment.send_list = [-1] * len(self.slot_assignment.send_list)
        self.slot_assignment.receive_list = [-1] * len(self.slot_assignment.receive_list)
        self.slot_assignment.pure_send_list = []
        self.messenger.queue.clear()
        self.neighborhood.synchronized_active_neighbor_count = len(self.neighborhood.current_neighbors) + 1
        self.slot_assignment.update_free_slots()

    def reset_timing_offsets(self):
        self.timing.clock_differential_stat = []
        self.timing.synchronization_offset_mean = 20
        self.timing.synchronized = False
    
    def update_offset(self, sender_id: int, message: UWBSynchronizationMessage):
        sync_msg = SynchronisationMessage(sender_id=sender_id, clock=self.timing.logical_clock.getLogicalTime(), neibLogical=message.syncClock/100000)
        sync_msg.offset += COMMUNICATION_DELAY

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
