from numpy import mean

from interfaces import Neighborhood, SlotAssignment, Timing
from messages import (MessageFactory, SynchronizationMessage, UWBSynchronizationMessage)
from messenger import Messenger
from interfaces.timing import COMMUNICATION_DELAY, THRESHOLD_SYNCTIME, SYNCHRONIZATION_PERIOD

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
        self.timing.synchronization_offset_mean = 20 if len(self.timing.clock_differential_stat) < 10  \
                                                    else mean(self.timing.clock_differential_stat)
        
        self.synchronize()
        self.broadcast_synchronization_message()

        if self.timing.synchronization_offset_mean < THRESHOLD_SYNCTIME:
            self.timing.synchronized = True

        next_state = self.next()
        if next_state == State.SCHEDULING:
            print("Offset: ", self.timing.synchronization_offset_mean)
            self.reset_scheduling()
            self.reset_timing_offsets()
            self.messenger.message_box.queue.clear()
            print("Entering scheduling...")

        return next_state

    def next(self) -> State:
        if self.timing.current_time_in_cycle > SYNCHRONIZATION_PERIOD and self.timing.synchronized:
            return State.SCHEDULING
        else:
            return State.SYNCHRONIZATION

    def broadcast_synchronization_message(self):
        self.timing.logical_clock.update_clock()
        time = int(round(self.timing.logical_clock.clock * 100000))
        self.messenger.broadcast_synchronization_message(time)

    def synchronize(self):
        # We listen for synchronization messages an arbitrary number of times
        for _ in range(10):
            if self.messenger.receive_new_message():
                message = self.messenger.message_box.pop()
                if isinstance(message, UWBSynchronizationMessage):
                    message.decode()
                    self.update_offset(message.sender_id, message)

                self.messenger.update_neighbor_dictionary()

    def reset_scheduling(self):
        self.slot_assignment.block = [-1] * len(self.slot_assignment.block)
        self.slot_assignment.send_list = [-1] * len(self.slot_assignment.send_list)
        self.slot_assignment.receive_list = [-1] * len(self.slot_assignment.receive_list)
        self.slot_assignment.pure_send_list = []
        self.messenger.message_box.queue.clear()
        self.neighborhood.synchronized_active_neighbor_count = len(self.neighborhood.current_neighbors) + 1
        self.slot_assignment.update_free_slots()

    def reset_timing_offsets(self):
        self.timing.clock_differential_stat = []
        self.timing.synchronization_offset_mean = 20
        self.timing.synchronized = False
    
    def update_offset(self, sender_id: int, message: UWBSynchronizationMessage):
        sync_msg = SynchronizationMessage(sender_id=sender_id, clock=self.timing.logical_clock.clock,
                                          neib_logical=message.synchronized_clock/100000)
        sync_msg.offset += COMMUNICATION_DELAY

        if abs(sync_msg.offset) > JUMP_THRESHOLD:
            self.timing.logical_clock.correct_logical_offset(sync_msg.offset)
        else:
            self.collaborative_offset_compensation(sync_msg)
    
    def collaborative_offset_compensation(self, message: SynchronizationMessage):
        self.neighborhood.neighbor_synchronization_received[message.sender_id] = message
        self.timing.clock_differential_stat.append(message.offset)

        if len(self.neighborhood.neighbor_synchronization_received) >= len(self.neighborhood.current_neighbors):
            total_offset = 0
            for _, synchronization in self.neighborhood.neighbor_synchronization_received.items():
                total_offset += synchronization.offset
            
            offset_correction = total_offset / (len(self.neighborhood.neighbor_synchronization_received) + 1)
            self.timing.logical_clock.correct_logical_offset(offset_correction)

            self.neighborhood.neighbor_synchronization_received = {}
