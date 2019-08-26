from numpy import mean

from interfaces import Neighborhood, SlotAssignment, Timing
from messages import (MessageFactory, SynchronizationMessage, UWBSynchronizationMessage)
from messenger import Messenger
from interfaces.timing import COMMUNICATION_DELAY, THRESHOLD_SYNCTIME, SYNCHRONIZATION_PERIOD

from .constants import JUMP_THRESHOLD, State
from .tdmaState import TDMAState


class Synchronization(TDMAState):
    def __init__(self, neighborhood: Neighborhood, slot_assignment: SlotAssignment,
                 timing: Timing, messenger: Messenger, id: int, multiprocess_communication_queue):
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.timing = timing
        self.id = id
        self.messenger = messenger
        self.multiprocess_communication_queue = multiprocess_communication_queue

    def execute(self) -> State:
        self.timing.synchronization_offset_mean = 20 if len(self.timing.clock_differential_stat) < 10  \
                                                    else mean(self.timing.clock_differential_stat)

        self.synchronize()

        if self.timing.synchronization_offset_mean < THRESHOLD_SYNCTIME:
            print('SYNCED :D')
            self.timing.synchronized = True

        self.broadcast_synchronization_message()

        next_state = self.next()
        if next_state == State.SCHEDULING:
            print("Offset: ", self.timing.synchronization_offset_mean)
            # TODO: what happens here if all devices are not synced?
            self.reset_scheduling()
            self.reset_timing_offsets()
            self.messenger.message_box.clear()
            print("Entering scheduling...")

        return next_state

    def next(self) -> State:
        print('(STEP) next')
        print(SYNCHRONIZATION_PERIOD, self.timing.current_time_in_cycle,
              self.timing.synchronized, self.neighborhood.are_neighbors_synced())
        print('IS ALONE? ', self.neighborhood.is_alone())
        print('IS OVER SYNC PERIOD? ', self.timing.current_time_in_cycle > SYNCHRONIZATION_PERIOD)
        print('IS SYNCED? ', self.timing.synchronized)
        print('ARE NEIGHBORS SYNCED? ', self.neighborhood.are_neighbors_synced())
        if self.neighborhood.is_alone() or \
                ((self.timing.current_time_in_cycle > SYNCHRONIZATION_PERIOD and self.timing.synchronized) and
                    self.neighborhood.are_neighbors_synced()):  # TODO: make sure it doesnt get stuck forever
            print('STATE SCHEDULING')
            return State.SCHEDULING
        else:
            print('STATE SYNCHRONIZATION')
            return State.SYNCHRONIZATION

    def broadcast_synchronization_message(self) -> None:
        print('(STEP) broadcast sync message')
        self.timing.logical_clock.update_clock()
        print('Logical clock: ', self.timing.logical_clock.clock)
        time = int(round(self.timing.logical_clock.clock * 100000))
        self.messenger.broadcast_synchronization_message(time, self.timing.synchronized)

    def synchronize(self):
        print('(STEP) synchronize')
        # We listen for synchronization messages an arbitrary number of times
        # todo @Yanjun, how this arbitrary number works?
        for _ in range(10):
            if self.messenger.receive_new_message():
                print('new message!')
                message = self.messenger.message_box.pop()
                if isinstance(message, UWBSynchronizationMessage):
                    message.decode()
                    self.update_offset(message.sender_id, message)

                self.messenger.update_neighbor_dictionary()

    def reset_scheduling(self):
        print('(STEP) Reset scheduling')
        self.slot_assignment.block = [-1] * len(self.slot_assignment.block)
        self.slot_assignment.send_list = [-1] * len(self.slot_assignment.send_list)
        self.slot_assignment.receive_list = [-1] * len(self.slot_assignment.receive_list)
        self.slot_assignment.pure_send_list = []
        self.messenger.message_box.clear()
        self.neighborhood.synchronized_active_neighbor_count = 0
        self.slot_assignment.update_free_slots()

    def reset_timing_offsets(self):
        print('(STEP) reset timing offsets')
        self.timing.clock_differential_stat = []
        self.timing.synchronization_offset_mean = 20
        self.timing.synchronized = False
    
    def update_offset(self, sender_id: int, message: UWBSynchronizationMessage):
        print('(STEP) update offset')
        sync_msg = SynchronizationMessage(sender_id=sender_id, clock=self.timing.logical_clock.clock,
                                          neib_logical=message.synchronized_clock/100000)
        sync_msg.offset += COMMUNICATION_DELAY

        if abs(sync_msg.offset) > JUMP_THRESHOLD:
            print("Jumped correction")
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
