from numpy import mean
from time import time
import random

from interfaces import Neighborhood, SlotAssignment, Timing
from messages import (MessageFactory, SynchronizationMessage, UWBSynchronizationMessage)
from messenger import Messenger
from interfaces.timing import COMMUNICATION_DELAY, THRESHOLD_SYNCTIME, SYNCHRONIZATION_PERIOD

from .constants import *
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
        self.time_to_sleep = abs(random.gauss(RANDOM_DELAY_MEAN, RANDOM_DELAY_VARIANCE))
        self.start_t = time()
        self.first_exec_time = None  # Execution time in milliseconds
        self.nb_cycles_neighbors_synced = 0

    def execute(self) -> State:
        if self.first_exec_time is None:
            self.first_exec_time = int(round(time() * SECONDS_TO_MILLISECONDS))

        self.timing.synchronization_offset_mean = 20 if len(self.timing.clock_differential_stat) < NB_SAMPLES_OFFSET \
            else mean(self.timing.clock_differential_stat)

        self.synchronize()
        self.timing.synchronized = abs(self.timing.synchronization_offset_mean) < THRESHOLD_SYNCTIME

        if self.neighborhood.are_neighbors_synced():
            self.nb_cycles_neighbors_synced += 1
        else:
            self.nb_cycles_neighbors_synced = 0

        if self.time_to_sleep <= time() - self.start_t:
            self.broadcast_synchronization_message()
            self.time_to_sleep = abs(random.gauss(RANDOM_DELAY_MEAN, RANDOM_DELAY_VARIANCE))
            self.start_t = time()

        next_state = self.next()
        if next_state == State.SCHEDULING:
            print("Offset: ", self.timing.synchronization_offset_mean)
            self.prepare_next_state()
            print("Entering scheduling...")

        return next_state

    def next(self) -> State:
        current_exec_time = int(round(time() * SECONDS_TO_MILLISECONDS)) - self.first_exec_time

        if self.neighborhood.is_alone() or self.is_left_behind() or \
                (current_exec_time > SYNCHRONIZATION_PERIOD and (self.timing.synchronized or self.is_left_behind())
                 and self.neighborhood.are_neighbors_synced()):
            return State.SCHEDULING
        else:
            return State.SYNCHRONIZATION

    def is_left_behind(self) -> bool:
        return self.nb_cycles_neighbors_synced > 10

    def broadcast_synchronization_message(self) -> None:
        self.timing.logical_clock.update_clock()
        t = int(round(self.timing.logical_clock.clock * TRANSMISSION_SCALING))
        self.messenger.broadcast_synchronization_message(t, self.timing.synchronized)

    def synchronize(self) -> None:
        self.messenger.receive_new_message()
        while not self.messenger.message_box.empty():
            message = self.messenger.message_box.pop()
            if isinstance(message, UWBSynchronizationMessage):
                message.decode()
                self.timing.update_current_time()
                self.update_offset(message.sender_id, message)

            self.messenger.update_neighbor_dictionary()
            self.messenger.receive_new_message()

        self.increment_time_alive()

    def increment_time_alive(self) -> None:
        for msg_id in self.neighborhood.neighbor_synchronization_received.keys():
            self.neighborhood.neighbor_synchronization_received[msg_id].time_alive += 1

    def prepare_next_state(self) -> None:
        self.neighborhood.synchronized_active_neighbor_count = 0
        self.slot_assignment.reset()
        self.timing.clear_synchronization_info()
        self.messenger.message_box.clear()
        self.messenger.received_synced_messages.clear()

    def update_offset(self, sender_id: int, message: UWBSynchronizationMessage) -> None:
        sync_msg = SynchronizationMessage(sender_id=sender_id, clock=self.timing.logical_clock.clock,
                                          neib_logical=(message.synchronized_clock / TRANSMISSION_SCALING))
        sync_msg.offset += COMMUNICATION_DELAY

        if abs(sync_msg.offset) > JUMP_THRESHOLD:
            self.timing.logical_clock.correct_logical_offset(sync_msg.offset)
        else:
            self.collaborative_offset_compensation(sync_msg)

    def collaborative_offset_compensation(self, message: SynchronizationMessage) -> None:
        self.neighborhood.neighbor_synchronization_received[message.sender_id] = message
        if len(self.timing.clock_differential_stat) > NB_SAMPLES_OFFSET:
            self.timing.clock_differential_stat = self.timing.clock_differential_stat[1:] + [message.offset]
        else:
            self.timing.clock_differential_stat.append(message.offset)

        if len(self.neighborhood.neighbor_synchronization_received) >= len(self.neighborhood.current_neighbors):
            total_offset = []
            for id, synchronization in self.neighborhood.neighbor_synchronization_received.items():
                if synchronization.time_alive <= 100:
                    total_offset.append(synchronization.offset)

            offset_correction = sum(total_offset) / (len(total_offset) + 1)
            self.timing.logical_clock.correct_logical_offset(offset_correction)

            self.neighborhood.neighbor_synchronization_received = {}
