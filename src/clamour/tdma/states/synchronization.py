from numpy import mean
from ctypes import c_int32 as int32
from time import time, sleep
import random
from typing import TYPE_CHECKING

from ...custom_terminal import print 
from ..neighborhood import Neighborhood
from ..slot_assignment import SlotAssignment
from ..timing import Timing, COMMUNICATION_DELAY, THRESHOLD_SYNCTIME, SYNCHRONIZATION_PERIOD
from ...messages.messageFactory import MessageFactory
from ...messages.synchronizationMessage import SynchronizationMessage
from ...messages.uwbMessage import UWBSynchronizationMessage

if TYPE_CHECKING:
    from ...messenger import Messenger

from .constants import *
from .tdmaState import TDMAState

class Synchronization(TDMAState):
    """
    During this state, we exchange SYNC messages to align clocks and get all tags into a shared logical timebase. 
    - Receive and process SYNC messages from neighbors 
    - Update local logical clock with offset correction 
    - Periodically broadcast own SYNC message 
    - If enough neighbors synced OR sync period expires, move on. 

    Relevant timing constants: 
    SYNCHRONIZATION_PERIOD controls how long this phase lasts.
    THRESHOLD_SYNCTIME decides whether the node considers itself synchronized.
    COMMUNICATION_DELAY is added when computing offsets.
    SYNCHRONIZATION_PERIOD / 3 is used as an early guard before the node can leave this state.
    """
    def __init__(self, neighborhood: Neighborhood, slot_assignment: SlotAssignment,
                 timing: Timing, messenger: "Messenger", id: int, multiprocess_communication_queue):
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
        self.has_done_first_correction = False

    def execute(self) -> State:
        # Record first execution time to init timing 
        if self.first_exec_time is None:
            self.first_exec_time = int(round(time() * SECONDS_TO_MILLISECONDS))
        # Calculate sync offset 
        self.timing.synchronization_offset_mean = 20 if len(self.timing.clock_differential_stat) < NB_SAMPLES_OFFSET \
            else mean(self.timing.clock_differential_stat)
        # Call sync - this is where msgs are received and processed
        self.synchronize()
        # Check sync status 
        self.timing.synchronized = abs(self.timing.synchronization_offset_mean) < THRESHOLD_SYNCTIME
        # Track sync cycles (consecutive cycles where synced)
        if self.neighborhood.are_neighbors_synced():
            self.nb_cycles_neighbors_synced += 1
        else:
            self.nb_cycles_neighbors_synced = 0
        # Periodically broadcast own sync message 
        if self.time_to_sleep <= time() - self.start_t:
            self.broadcast_synchronization_message()
            self.time_to_sleep = abs(random.gauss(RANDOM_DELAY_MEAN, RANDOM_DELAY_VARIANCE))
            self.start_t = time()
        # Remove old neighbor data
        self.neighborhood.collect_garbage(delay=1)
        # Decide if moving to SCHEDULING 
        next_state = self.next()
        # If transitioning, final broadcast burst to let others sync 
        if next_state == State.SCHEDULING:
            for _ in range(10):
                sleep(0.005)
                self.broadcast_synchronization_message()

            self.prepare_next_state()
            print(f"Synchronization.execute(): Entering scheduling at {self.timing.logical_clock.clock} in cycle with offset {self.timing.synchronization_offset_mean}", 'info', 'tdma')
        return next_state

    def next(self) -> State:
        current_exec_time = int(round(time() * SECONDS_TO_MILLISECONDS)) - self.first_exec_time

        if current_exec_time < SYNCHRONIZATION_PERIOD / 3:
            return State.SYNCHRONIZATION

        if self.neighborhood.is_alone_in_state(State.SYNCHRONIZATION) or \
                current_exec_time > SYNCHRONIZATION_PERIOD or \
                ((self.timing.synchronized or self.is_left_behind()) and
                 self.neighborhood.are_neighbors_synced()):
            # If the tag is the only left trying to sync or ran out of time or everyone is ~~ in sync, let's move on to scheduling
            # Following print explicits what between the 'or's caused the transition 
            info = f"""
            Synchronization.next(), moving to scheduling. Transition trace: 
            Tag is the only one left trying to sync: {self.neighborhood.is_alone_in_state(State.SYNCHRONIZATION)}
            Tag timed out: {current_exec_time > SYNCHRONIZATION_PERIOD}
            Everyone ~ready to move on: {((self.timing.synchronized or self.is_left_behind()) and self.neighborhood.are_neighbors_synced())}
            """
            print(info, 'info', 'tdma')
            self.timing.sync_timestamp = self.timing.logical_clock.clock
            return State.SCHEDULING
        else:
            return State.SYNCHRONIZATION

    def is_left_behind(self) -> bool:
        return self.nb_cycles_neighbors_synced > 10

    def broadcast_synchronization_message(self) -> None:
        self.timing.logical_clock.update_clock()
        t = int32(round(self.timing.logical_clock.clock * TRANSMISSION_SCALING))
        self.messenger.broadcast_synchronization_message(t, self.timing.synchronized)

    def synchronize(self) -> None:
        self.messenger.receive_new_message(State.SYNCHRONIZATION)
        while not self.messenger.message_box.empty():
            message = self.messenger.message_box.pop()
            if isinstance(message, UWBSynchronizationMessage):
                message.decode()
                self.timing.update_current_time()
                self.update_offset(message.sender_id, message)

            self.messenger.update_topology(State.SYNCHRONIZATION)
            self.messenger.receive_new_message(State.SYNCHRONIZATION)

        self.increment_time_alive()

    def increment_time_alive(self) -> None:
        for msg_id in self.neighborhood.neighbor_synchronization_received.keys():
            self.neighborhood.neighbor_synchronization_received[msg_id].time_alive += 1

    def prepare_next_state(self) -> None:
        self.neighborhood.synchronized_active_neighbor_count = 0
        self.slot_assignment.reset()
        self.timing.clear_synchronization_info()
        self.timing.update_task_start_time(len(self.neighborhood.current_neighbors))
        self.messenger.message_box.clear()
        self.messenger.received_messages.clear()
        self.messenger.should_go_back_to_sync = 0

    def update_offset(self, sender_id: int, message: UWBSynchronizationMessage) -> None:
        sync_msg = SynchronizationMessage(sender_id=sender_id, clock=self.timing.logical_clock.clock,
                                          neib_logical=(message.synchronized_clock / TRANSMISSION_SCALING))
        sync_msg.offset += COMMUNICATION_DELAY

        if self.has_done_first_correction:
            if JUMP_THRESHOLD < abs(sync_msg.offset) < SAFE_THRESHOLD:
                self.timing.logical_clock.correct_logical_offset(sync_msg.offset)
            else:
                self.collaborative_offset_compensation(sync_msg)
        else:
            self.has_done_first_correction = True

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