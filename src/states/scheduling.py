from interfaces import Neighborhood, SlotAssignment, Timing
from interfaces.timing import (NB_NODES, SYNCHRONIZATION_PERIOD, TASK_START_TIME,
                               SCHEDULING_SLOT_DURATION, NB_TASK_SLOTS)
from messenger import Messenger
from random import sample

from .constants import State, TAG_ID_MASK
from .tdmaState import TDMAState


class Scheduling(TDMAState):
    def __init__(self, neighborhood: Neighborhood, slot_assignment: SlotAssignment,
                 timing: Timing, id: int, messenger: Messenger):
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.timing = timing
        self.id = id
        self.messenger = messenger
        self.first_scheduling_execution = True

    def execute(self) -> State:
        self.messenger.clear_non_scheduling_messages()
        self.slot_assignment.update_free_slots()

        if self.neighborhood.is_alone_in_state(State.SCHEDULING):
            self.alone_slot_assignment()
        else:
            self.community_slot_assignment()

        return self.next()

    def next(self) -> State:
        if self.neighborhood.is_alone_in_state(-1) or self.timing.current_time_in_cycle > TASK_START_TIME:
            print("Receive List: ", self.slot_assignment.receive_list)
            print("Send List: ", self.slot_assignment.pure_send_list)
            print("Entering listen state...")
            return State.LISTEN
        else:
            return State.SCHEDULING

    def community_slot_assignment(self):
        if self.is_broadcast_slot():
            self.messenger.broadcast_control_message()
        else:
            self.messenger.receive_message(State.SCHEDULING)

        self.slot_assignment.update_free_slots()
        self.update_pure_send_list()

    def alone_slot_assignment(self):
        random_slots = sample(range(NB_TASK_SLOTS), int(2 * NB_TASK_SLOTS / 3))  # We must leave some free slots
        self.slot_assignment.pure_send_list = [i if i in random_slots else -1 for i in range(NB_TASK_SLOTS)]

    def is_broadcast_slot(self) -> bool:
        return int(((self.timing.current_time_in_cycle - SYNCHRONIZATION_PERIOD) % (NB_NODES * SCHEDULING_SLOT_DURATION))
                / SCHEDULING_SLOT_DURATION) == self.id & TAG_ID_MASK

    def update_pure_send_list(self):
        self.slot_assignment.pure_send_list = [x for x in range(len(self.slot_assignment.send_list))
                                               if self.slot_assignment.send_list[x] not in [-1, -2]
                                               and self.slot_assignment.receive_list[x] == -1]


