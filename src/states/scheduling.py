from interfaces import Neighborhood, SlotAssignment, Timing
from interfaces.timing import (NB_NODES, SYNCHRONIZATION_PERIOD, TASK_START_TIME, SCHEDULING_SLOT_DURATION)
from messenger import Messenger

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

    def execute(self) -> State:
        self.slot_assignment.update_free_slots()

        if int(((self.timing.current_time_in_cycle - SYNCHRONIZATION_PERIOD) % (NB_NODES * SCHEDULING_SLOT_DURATION))
               / SCHEDULING_SLOT_DURATION) == self.id & TAG_ID_MASK:
            self.messenger.broadcast_control_message()
        else:
            self.messenger.receive_message()

        self.slot_assignment.update_free_slots()
        self.update_pure_send_list()

        return self.next()

    def next(self) -> State:
        if self.timing.current_time_in_cycle > TASK_START_TIME:
            print("Receive List: ", self.slot_assignment.receive_list)
            print("Send List: ", self.slot_assignment.pure_send_list)
            print(self.slot_assignment.free_slots)
            print("Entering listen state...")
            return State.LISTEN
        else:
            return State.SCHEDULING

    def update_pure_send_list(self):
        self.slot_assignment.pure_send_list = [x for x in range(len(self.slot_assignment.send_list))
                                               if self.slot_assignment.send_list[x] not in [-1, -2]]
