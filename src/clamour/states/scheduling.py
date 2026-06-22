from typing import TYPE_CHECKING
from random import sample, randint

from .constants import State, TAG_ID_MASK
from .tdmaState import TDMAState 

from ..interfaces import Neighborhood, SlotAssignment, Timing
from ..interfaces.timing import NB_NODES, NB_TASK_SLOTS, SCHEDULING_SLOT_DURATION
from ..custom_terminal import print 

if TYPE_CHECKING:
    from ..messenger import Messenger

class Scheduling(TDMAState):
    def __init__(self, neighborhood: Neighborhood, slot_assignment: SlotAssignment,
                 timing: Timing, tag_id: int, messenger: "Messenger"):
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.timing = timing
        self.id = tag_id
        self.messenger = messenger
        self.first_scheduling_execution = True
        self.should_go_back_to_sync = False

    def execute(self) -> State:
        self.messenger.clear_non_scheduling_messages()
        self.slot_assignment.update_free_slots()

        if self.neighborhood.is_alone_in_state(-1):
            self.alone_slot_assignment()
        else:
            self.community_slot_assignment()

        return self.next()

    def next(self) -> State:
        if self.should_go_back_to_sync:
            self.should_go_back_to_sync = False
            return State.SYNCHRONIZATION

        if self.neighborhood.is_alone_in_state(-1) or (self.timing.logical_clock.clock-self.timing.sync_timestamp) > self.timing.task_start_time:
            print(f"Slot assignement RECEIVE list: {self.slot_assignment.receive_list}", 'info', 'tdma')
            print(f"Slot assignement SEND list: {self.slot_assignment.pure_send_list}", 'info', 'tdma')
            self.timing.cycle_start = self.timing.logical_clock.clock

            if len(self.slot_assignment.pure_send_list) == 0:
                print(f"Scheduling.next(): -------Artificially adding slots -------", 'info', 'tdma')
                self.slot_assignment.pure_send_list.extend({randint(0, NB_TASK_SLOTS) for _ in range(2)})

            self.messenger.message_box.clear()
            self.messenger.received_messages.clear()
            self.messenger.should_go_back_to_sync = 0

            return State.LISTEN
        else:
            return State.SCHEDULING

    def community_slot_assignment(self):
        if self.is_broadcast_slot():
            self.messenger.broadcast_control_message()
        else:
            self.should_go_back_to_sync = self.messenger.receive_message(State.SCHEDULING)

        self.slot_assignment.update_free_slots()
        self.update_pure_send_list()

    def alone_slot_assignment(self):
        random_slots = sample(range(NB_TASK_SLOTS), int(2 * NB_TASK_SLOTS / 3))  # We must leave some free slots
        self.slot_assignment.pure_send_list = [i if i in random_slots else -1 for i in range(NB_TASK_SLOTS)]

    def is_broadcast_slot(self) -> bool: 
        # NOTE: careful, NB_NODES must match highest ID to allow for slot proposal to be reached. See comment in definition of NB_NODES
        return int(((self.timing.logical_clock.clock - self.timing.sync_timestamp) % (NB_NODES * SCHEDULING_SLOT_DURATION))
                / SCHEDULING_SLOT_DURATION) == self.id & TAG_ID_MASK

    def update_pure_send_list(self):
        self.slot_assignment.pure_send_list = [x for x in range(len(self.slot_assignment.send_list))
                                               if self.slot_assignment.send_list[x] not in [-1, -2]
                                               and self.slot_assignment.receive_list[x] == -1]