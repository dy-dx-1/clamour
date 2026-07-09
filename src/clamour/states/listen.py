from typing import TYPE_CHECKING

from .constants import State
from .tdmaState import TDMAState

from ..interfaces import SlotAssignment, Timing, Neighborhood
from ..custom_terminal import print 

if TYPE_CHECKING:
    from ..messenger import Messenger

class Listen(TDMAState):
    """
    Passive state where the tag waits for it's task slot to become active: 
    - Checks whether current time is still inside task cycle
    - If current timeslot is for us, move to TASK. Else, stay in listen. 
    
    Relevant timing constants: 
    in_cycle() uses the logical clock and the task cycle timing.
    in_taskslot() checks whether the current task slot belongs to this node.
    SLOT_FOR_RESET is used in the cycle limit logic to stop the cycle early enough to reset.

    NOTES:
    - All tags enter at roughly the same time, but only tags whose assigned slot match current slot will become active. 
    """
    def __init__(self, slot_assignment: SlotAssignment, timing: Timing, 
                 messenger: "Messenger", neighborhood: Neighborhood):
        self.timing = timing
        self.messenger = messenger
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.should_go_back_to_sync = False

    def execute(self) -> State:
        self.should_go_back_to_sync = self.messenger.receive_new_message(State.LISTEN)[1]
        return self.next()

    def next(self) -> State:
        if self.should_go_back_to_sync or not self.timing.in_cycle():
            print(f"Listen.next(): should_go_back_to_sync: {self.should_go_back_to_sync} time: {self.timing.in_cycle()}", 'info', 'tdma')
            self.should_go_back_to_sync = False
            self.messenger.message_box.clear()
            self.messenger.received_messages.clear()
            return State.SYNCHRONIZATION
        else:
            return State.TASK if self.timing.in_taskslot(self.slot_assignment.pure_send_list) else State.LISTEN