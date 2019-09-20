from interfaces import SlotAssignment, Timing, Neighborhood
from interfaces.timing import Timing
from messenger import Messenger

from .constants import State
from .tdmaState import TDMAState


class Listen(TDMAState):
    def __init__(self, slot_assignment: SlotAssignment, timing: Timing, 
                 messenger: Messenger, neighborhood: Neighborhood):
        self.timing = timing
        self.messenger = messenger
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment

    def execute(self) -> State:
        next_state = self.next()
        
        if next_state == State.LISTEN:
            self.messenger.receive_new_message()

        return next_state

    def next(self) -> State:
        if self.timing.in_cycle():
            if self.timing.in_taskslot(self.slot_assignment.pure_send_list):
                return State.TASK
            else:
                return State.LISTEN
        else:
            return State.SYNCHRONIZATION
