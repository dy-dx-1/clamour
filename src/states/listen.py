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
        self.should_go_back_to_sync = False

    def execute(self) -> State:
        self.should_go_back_to_sync = self.messenger.receive_new_message(State.LISTEN)[1]
        return self.next()

    def next(self) -> State:
        if self.should_go_back_to_sync or not self.timing.in_cycle():
            self.should_go_back_to_sync = False
            return State.SYNCHRONIZATION
        else:
            return State.TASK if self.timing.in_taskslot(self.slot_assignment.pure_send_list) else State.LISTEN
