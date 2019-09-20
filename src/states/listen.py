from time import sleep

from interfaces import SlotAssignment, Timing, Neighborhood
from interfaces.timing import FULL_CYCLE_DURATION, SLOT_FOR_RESET, Timing, TASK_START_TIME
from messenger import Messenger

from .constants import State
from .tdmaState import TDMAState


class Listen(TDMAState):
    def __init__(self, slot_assignment: SlotAssignment, timing: Timing, 
                 messenger: Messenger, neighborhood: Neighborhood):
        self.slot_assignment = slot_assignment
        self.timing = timing
        self.messenger = messenger
        self.neighborhood = neighborhood

    def execute(self) -> State:
        next_state = self.next()
        
        # if next_state == State.LISTEN:
        #     sleep(0.002)  # Milliseconds

        return next_state

    def next(self) -> State:
        self.timing.update_current_time()
        if self.timing.current_time_in_cycle < FULL_CYCLE_DURATION - SLOT_FOR_RESET:
            if self.timing.current_slot_id in self.slot_assignment.pure_send_list:
                return State.TASK
            else:
                return State.LISTEN
        else:
            return State.SYNCHRONIZATION
