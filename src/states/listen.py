from interfaces import SlotAssignment, Timing
from interfaces.timing import FCYCLE, SLOT_FOR_RESET, Timing, tdmaExcStartTime
from messages import UWBCommunicationMessage
from messenger import Messenger

from .constants import State
from .tdmaState import TDMAState


class Listen(TDMAState):
    def __init__(self, slot_assignment: SlotAssignment, timing: Timing, messenger: Messenger):
        self.slot_assignment = slot_assignment
        self.timing = timing
        self.messenger = messenger

    def execute(self) -> State:
        self.timing.update_frame_id()
        self.timing.update_slot_id()
        next_state = self.next()
        
        if next_state == State.LISTEN:
            self.listen_for_messages()

        return next_state

    def next(self) -> State:
        if (self.timing.current_time_in_cycle < FCYCLE - SLOT_FOR_RESET) and (self.timing.current_time_in_cycle > tdmaExcStartTime):
            if self.timing.current_slot_id in self.slot_assignment.pure_send_list:
                return State.TASK
            else:
                return State.LISTEN
        else:
            return State.SYNCHRONIZATION

    def listen_for_messages(self):
        sender_id, data, _ = self.messenger.obtain_message_from_pozyx()

        if self.messenger.is_new_message(sender_id, data):
            self.messenger.update_neighbor_dictionary()
            if isinstance(self.messenger.message_box.peek_first(), UWBCommunicationMessage):
                # TODO: when the device is in wait state, it may still perform actions that do not require interaction,
                #       such as counting steps using a pedometer
                pass
