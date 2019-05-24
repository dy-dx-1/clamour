from .constants import State
from .tdmaState import TDMAState

from interfaces import Neighborhood, SlotAssignment, Anchors, Timing
from interfaces.timing import syncTimeLen, numNodes, tdmaSchSlotLen, tdmaExcStartTime
from pypozyx import PozyxSerial

class Scheduling(TDMAState):
    def __init__(self, neighborhood: Neighborhood, slot_assignment: SlotAssignment,
                 timing: Timing, anchors: Anchors, id: int):
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.anchors = anchors
        self.timing = timing
        self.id = id

    def execute(self):
        self.neighborhood.neighbor_list = self.neighborhood.synchronized_neighbors
        self.slot_assignment.update_free_slots()

        if int(((self.timing.current_time_in_cycle - syncTimeLen) % (numNodes*tdmaSchSlotLen)) / tdmaSchSlotLen) == self.id - 99:
            message_handler.broadcast_control_message()
        else:
            message_handler.receive_message()
        
        self.slot_assignment.update_free_slots()
        self.update_pure_send_list()

        return self.next()

    def next(self) -> State:
        if self.timing.current_time_in_cycle > tdmaExcStartTime:
            return State.LISTEN
        else:
            return State.SCHEDULING

    def update_pure_send_list(self):
        self.slot_assignment.pure_send_list = [x for x in range(len(self.slot_assignment.send_list)) 
                                                 if self.slot_assignment.send_list[x] not in [-1, -2]]
