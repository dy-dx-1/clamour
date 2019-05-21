from .constants import State
from .tdmaState import TDMAState

class Synchronization(TDMAState):
    def __init__(self, neighborhood, slot_assignment, timing, message_box):
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.timing = timing
        self.message_box = message_box

    def execute(self):
        pass

    def next(self) -> State:
        pass