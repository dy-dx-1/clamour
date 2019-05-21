from .constants import State
from .tdmaState import TDMAState

class Scheduling(TDMAState):
    def __init__(self, neighborhood, slot_assignment, anchors):
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.anchors = anchors

    def execute(self):
        pass

    def next(self) -> State:
        pass