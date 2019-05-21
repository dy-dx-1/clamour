from .constants import State
from .tdmaState import TDMAState

class Initialization(TDMAState):
    def __init__(self, neighborhood, message_box, anchors):
        self.neighborhood = neighborhood
        self.message_box = message_box
        self.anchors = anchors

    def execute(self):
        pass

    def next(self) -> State:
        pass