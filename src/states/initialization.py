from .constants import State
from .tdmaState import TDMAState

class Initialization(TDMAState):
    def __init__(self, neighborhood, message_box, anchors, id):
        self.neighborhood = neighborhood
        self.message_box = message_box
        self.anchors = anchors
        self.id = id

    def execute(self):
        pass

    def next(self) -> State:
        pass