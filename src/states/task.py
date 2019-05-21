from .constants import State
from .tdmaState import TDMAState

class Task(TDMAState):
    def __init__(self, timing, anchors):
        self.timing = timing
        self.anchors = anchors

    def execute(self):
        pass
        
    def next(self) -> State:
        pass
