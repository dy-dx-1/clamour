from .constants import State
from .tdmaState import TDMAState

class Task(TDMAState):
    def __init__(self, timing):
        self.timing = timing

    def execute(self):
        pass
        
    def next(self) -> State:
        pass
