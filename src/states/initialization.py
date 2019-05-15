from .constants import State
from .tdmaState import TDMAState

class Initialization(TDMAState):
    def __init__(self):
        pass

    def execute(self):
        pass

    def next(self) -> State:
        pass