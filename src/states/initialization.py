from .constants import State
from .tdmaState import TDMAState

class Initialization(TDMAState):
    def __init__(self, neighborhood):
        self.neighborhood = neighborhood

    def execute(self):
        pass

    def next(self) -> State:
        pass