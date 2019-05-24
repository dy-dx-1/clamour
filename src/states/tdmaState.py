from abc import ABC

from .constants import State


class TDMAState(ABC):
    def execute(self) -> State:
        pass

    def next(self) -> State:
        pass
