from abc import ABC

from .constants import State


class TDMAState(ABC):
    def execute(self) -> State:
        pass

    def next(self) -> State:
        pass

def print_progress(method):
    def progress(*args, **kwargs):
        full_name = str(method.__qualname__)
        print("Executing " + full_name[:full_name.find(".")])
        return method(*args, **kwargs)
    return progress
