from .neighborhood import Neighborhood
from .slot_assignment import SlotAssignment
from .timing import Timing

__all__ = ["Neighborhood", "SlotAssignment", "Timing", "State", "TDMAState", "Initialization", "Listen", "Scheduling", "Synchronization", "Task"]


def __getattr__(name):
    if name in {"State", "TDMAState", "Initialization", "Listen", "Scheduling", "Synchronization", "Task"}:
        from .states import State, TDMAState, Initialization, Listen, Scheduling, Synchronization, Task
        return {
            "State": State,
            "TDMAState": TDMAState,
            "Initialization": Initialization,
            "Listen": Listen,
            "Scheduling": Scheduling,
            "Synchronization": Synchronization,
            "Task": Task,
        }[name]
    raise AttributeError(name)
