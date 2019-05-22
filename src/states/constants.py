from enum import Enum

class State(Enum):
    INITIALIZATION: 0
    SYNCHRONIZATION: 1
    SCHEDULING: 2
    TASK: 3
    LISTEN: 4

JUMP_THRESHOLD = 0.5
