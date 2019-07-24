from enum import IntEnum


class MessageType(IntEnum):
    UNKNOWN = 0
    SYNC = 1
    TDMA = 2
    COMM = 3


class UpdateType(IntEnum):
    PEDOMETER = 0
    TRILATERATION = 1
    RANGING = 2
    ZERO_MOVEMENT = 3
