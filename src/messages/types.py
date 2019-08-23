from enum import IntEnum


class MessageType(IntEnum):
    SYNC = 0
    TDMA = 1
    UNKNOWN = 2


class UpdateType(IntEnum):
    PEDOMETER = 0
    TRILATERATION = 1
    RANGING = 2
    ZERO_MOVEMENT = 3
