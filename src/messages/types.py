from enum import IntEnum


class MessageType(IntEnum):
    SYNC = 0  # 0b0(0/1)
    TDMA = 2  # 0b10
    TOPOLOGY = 3  # 0b11


class UpdateType(IntEnum):
    PEDOMETER = 0
    TRILATERATION = 1
    RANGING = 2
    ZERO_MOVEMENT = 3
    TOPOLOGY = 4
