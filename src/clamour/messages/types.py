from enum import IntEnum


class MessageType(IntEnum):
    SYNC = 0  # 0b0(0/1)
    TDMA = 2  # 0b10
    TOPOLOGY = 3  # 0b11


class UpdateType(IntEnum):
    PEDOMETER = 0
    RANGING = 1
    TOPOLOGY = 2
    CUSTOM_POSE = 3
