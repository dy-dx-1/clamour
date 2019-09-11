from .types import MessageType
from ctypes import c_int32 as int32


class InvalidValueException(Exception):
    pass


class UWBMessage(object):
    def __init__(self, sender_id: int, message_type: MessageType, data: int):
        self.sender_id = sender_id
        self.message_type = message_type
        self.data = data

    def __eq__(self, other):
        return self.sender_id == other.sender_id and self.data == other.data and self.message_type == other.message_type

    def decode(self):
        pass

    def encode(self):
        pass


class UWBSynchronizationMessage(UWBMessage):
    def __init__(self, sender_id: int, message_type: MessageType=MessageType.SYNC,
                 data: int=0, synchronized: bool = False):
        super(UWBSynchronizationMessage, self).__init__(sender_id, message_type, data)
        self.CLOCK_MASK = 0x3FFFFFFF
        self.SYNC_MASK = 0x40000000
        self.synchronized_clock = -1
        self.synchronized = synchronized

    def decode(self):
        self.synchronized_clock = (self.data & self.CLOCK_MASK) << 2
        self.synchronized = bool(self.data & self.SYNC_MASK)
        print(f"Decoded with sync {self.synchronized}")

    def encode(self):
        if self.synchronized_clock.value < 0:
            raise InvalidValueException("One of the attributes of the message could not be encoded, because it is negative")

        self.data = int32((bool(self.message_type) << 31) | (self.synchronized << 30) | (self.synchronized_clock >> 2))

    def __hash__(self):
        return hash(f"{self.sender_id}{self.data}")

    def __repr__(self):
        return f"Type: {self.message_type} clock: {self.synchronized_clock} synced: {self.synchronized}"


class UWBTDMAMessage(UWBMessage):
    def __init__(self, sender_id: int, message_type: MessageType=MessageType.TDMA, data: int=0, slot: int=-1, code: int=-5):
        super(UWBTDMAMessage, self).__init__(sender_id, message_type, data)
        self.SLOT_MASK = 0x3FFF8000
        self.TDMA_CODE_MASK = 0x7FFF
        self.slot = slot
        self.code = code

    def decode(self):
        self.slot = (self.data & self.SLOT_MASK) >> 15
        self.code = self.data & self.TDMA_CODE_MASK
        if self.code > 16384:
            self.code = 16384 - self.code

    def encode(self):
        if self.slot < 0:
            raise InvalidValueException("One of the attributes of the message could not be encoded, because it is negative")

        if self.code < 0:
            self.code = 16384 - self.code
        
        self.data = int32((bool(self.message_type) << 31) | (self.slot << 15) | self.code)

    def __hash__(self):
        return hash(f"{self.sender_id}{self.data}")

    def __repr__(self):
        return f"Type {self.message_type} slot: {self.slot} code: {self.code}"

    def __eq__(self, other: 'UWBTDMAMessage'):
        return super(UWBTDMAMessage, self).__eq__(other) and self.code == other.code and self.slot == other.slot
