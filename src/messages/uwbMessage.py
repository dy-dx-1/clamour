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
    def __init__(self, sender_id: int, message_type: MessageType = MessageType.SYNC,
                 data: int = 0, synchronized: bool = False):
        super(UWBSynchronizationMessage, self).__init__(sender_id, message_type, data)
        self.CLOCK_MASK = 0x3FFFFFFF
        self.SYNC_MASK = 0x40000000
        self.synchronized_clock = -1
        self.synchronized = synchronized

    def decode(self):
        self.synchronized_clock = (self.data & self.CLOCK_MASK) << 2
        self.synchronized = bool(self.data & self.SYNC_MASK)

    def encode(self):
        if self.synchronized_clock.value < 0:
            raise InvalidValueException("One of the attributes of the message could not be encoded, because it is negative")

        self.data = int32((bool(self.message_type) << 31) | (self.synchronized << 30) | (self.synchronized_clock.value >> 2)).value

    def __hash__(self):
        return hash(str(self.sender_id) + str(self.data))

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()

    def __repr__(self):
        return "Type: " + str(self.message_type) + "clock: " + str(self.synchronized_clock.value) + "synced: " + str(self.synchronized)


class UWBTDMAMessage(UWBMessage):
    def __init__(self, sender_id: int, message_type: MessageType = MessageType.TDMA,
                 data: int = 0, slot: int = -1, code: int = -5):
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
        
        self.data = int32((self.message_type << 30) | (self.slot << 15) | self.code).value

    def __hash__(self):
        return hash(str(self.sender_id) + str(self.data))

    def __repr__(self):
        return "Type: " + str(self.message_type) + "slot: " + str(self.slot) + "code: " + str(self.code)

    def __eq__(self, other: 'UWBTDMAMessage'):
        return self.__hash__() == other.__hash__()


class UWBTopologyMessage(UWBMessage):
    def __init__(self, sender_id: int, message_type: MessageType = MessageType.TDMA,
                 data: int = 0, topology: list = None):
        super(UWBTopologyMessage, self).__init__(sender_id, message_type, data)
        self.TAG_ID_MASK = 0xFF
        self.TAG_BASE_VALUE = 0x2000
        self.NB_BITS = 30  # Only the last 30 bits are used for data, because the 2 MSB are for message type
        self.data = data
        self.neighborhood = topology if topology is not None else []
        self.bitwise_neighbors = 0

    def decode(self):
        self.neighborhood = [(i + 1 | self.TAG_BASE_VALUE) for i in range(self.NB_BITS) if (self.data >> i) & 0x1 == 1]

    def encode(self):
        self.calculate_bitwise_neighbors()
        self.data = int32((self.message_type << 30) | self.bitwise_neighbors).value

    def calculate_bitwise_neighbors(self) -> None:
        self.bitwise_neighbors = 0
        for neighbor in self.neighborhood:
            self.bitwise_neighbors |= 1 << ((neighbor & self.TAG_ID_MASK) - 1)

    def __eq__(self, other: 'UWBTDMAMessage'):
        return self.__hash__() == other.__hash__()

    def __hash__(self):
        return hash(str(self.sender_id) + str(self.data))
