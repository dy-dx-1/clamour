from .types import MessageType


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
        self.CLOCK_MASK = 0b111111111111111111111111111111
        self.SYNC_MASK = 0b01000000000000000000000000000000
        self.synchronized_clock = -1
        self.synchronization_ok = synchronized

    def decode(self):
        self.synchronized_clock = self.data & self.CLOCK_MASK
        self.synchronization_ok = bool(self.data & self.SYNC_MASK)

    def encode(self):
        if self.synchronized_clock < 0:
            print("Clock is negative. Have positive feelings instead.", self.synchronized_clock)
            raise InvalidValueException("One of the attributes of the message could not be encoded, because it is negative")
        
        self.data = (self.message_type << 31) + (self.synchronization_ok << 30) + (self.synchronized_clock >> 2)

    def __repr__(self):
        print("Type:", self.message_type, "Clock:", self.synchronized_clock, "Synced:", self.synchronization_ok)


class UWBTDMAMessage(UWBMessage):
    def __init__(self, sender_id: int, message_type: MessageType=MessageType.TDMA, data: int=0, slot: int=-1, code: int=-5):
        super(UWBTDMAMessage, self).__init__(sender_id, message_type, data)
        self.SLOT_MASK = 0b111111111111111000000000000000
        self.TDMA_CODE_MASK = 0b111111111111111
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
        
        self.data = (self.message_type << 30) + (self.slot << 15) + self.code

    def __repr__(self):
        print(" Type ", self.message_type, " slot ", self.slot, " code ", self.code)

    def __eq__(self, other: 'UWBTDMAMessage'):
        return self.code == other.code and self.slot == other.slot
