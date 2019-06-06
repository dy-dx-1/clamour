from .types import MessageType


class InvalidValueException(Exception):
    pass


class UWBMessage(object):
    def __init__(self, message_type: MessageType, data: int):
        self.data = data
        self.message_type = message_type

    def decode(self):
        pass

    def encode(self):
        pass


class UWBSynchronizationMessage(UWBMessage):
    def __init__(self, message_type: MessageType=MessageType.SYNC, data: int=0):
        super(UWBSynchronizationMessage, self).__init__(message_type, data)
        self.CLOCK_MASK = 0b111111111111111111111111111111
        self.synchronized_clock = -1

    def decode(self):
        self.synchronized_clock = self.data & self.CLOCK_MASK

    def encode(self):
        if self.synchronized_clock < 0:
            raise InvalidValueException("One of the attributes of the message could not be encoded, because it is negative")
        
        self.data = (self.message_type << 30) + self.synchronized_clock

    def __repr__(self):
        print(" Type ", self.message_type, " Clock ", self.synchronized_clock)


class UWBTDMAMessage(UWBMessage):
    def __init__(self, message_type: MessageType=MessageType.TDMA, data: int=0, slot: int=-1, code: int=-5):
        super(UWBTDMAMessage, self).__init__(message_type, data)
        self.SLOT_MASK = 0b111111111111111000000000000000
        self.TDMA_CODE_MASK = 0b111111111111111
        self.tdma_slot_tid = slot
        self.tdmaCode = code

    def decode(self):
        self.tdma_slot_tid = (self.data & self.SLOT_MASK) >> 15
        self.tdmaCode = self.data & self.TDMA_CODE_MASK
        if self.tdmaCode > 16384:
            self.tdmaCode = 16384 - self.tdmaCode

    def encode(self):
        if self.tdma_slot_tid < 0:
            raise InvalidValueException("One of the attributes of the message could not be encoded, because it is negative")

        if self.tdmaCode < 0:
            self.tdmaCode = 16384 - self.tdmaCode
        
        self.data = (self.message_type << 30) + (self.tdma_slot_tid << 15) + self.tdmaCode

    def __repr__(self):
        print(" Type ", self.message_type, " slot ", self.tdma_slot_tid, " code ", self.tdmaCode)

    def __eq__(self, other: 'UWBTDMAMessage'):
        return self.tdmaCode == other.tdmaCode and self.tdma_slot_tid == other.tdma_slot_tid


class UWBCommunicationMessage(UWBMessage):
    def __init__(self, message_type: MessageType=MessageType.COMM, data: int=0):
        super(UWBCommunicationMessage, self).__init__(message_type, data)
        self.CONFIDENCE_MASK = 0b1111
        self.XPOS_MASK = 0b1111111111
        self.YPOS_MASK = 0b1111111111
        self.ZPOS_MASK = 0b111111
        self.com_x_pos = -1
        self.com_y_pos = -1
        self.com_z_pos = -1
        self.com_confidence = -1
    
    def decode(self):
        self.com_confidence = (self.data >> 26) & self.CONFIDENCE_MASK
        self.com_x_pos = (self.data >> 16) & self.XPOS_MASK
        self.com_y_pos = (self.data >> 6) & self.YPOS_MASK
        self.com_z_pos = self.data & self.ZPOS_MASK

    def encode(self):
        if (any([x < 0 for x in [self.com_x_pos, self.com_y_pos, self.com_z_pos, self.com_confidence]])):
            raise InvalidValueException("One of the attributes of the message could not be encoded, because it is negative")
        
        self.data = (self.message_type << 30) + (self.com_confidence << 26) +\
                    (self.com_x_pos << 16) + (self.com_y_pos << 6) + self.com_z_pos

    def __repr__(self):
        print(" Type ", self.message_type, " confidence ", self.com_confidence, " X ",
              self.com_x_pos, " Y ", self.com_y_pos, " X ", self.com_z_pos)
