from .types import MessageType


class UWBMessage(object):
    def __init__(self, intData=0):
        TYPE_BIT_MASK =   0b11000000000000000000000000000000# 0b11000000 00000000 00000000 00000000
        self.data = intData
        self.msgType = (self.data & TYPE_BIT_MASK) >> 30

    def decode(self):
        # Is this a posssible state?
        # elif(self.msgType == 0):
        #     self.data = 0
        pass

    def encode(self):
        pass

    def __repr__(self):
        # Is this a posssible state?
        # elif(self.msgType == 0):
        #     print(" Type ", 0, " data ", self.data)
        pass


class UWBSynchronizationMessage(UWBMessage):
    def __init__(self):
        super(UWBSynchronizationMessage, self).__init__()
        self.CLOCK_MASK =      0b00111111111111111111111111111111
        self.syncClock = -1

    def decode(self):
        self.syncClock = self.data & self.CLOCK_MASK

    def encode(self):
        self.data = (self.msgType << 30) + self.syncClock

    def __repr__(self):
        print(" Type ", self.msgType, " Clock ", self.syncClock)


class UWBTDMAMessage(UWBMessage):
    def __init__(self):
        super(UWBTDMAMessage, self).__init__()
        self.SLOT_MASK =       0b00111111111111111000000000000000
        self.TDMACODE_MASK =   0b00000000000000000111111111111111
        self.tdmaSlotid = -1
        self.tdmaCode = -5

    def decode(self):
        self.tdmaSlotid = (self.data & self.SLOT_MASK) >> 15
        self.tdmaCode = self.data & self.TDMACODE_MASK
        if self.tdmaCode > 16384:
            self.tdmaCode = 16384 - self.tdmaCode

    def encode(self):
        if self.tdmaCode < 0:
            self.tdmaCode = 16384 - self.tdmaCode
        self.data = (self.msgType << 30) + (self.tdmaSlotid << 15) + self.tdmaCode

    def __repr__(self):
        print(" Type ",self.msgType, " slot ", self.tdmaSlotid, " code ", self.tdmaCode)


class UWBCommunicationMessage(UWBMessage):
    def __init__(self):
        super(UWBCommunicationMessage, self).__init__()
        self.CONFIDENCE_MASK = 0b001111
        self.XPOS_MASK =       0b0000001111111111
        self.YPOS_MASK =       0b00000000000000001111111111
        self.ZPOS_MASK =       0b00000000000000000000000000111111
        self.comXpos = -1
        self.comYpos = -1
        self.comZpos = -1
        self.comConfidence = -1
    
    def decode(self):
        self.comConfidence = (self.data >> 26) & self.CONFIDENCE_MASK
        self.comXpos = (self.data >> 16) & self.XPOS_MASK
        self.comYpos = (self.data >> 6) & self.YPOS_MASK
        self.comZpos = self.data & self.ZPOS_MASK

    def encode(self):
        self.data = (self.msgType << 30) + (self.comConfidence << 26) + (self.comXpos << 16) + (self.comYpos << 6) + self.comZpos

    def __repr__(self):
        print(" Type ", self.msgType, " confidence ", self.comConfidence," X ", self.comXpos," Y ",self.comYpos," X ",self.comZpos)
