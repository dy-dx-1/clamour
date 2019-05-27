from .types import MessageType
from .uwbMessage import (UWBCommunicationMessage, UWBMessage,
                         UWBSynchronizationMessage, UWBTDMAMessage)


class MessageFactory():
    @staticmethod
    def create(message_data=0) -> UWBMessage:
        TYPE_BIT_MASK = 0b11000000000000000000000000000000
        message_type = (message_data & TYPE_BIT_MASK) >> 30

        if message_type == MessageType.SYNC:
            return UWBSynchronizationMessage(message_type, message_data)
        elif message_type == MessageType.TDMA:
            return UWBTDMAMessage(message_type, message_data)
        elif message_type == MessageType.COMM:
            return UWBCommunicationMessage(message_type, message_data)
