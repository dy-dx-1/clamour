from .types import MessageType
from .uwbMessage import (UWBCommunicationMessage, UWBMessage, UWBSynchronizationMessage, UWBTDMAMessage)


class InvalidMessageTypeException(Exception):
    pass


class MessageFactory:
    @staticmethod
    def create(sender_id: int, message_data: int=0) -> UWBMessage:
        type_bit_mask = 0b11000000000000000000000000000000
        message_type = (message_data & type_bit_mask) >> 30

        if message_type == MessageType.SYNC:
            return UWBSynchronizationMessage(sender_id, message_type, message_data)
        elif message_type == MessageType.TDMA:
            return UWBTDMAMessage(sender_id, message_type, message_data)
        elif message_type == MessageType.COMM:
            return UWBCommunicationMessage(sender_id, message_type, message_data)
        else:
            raise InvalidMessageTypeException("The message type in the received data does not match any known message type.")
