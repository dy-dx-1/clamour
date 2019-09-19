from pypozyx import Data
from .types import MessageType
from .uwbMessage import (UWBMessage, UWBSynchronizationMessage, UWBTDMAMessage)


CUSTOM_MESSAGE_SIGNATURE = 0xAA
TYPE_BIT_MASK = 0x80000000


class InvalidMessageTypeException(Exception):
    pass


class MessageFactory:
    @staticmethod
    def create(sender_id: int, raw_data: Data) -> UWBMessage:
        message_type = (raw_data[1] & TYPE_BIT_MASK) >> 31
        message_data = raw_data[1]

        if MessageFactory.is_custom_message(raw_data[0]) and message_type == MessageType.SYNC:
            return UWBSynchronizationMessage(sender_id, message_type, message_data)
        elif MessageFactory.is_custom_message(raw_data[0]) and message_type == MessageType.TDMA:
            # TODO: Add two-hop neighbor message
            return UWBTDMAMessage(sender_id, message_type, message_data)
        else:
            raise InvalidMessageTypeException("The message type in the received data does not match any known message type.")

    @staticmethod
    def is_custom_message(data: int) -> bool:
        return data == CUSTOM_MESSAGE_SIGNATURE
