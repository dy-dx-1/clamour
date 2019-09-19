from .types import MessageType
from .uwbMessage import (UWBMessage, UWBSynchronizationMessage, UWBTDMAMessage)


CUSTOM_MESSAGE_SIGNATURE = 0xAA
CUSTOM_MESSAGE_MASK = 0xFF00000000
CUSTOM_MESSAGE_MASK_INV = 0x00FFFFFFFF
TYPE_BIT_MASK = 0x80000000


class InvalidMessageTypeException(Exception):
    pass


class MessageFactory:
    @staticmethod
    def create(sender_id: int, raw_data: int = 0) -> UWBMessage:
        message_type = (raw_data & TYPE_BIT_MASK) >> 31
        message_data = raw_data & CUSTOM_MESSAGE_MASK_INV

        if MessageFactory.is_custom_message(raw_data) and message_type == MessageType.SYNC:
            return UWBSynchronizationMessage(sender_id, message_type, message_data)
        elif MessageFactory.is_custom_message(raw_data) and message_type == MessageType.TDMA:
            # TODO: Add two-hop neighbor message
            return UWBTDMAMessage(sender_id, message_type, message_data)
        else:
            raise InvalidMessageTypeException("The message type in the received data does not match any known message type.")

    @staticmethod
    def is_custom_message(data: int) -> bool:
        print(hex((data & CUSTOM_MESSAGE_MASK) >> 32), hex(data))
        return (data & CUSTOM_MESSAGE_MASK) >> 32 == CUSTOM_MESSAGE_SIGNATURE
