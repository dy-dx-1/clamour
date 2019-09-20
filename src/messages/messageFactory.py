from pypozyx import Data
from .types import MessageType
from .uwbMessage import (UWBMessage, UWBSynchronizationMessage, UWBTDMAMessage, UWBTopologyMessage)


CUSTOM_MESSAGE_SIGNATURE = 0xAA
TYPE_BIT_MASK = 0x80000000


class MessageFactory:
    @staticmethod
    def create(sender_id: int, raw_data: Data) -> UWBMessage:
        """For SYNC messages, only one bit (MSB) is used for type. The second one is used as an OK.
        For  TDMA and topology, the 2 MSB are used for type. The difference in nb bits used for type
        explains the 2 message_type variables.
        """
        message_type_a = (raw_data[1] & TYPE_BIT_MASK) >> 31
        message_type_b = (raw_data[1] & TYPE_BIT_MASK) >> 30
        message_data = raw_data[1]

        if MessageFactory.is_custom_message(raw_data[0]):
            if message_type_a == MessageType.SYNC:
                return UWBSynchronizationMessage(sender_id, message_type_a, message_data)
            else:
                if message_type_b == MessageType.TDMA:
                    return UWBTDMAMessage(sender_id, message_type_b, message_data)
                elif message_type_b == MessageType.TOPOLOGY:
                    return UWBTopologyMessage(sender_id, message_type_b, message_data)

    @staticmethod
    def is_custom_message(data: int) -> bool:
        return data == CUSTOM_MESSAGE_SIGNATURE
