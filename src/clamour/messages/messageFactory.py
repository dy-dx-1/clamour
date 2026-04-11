from .types import MessageType
from .uwbMessage import (UWBMessage, UWBSynchronizationMessage, UWBTDMAMessage, UWBTopologyMessage)
import struct

CUSTOM_MESSAGE_SIGNATURE = 0xAA
TYPE_A_BIT_MASK = 0x80000000
TYPE_B_BIT_MASK = 0xC0000000


class MessageFactory:
    @staticmethod
    def create(sender_id: int, raw_data: bytes) -> UWBMessage | None:
        """For SYNC messages, only one bit (MSB) is used for type. The second one is used as an OK.
        For  TDMA and topology, the 2 MSB are used for type. The difference in nb bits used for type
        explains the 2 message_type variables.
        """
        if len(raw_data) < 5:
            return None  # not a valid BI message, this is expected to be used in messenger.py where control messages follow "BI" format
        
        if not MessageFactory.is_custom_message(raw_data[0]): # check  is_custom_message first
            return None 
        
        header, message_data = struct.unpack('<BI', raw_data[:5])
        message_type_a = (message_data & TYPE_A_BIT_MASK) >> 31
        message_type_b = (message_data & TYPE_B_BIT_MASK) >> 30
            
        if message_type_a == MessageType.SYNC:
            return UWBSynchronizationMessage(sender_id, message_type_a, message_data)
        elif message_type_b == MessageType.TDMA:
            return UWBTDMAMessage(sender_id, message_type_b, message_data)
        elif message_type_b == MessageType.TOPOLOGY:
            return UWBTopologyMessage(sender_id, message_type_b, message_data)
        else:
            return None 
        
    @staticmethod
    def is_custom_message(data:int) -> bool:
        return data == CUSTOM_MESSAGE_SIGNATURE
