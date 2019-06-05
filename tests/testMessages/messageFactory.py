import unittest

from src.messages.messageFactory import (MessageFactory, MessageType, 
                                        UWBCommunicationMessage, UWBSynchronizationMessage, 
                                        UWBTDMAMessage, InvalidMessageTypeException)


class TestMessageFactory(unittest.TestCase):
    def setUp(self):
        self.message_factory = MessageFactory()

    def test_init(self):
        self.assertIsNotNone(self.message_factory)

    def test_create(self):
        BITSHIFT = 30

        # Case 1:
        msg = self.message_factory.create(int(MessageType.COMM) << BITSHIFT)
        self.assertIsInstance(msg, UWBCommunicationMessage)

        # Case 2:
        msg = self.message_factory.create(int(MessageType.SYNC) << BITSHIFT)
        self.assertIsInstance(msg, UWBSynchronizationMessage)

        # Case 3:
        msg = self.message_factory.create(int(MessageType.TDMA) << BITSHIFT)
        self.assertIsInstance(msg, UWBTDMAMessage)

        # Case 4:
        with self.assertRaises(InvalidMessageTypeException):
            _ = self.message_factory.create(4 << BITSHIFT)


if __name__ == "__main__":
    unittest.main()
