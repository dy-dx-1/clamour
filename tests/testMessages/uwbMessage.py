import unittest

from src.messages.uwbMessage import UWBCommunicationMessage, UWBSynchronizationMessage, UWBTDMAMessage


class TestUWBCommunicationMessage(unittest.TestCase):
    def setUp(self):
        self.communication_message = UWBCommunicationMessage()

    def test_init(self):
        self.assertIsNotNone(self.communication_message)

    def test_encode(self):
        pass

    def test_decode(self):
        pass


class TestUWBSynchronizationMessage(unittest.TestCase):
    def setUp(self):
        self.synchronization_message = UWBSynchronizationMessage()

    def test_init(self):
        self.assertIsNotNone(self.synchronization_message)

    def test_encode(self):
        self.synchronization_message.encode()
        self.assertEqual(self.synchronization_message.data, 1 << 30 - 1)

    def test_decode(self):
        self.synchronization_message.decode()
        self.assertEqual(self.synchronization_message.synchronized_clock, -1)


class TestUWBTDMAMessage(unittest.TestCase):
    def setUp(self):
        self.tdma_message = UWBTDMAMessage()

    def test_init(self):
        self.assertIsNotNone(self.tdma_message)

    def test_encode(self):
        pass

    def test_decode(self):
        pass


if __name__ == "__main__":
    unittest.main()
