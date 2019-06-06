import unittest

from src.messages.uwbMessage import UWBCommunicationMessage, UWBSynchronizationMessage, UWBTDMAMessage


class TestUWBCommunicationMessage(unittest.TestCase):
    def setUp(self):
        self.communication_message = UWBCommunicationMessage()

    def test_init(self):
        self.assertIsNotNone(self.communication_message)

    def test_encode(self):
        self.communication_message.encode()
        expected_result = (3 << 30) + (-1 << 26) + (-1 << 16) + (-1 << 6) - 1
        self.assertEqual(self.communication_message.data, expected_result)

    def test_decode(self):
        self.communication_message.decode()
        self.assertEqual(self.communication_message.com_x_pos, -1)
        self.assertEqual(self.communication_message.com_y_pos, -1)
        self.assertEqual(self.communication_message.com_z_pos, -1)
        self.assertEqual(self.communication_message.com_confidence, -1)


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
