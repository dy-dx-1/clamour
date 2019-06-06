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
        self.tdma_message_a = UWBTDMAMessage()
        self.tdma_message_b = UWBTDMAMessage(code=1)

    def test_init(self):
        self.assertIsNotNone(self.tdma_message_a)
        self.assertIsNotNone(self.tdma_message_b)

    def test_encode(self):
        # Case 1:
        self.tdma_message_a.encode()
        self.assertEqual(self.tdma_message_a.tdmaCode, 16384 + 5)
        self.assertEqual(self.tdma_message_a.data, (2 << 30) + (-1 << 30) + 16384 + 5)

        # Case 2:
        self.tdma_message_b.encode()
        self.assertEqual(self.tdma_message_b.tdmaCode, 1)
        self.assertEqual(self.tdma_message_b.data, (2 << 30) + (-1 << 30) + 1)

    def test_decode(self):
        # Case 1:
        self.tdma_message_a.decode()
        self.assertEqual(self.tdma_message_a.tdma_slot_tid, -1)
        self.assertEqual(self.tdma_message_a.tdmaCode, -5)

        # Case 2:
        self.tdma_message_b.decode()
        self.assertEqual(self.tdma_message_a.tdma_slot_tid, -1)
        self.assertEqual(self.tdma_message_a.tdmaCode, 1)

    def test_equals(self):
        # Case 1:
        self.assertFalse(self.tdma_message_a == self.tdma_message_b)

        # Case 2:
        self.assertTrue(self.tdma_message_a == UWBTDMAMessage())


if __name__ == "__main__":
    unittest.main()
