import unittest

from src.messages.uwbMessage import UWBCommunicationMessage, UWBSynchronizationMessage, UWBTDMAMessage, InvalidValueException


class TestUWBCommunicationMessage(unittest.TestCase):
    def setUp(self):
        self.communication_message = UWBCommunicationMessage()
        self.invalid_communication_message = UWBCommunicationMessage()

    def test_init(self):
        self.assertIsNotNone(self.communication_message)
        self.assertIsNotNone(self.invalid_communication_message)

    def test_encode(self):
        # Case 1:
        self.communication_message.com_x_pos = 1
        self.communication_message.com_y_pos = 1
        self.communication_message.com_z_pos = 1
        self.communication_message.com_confidence = 1
        self.communication_message.encode()

        expected_result = (3 << 30) + (1 << 26) + (1 << 16) + (1 << 6) + 1
        self.assertEqual(self.communication_message.data, expected_result)

        # Case 2:
        with self.assertRaises(InvalidValueException):
            self.invalid_communication_message.encode()
        
    def test_decode(self):
        self.communication_message.com_x_pos = 1
        self.communication_message.com_y_pos = 1
        self.communication_message.com_z_pos = 1
        self.communication_message.com_confidence = 1
        self.communication_message.encode()

        self.communication_message.decode()

        self.assertEqual(self.communication_message.com_x_pos, 1)
        self.assertEqual(self.communication_message.com_y_pos, 1)
        self.assertEqual(self.communication_message.com_z_pos, 1)
        self.assertEqual(self.communication_message.com_confidence, 1)


class TestUWBSynchronizationMessage(unittest.TestCase):
    def setUp(self):
        self.synchronization_message = UWBSynchronizationMessage()
        self.invald_synchronization_message = UWBSynchronizationMessage()

    def test_init(self):
        self.assertIsNotNone(self.synchronization_message)
        self.assertIsNotNone(self.invald_synchronization_message)

    def test_encode(self):
        # Case 1:
        self.synchronization_message.synchronized_clock = 10
        self.synchronization_message.encode()
        self.assertEqual(self.synchronization_message.data, (1 << 30) + 10)

        # Case 2:
        with self.assertRaises(InvalidValueException):
            self.invald_synchronization_message.encode()

    def test_decode(self):
        self.synchronization_message.synchronized_clock = 10
        self.synchronization_message.encode()
        self.synchronization_message.decode()
        self.assertEqual(self.synchronization_message.synchronized_clock, 10)


class TestUWBTDMAMessage(unittest.TestCase):
    def setUp(self):
        self.tdma_message_a = UWBTDMAMessage(slot=2, code=2)
        self.tdma_message_b = UWBTDMAMessage(slot=3, code=-1)
        self.invalid_tdma_message = UWBTDMAMessage()

    def test_init(self):
        self.assertIsNotNone(self.tdma_message_a)
        self.assertIsNotNone(self.tdma_message_b)
        self.assertIsNotNone(self.invalid_tdma_message)

    def test_encode(self):
        # Case 1:
        self.tdma_message_a.encode()
        self.assertEqual(self.tdma_message_a.code, 2)
        self.assertEqual(self.tdma_message_a.data, (2 << 30) + (2 << 15) + 2)

        # Case 2:
        self.tdma_message_b.encode()
        self.assertEqual(self.tdma_message_b.code, 16384 + 1)
        self.assertEqual(self.tdma_message_b.data, (2 << 30) + (3 << 15) + 16384 + 1)

        # Case 3:
        with self.assertRaises(InvalidValueException):
            self.invalid_tdma_message.encode()

    def test_decode(self):
        # Case 1:
        self.tdma_message_a.encode()
        self.tdma_message_a.decode()
        self.assertEqual(self.tdma_message_a.slot, 2)
        self.assertEqual(self.tdma_message_a.code, 2)

        # Case 2:
        self.tdma_message_b.encode()
        self.tdma_message_b.decode()
        self.assertEqual(self.tdma_message_b.slot, 3)
        self.assertEqual(self.tdma_message_b.code, -1)

    def test_equals(self):
        # Case 1:
        self.assertFalse(self.tdma_message_a == self.tdma_message_b)

        # Case 2:
        self.assertTrue(self.tdma_message_a == UWBTDMAMessage(slot=2, code=2))


if __name__ == "__main__":
    unittest.main()
