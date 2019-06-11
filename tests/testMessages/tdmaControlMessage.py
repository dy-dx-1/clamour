import unittest

from src.messages.tdmaControlMessage import TDMAControlMessage


class TestControlMessage(unittest.TestCase):
    def setUp(self):
        self.control_message = TDMAControlMessage(0)

    def test_init(self):
        self.assertIsNotNone(self.control_message)

    def test_equals(self):
        # Case 1:
        message_a = TDMAControlMessage(1)
        self.assertTrue(self.control_message == message_a)

        # Case 2:
        message_b = TDMAControlMessage(0, 5, 5)
        self.assertFalse(self.control_message == message_b)


if __name__ == "__main__":
    unittest.main()
