import unittest

from src.messages.synchronizationMessage import SynchronisationMessage


class TestSynchronizationMessage(unittest.TestCase):
    def setUp(self):
        self.synchronization_message = SynchronisationMessage(0)

    def test_init(self):
        self.assertIsNotNone(self.synchronization_message)


if __name__ == "__main__":
    unittest.main()
