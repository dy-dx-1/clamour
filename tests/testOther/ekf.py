import unittest

from src.ekf import CustomEKF, Coordinates


class TestUWBCommunicationMessage(unittest.TestCase):
    def setUp(self):
        self.ekf = CustomEKF(Coordinates())

    def test_init(self):
        self.assertIsNotNone(self.ekf)


if __name__ == "__main__":
    unittest.main()
