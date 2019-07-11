# TODO: Rewrite these tests for new EKF

import unittest

from src.pedometer.ekf import CustomEKF, Coordinates
from numpy import array


class TestUWBCommunicationMessage(unittest.TestCase):
    def setUp(self):
        self.ekf = CustomEKF(Coordinates(), 0)

    def test_init(self):
        self.assertIsNotNone(self.ekf)

    def test_set_qf(self):
        # Test shape of Q and F
        self.assertEqual(self.ekf.Q.shape, (9, 9))
        self.assertEqual(self.ekf.F.shape, (9, 9))

        # Test some elements of Q and F
        self.assertEqual(self.ekf.Q[1][1], 0.01)
        self.assertEqual(self.ekf.F[0][0], 1)
        self.assertEqual(self.ekf.F[0][1], 0.1)
        self.assertAlmostEqual(self.ekf.F[0][2], 0.01)
        self.assertEqual(self.ekf.F[1][1], 1)

    def test_hx_of_position(self):
        self.assertEqual(self.ekf.hx_trilateration(self.ekf.x).shape[0], (6))

    def test_h_of_range(self):
        # Verify if error in function (deltas indices)
        self.assertTrue(False)

        # Case 1:
        x = array([0, 1, 2, 3, 4, 5, 6, 7, 8])
        neighbor_positions = array([[0, 1, 2], [2, 3, 4], [5, 6, 7], [9, 10, 11]])
        result = self.ekf.h_ranging(x, neighbor_positions)
        
        self.assertEqual(result.shape, (6, 9))
        self.assertEqual(result[0][0], 0)
        self.assertEqual(result[1][0], 0)

        # Case 2:
        x = array([0, 1, 2, 3, 4, 5, 6, 7, 8])
        neighbor_positions = array([[0, 1, 2]])
        result = self.ekf.h_ranging(x, neighbor_positions)
        
        self.assertEqual(result.shape, (6, 9))
        self.assertEqual(result[0][0], 0)


if __name__ == "__main__":
    unittest.main()
