import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import unittest
import src.clamour as clamour


class TestBasicFunction(unittest.TestCase):
    def setUp(self):
        self.func = clamour.main
 
    def test_1(self):
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()
