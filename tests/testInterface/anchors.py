import unittest
from src.interfaces.anchors import Anchors


class TestAnchors(unittest.TestCase):
    def setUp(self):
        self.anchors = Anchors()
 
    def test_init(self):
        self.assertEqual(self.anchors.available_anchors, [])
        self.assertEqual(self.anchors.discovery_done, False)
        
        # The two following tests will need to be re-written based on
        # wheter or not some anchors are hardcoded.
        self.assertEqual(self.anchors.anchors_list, [])
        self.assertEqual(self.anchors.anchors_dict, {})

if __name__ == "__main__":
    unittest.main()

