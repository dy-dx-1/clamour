import unittest
from src.interfaces.neighborhood import Neighborhood


class TestNeighborhood(unittest.TestCase):
    def setUp(self):
        self.neighborhood = Neighborhood()
 
    def test_init(self):
        self.assertTrue(self.neighborhood.is_alone)
        self.assertEqual(self.neighborhood.synchronized_neighbors, [])
        self.assertEqual(self.neighborhood.current_neighbors, {})
        self.assertEqual(self.neighborhood.neighbor_synchronization_received, {})
        self.assertEqual(self.neighborhood.synchronized_active_neighbor_count, 0)
        
        
if __name__ == "__main__":
    unittest.main()
