import unittest

from src.logicalClock import LogicalClock, perf_counter


class TestUWBCommunicationMessage(unittest.TestCase):
    def setUp(self):
        self.logical_clock = LogicalClock()

    def test_init(self):
        self.assertIsNotNone(self.logical_clock)

    def test_update_clock(self):
        old_clock = self.logical_clock.clock
        old_hardware_time = self.logical_clock.last_hardware_time
        self.logical_clock.update_clock()
        
        self.assertGreater(self.logical_clock.clock, old_clock)
        self.assertGreater(self.logical_clock.last_hardware_time, old_hardware_time)
    
    def correct_logical_offset(self):
        self.logical_clock.correct_logical_offset(10)
        self.assertEqual(self.logical_clock.clock, 10)

    def test_reset_logical_rate(self):
        self.logical_clock.reset_logical_rate()
        self.assertEqual(self.logical_clock.logicalRate, 0)


if __name__ == "__main__":
    unittest.main()
