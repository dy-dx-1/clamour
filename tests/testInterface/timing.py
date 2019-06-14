import unittest

from src.interfaces.timing import Timing, TASK_START_TIME, FRAME_DURATION, TASK_SLOT_DURATION


class TestTiming(unittest.TestCase):
    def setUp(self):
        self.timing = Timing()
 
    def test_init(self):
        self.assertEqual(self.timing.synchronization_offset_mean, 20)
        self.assertEqual(self.timing.clock_differential, [0])
        self.assertEqual(self.timing.clock_differential_dev, 10)
        self.assertEqual(self.timing.clock_differential_stat, [])
        self.assertIsNotNone(self.timing.logical_clock)
        self.assertEqual(self.timing.received_frequency_sample, [])
        self.assertEqual(self.timing.current_time_in_cycle, 0)
        self.assertFalse(self.timing.synchronized)
        self.assertEqual(self.timing.current_slot_id, -1)
        self.assertEqual(self.timing.frame_id, 0)

    def test_update_frame_id(self):
        self.timing.update_frame_id()
        self.assertEqual(self.timing.frame_id, int(-TASK_START_TIME / FRAME_DURATION))


    def test_update_slot_id(self):
        self.timing.update_slot_id()
        self.assertEqual(self.timing.current_slot_id, int((-TASK_START_TIME % FRAME_DURATION) / FRAME_DURATION))


if __name__ == "__main__":
    unittest.main()
