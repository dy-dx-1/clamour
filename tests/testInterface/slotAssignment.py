import unittest

from src.interfaces.slotAssignment import SlotAssignment, NB_TASK_SLOTS


class TestSlotAssignment(unittest.TestCase):
    def setUp(self):
        self.slots = SlotAssignment()
 
    def test_init(self):
        self.assertEqual(self.slots.block, [-1] * NB_TASK_SLOTS)
        self.assertEqual(self.slots.non_block, [])
        self.assertEqual(self.slots.send_list, [-1] * NB_TASK_SLOTS)
        self.assertEqual(self.slots.pure_send_list, [])
        self.assertEqual(self.slots.block, [-1] * NB_TASK_SLOTS)
        self.assertEqual(self.slots.free_slots, NB_TASK_SLOTS)
        self.assertEqual(self.slots.subpriority_slots, [])

    def test_update_free_slots(self):
        # Case 1:
        self.slots.send_list = [-1] * NB_TASK_SLOTS
        self.slots.receive_list = [-1] * NB_TASK_SLOTS
        
        self.slots.update_free_slots()

        self.assertEqual(self.slots.block, [-1] * NB_TASK_SLOTS)
        self.assertEqual(self.slots.non_block, [i for i in range(NB_TASK_SLOTS)])
        self.assertEqual(self.slots.subpriority_slots, [])
        self.assertEqual(self.slots.free_slots, NB_TASK_SLOTS)

        # Case 2:
        self.slots.send_list = [-1] * (NB_TASK_SLOTS - 1) + [2]
        self.slots.receive_list = [-1] * NB_TASK_SLOTS
                
        self.slots.update_free_slots()
        
        self.assertEqual(self.slots.block, [-1] * (NB_TASK_SLOTS - 1) + [1])
        self.assertEqual(self.slots.non_block, [i for i in range(NB_TASK_SLOTS - 1)])
        self.assertEqual(self.slots.subpriority_slots, [])
        self.assertEqual(self.slots.free_slots, NB_TASK_SLOTS - 1)


if __name__ == "__main__":
    unittest.main()
