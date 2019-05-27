from .timing import NB_TASK_SLOTS

class SlotAssignment():
    def __init__(self):
        self.block = [-1] * NB_TASK_SLOTS
        self.non_block = []
        self.send_list = [-1] * NB_TASK_SLOTS
        self.pure_send_list = []
        self.receive_list = [-1] * NB_TASK_SLOTS
        self.free_slots = NB_TASK_SLOTS
        self.subpriority_slots = []

    def update_free_slots(self):
        self.non_block.clear()
        self.subpriority_slots.clear()
        self.free_slots = 0

        for x in range(NB_TASK_SLOTS):
            if self.send_list[x] != -1 or self.receive_list[x] != -1:
                self.block[x] = 1
            else:
                self.non_block.append(x)
                if self.send_list[x] == -2:
                    self.subpriority_slots.append(0)
                self.free_slots += 1
