from .timing import tdmaNumSlots

class SlotAssignment():
    def __init__(self):
        self.block = []
        self.non_block = []
        self.send_list = []
        self.pure_send_list = []
        self.receive_list = []
        self.free_slots = []
        self.subpriority_slots = []

    def update_free_slots(self):
        #TODO: Refactor after MessageHandler is done
        self.nonBlock = []
        self.subpriority_slots = []
        freeSlotCount = 0

        for x in range(tdmaNumSlots):
            if self.send_list[x] != -1 or self.receive_list[x] != -1:
                self.block[x] = 1
            else:
                self.nonBlock.append(x)
                if self.send_list[x] == -2:
                    self.subpriority_slots.append(0)
                freeSlotCount += 1

        self.free_slots = freeSlotCount #The amonunt of free slots, one more than slot id.
