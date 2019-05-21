class SlotAssignment():
    def __init__(self, block, non_block, send_list, pure_send_list,
                receive_list, free_slots, subpriority_slots):
        self.block = block
        self.non_block = non_block
        self.send_list = send_list
        self.pure_send_list = pure_send_list
        self.receive_list = receive_list
        self.free_slots = free_slots
        self.subpriority_slots = subpriority_slots
