class SynchronizationMessage:
    def __init__(self, sender_id: int, clock: int = 0, neib_logical: int = 0, time_alive: int = 0):
        self.sender_id = sender_id
        self.clock = clock
        self.neib_logical = neib_logical
        self.offset = self.neib_logical - self.clock
        self.time_alive = time_alive
