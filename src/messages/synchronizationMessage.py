from time import perf_counter


class SynchronisationMessage:
    def __init__(self, sender_id: int, clock: int=0, neib_logical: int=0, neib_rate: int=0, relative_rate: int=0):
        self.sender_id = sender_id
        self.node_hardware = perf_counter()
        self.clock = clock
        self.neib_logical = neib_logical
        self.neib_rate = neib_rate
        self.relativeRate = relative_rate
        self.offset = self.neib_logical - self.clock
