from time import perf_counter


class SynchronisationMessage():
    def __init__(self, sender_id: int, clock:int=0, neibLogical:int=0, neibRate:int=0, relativeRate:int=0):
        self.sender_id = sender_id
        self.nodeHardware = perf_counter()
        self.clock = clock
        self.neibLogical = neibLogical
        self.neibRate = neibRate
        self.relativeRate = relativeRate
        self.offset = self.neibLogical - self.clock
