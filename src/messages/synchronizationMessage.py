from time import perf_counter


class SynchronisationMessage():
    def __init__(self, id, nodeLogical=0, neibHardware=0, neibLogical=0, neibRate=0, relativeRate=0):
        self.id = id
        self.nodeHardware = perf_counter()
        self.nodeLogical = nodeLogical
        self.neibHardware = neibHardware
        self.neibLogical = neibLogical
        self.neibRate = neibRate
        self.relativeRate = relativeRate
        self.offset = self.neibLogical - self.nodeLogical
