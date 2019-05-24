class TDMAControlMessage():
    def __init__(self, id, slot=-1, code=-1):
        self.id = id
        self.slot = slot
        self.code = code

    def __equals__(self, other: TDMAControlMessage):
        return self.slot == other.slot and self.code == other.code
