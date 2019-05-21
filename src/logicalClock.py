from time import perf_counter


class LogicalClock():
    def __init__(self, logicalRate=0, offset=0):
        self.offset = offset
        self.logicalRate = logicalRate
        self.last_hardware_time = perf_counter()
        self.clock = 0

    # TODO: rename to updateClock
    def getLogicalTime(self):
        self.clock += ((perf_counter() - self.last_hardware_time) * (self.logicalRate + 1))
        self.last_hardware_time = perf_counter()
        return self.clock

    def correctLogicalOffset(self, correction):
        self.clock += correction

    def resetLogicalRate(self):
        self.logicalRate = 0
