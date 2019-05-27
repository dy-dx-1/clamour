from time import perf_counter


class LogicalClock():
    def __init__(self, logicalRate=0, offset=0):
        self.offset = offset
        self.logicalRate = logicalRate
        self.last_hardware_time = perf_counter()
        self.clock = 0

    def get_updated_clock(self) -> int:
        self.clock += ((perf_counter() - self.last_hardware_time) * (self.logicalRate + 1))
        self.last_hardware_time = perf_counter()
        return self.clock

    def correct_logical_offset(self, correction: int) -> None:
        self.clock += correction

    def reset_logical_rate(self) -> None:
        self.logicalRate = 0
