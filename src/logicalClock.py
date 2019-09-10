from time import perf_counter


class LogicalClock:
    def __init__(self, logical_rate=0, offset=0):
        self.offset = offset
        self.logicalRate = logical_rate
        self.last_hardware_time = perf_counter()
        self.clock = 0

    def update_clock(self) -> None:
        self.clock += (perf_counter() - self.last_hardware_time) * 1000
        # print(f"Logical clock: {self.clock}")
        self.last_hardware_time = perf_counter()

    def correct_logical_offset(self, correction: int) -> None:
        print(f"Correction: {correction}")
        self.clock += -self.clock if self.clock + correction < 0 else correction

    def reset_logical_rate(self) -> None:
        self.logicalRate = 0
