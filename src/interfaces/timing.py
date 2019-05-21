class Timing():
    def __init__(self):
        self.synchronization_offset_mean = 0
        self.clock_differential = 0
        self.clock_differential_dev = 0
        self.clock_differential_stat = 0
        self.logical_clock = object()
        self.received_frequency_sample = 0
        self.jumped = False
        self.current_time_in_cycle = 0
        self.synchronized = False
        self.current_slot_id = 0
        self.frame_id = 0
