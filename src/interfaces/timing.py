class Timing():
    def __init__(self, synchronization_offset_mean, clock_differential,
                clock_differential_dev, clock_differential_stat, logical_clock,
                received_frequency_sample, jumped, current_time_in_cycle,
                synchronized, current_slot_id, frame_id):
        self.synchronization_offset_mean = synchronization_offset_mean
        self.clock_differential = clock_differential
        self.clock_differential_dev = clock_differential_dev
        self.clock_differential_stat = clock_differential_stat
        self.logical_clock = logical_clock
        self.received_frequency_sample = received_frequency_sample
        self.jumped = jumped
        self.current_time_in_cycle = current_time_in_cycle
        self.synchronized = synchronized
        self.current_slot_id = current_slot_id
        self.frame_id = frame_id
