from logicalClock import LogicalClock

COMMUNICATION_DELAY = 0.027 # delay estimation is 0.005s.
SLOT_FOR_RESET = 30
THRESHOLD_SYNTIME=0.01

SYNCHRONIZATION_PERIOD = 20000 # Time leave for syn (ms)
NB_NODES = 12
SCHEDULING_SLOT_DURATION = 30

NB_SCHEDULING_CYCLES = 30
TASK_SLOT_DURATION = 30
NB_TASK_SLOTS = 13
NB_FULL_CYCLES = 1000

TASK_START_TIME = SYNCHRONIZATION_PERIOD + SCHEDULING_SLOT_DURATION * NB_NODES * NB_SCHEDULING_CYCLES
FULL_CYCLE_DURATION = TASK_START_TIME + TASK_SLOT_DURATION * NB_TASK_SLOTS * NB_FULL_CYCLES
FRAME_DURATION = TASK_SLOT_DURATION * NB_TASK_SLOTS

class Timing():
    def __init__(self):
        self.synchronization_offset_mean = 20
        self.clock_differential = [0]
        self.clock_differential_dev = 10
        self.clock_differential_stat = []
        self.logical_clock = LogicalClock()
        self.received_frequency_sample = []
        self.current_time_in_cycle = 0
        self.synchronized = False
        self.current_slot_id = -1
        self.frame_id = 0

    def update_frame_id(self):
        self.frame_id = int((self.current_time_in_cycle - TASK_START_TIME) / FRAME_DURATION)

    def update_slot_id(self):
        self.current_slot_id = int(((self.current_time_in_cycle - TASK_START_TIME) % FRAME_DURATION) / TASK_SLOT_DURATION)
