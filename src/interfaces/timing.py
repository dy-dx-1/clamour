import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from logicalClock import LogicalClock

SECONDS_TO_MILLISECONDS = 1000

COMMUNICATION_DELAY = 5
MAX_RANGING_DELAY = 15
SLOT_FOR_RESET = 30
THRESHOLD_SYNCTIME = 15

SYNCHRONIZATION_PERIOD = 7500
NB_NODES = 35
SCHEDULING_SLOT_DURATION = 30

NB_SCHEDULING_CYCLES = 30
TASK_SLOT_DURATION = 30
NB_TASK_SLOTS = 35
NB_FULL_CYCLES = 1000

# |<---------------------------------FULL_CYCLE_DURATION------------------------------------->|
# |<----------TASK_START_TIME------>|  + |-------FRAME_DURATION * NB_FULL_CYCLES------------->|

TASK_START_TIME = SYNCHRONIZATION_PERIOD + SCHEDULING_SLOT_DURATION * NB_NODES * NB_SCHEDULING_CYCLES / 2
FRAME_DURATION = TASK_SLOT_DURATION * NB_TASK_SLOTS
FULL_CYCLE_DURATION = TASK_START_TIME + FRAME_DURATION * NB_FULL_CYCLES


class Timing:
    def __init__(self):
        self.synchronization_offset_mean = 20
        self.clock_differential_stat = []
        self.logical_clock = LogicalClock()
        self.current_time_in_cycle = 0
        self.synchronized = False
        self.current_slot_id = -1
        self.frame_id = 0

    def update_current_time(self):
        self.logical_clock.update_clock()
        self.current_time_in_cycle = int(self.logical_clock.clock) % FULL_CYCLE_DURATION

    def update_frame_id(self):
        self.frame_id = int((self.current_time_in_cycle - TASK_START_TIME) / FRAME_DURATION)

    def update_slot_id(self):
        self.current_slot_id = int(((self.current_time_in_cycle - TASK_START_TIME) % FRAME_DURATION) / TASK_SLOT_DURATION)

    def enough_time_left(self) -> bool:
        print(((self.current_time_in_cycle - TASK_START_TIME) % FRAME_DURATION) / NB_TASK_SLOTS)
        return ((self.current_time_in_cycle - TASK_START_TIME) % FRAME_DURATION) / NB_TASK_SLOTS < MAX_RANGING_DELAY

    def clear_synchronization_info(self):
        self.clock_differential_stat = []
        self.synchronization_offset_mean = 20
        self.synchronized = False
