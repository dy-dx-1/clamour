import os
import sys
from math import floor
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from logicalClock import LogicalClock

SECONDS_TO_MILLISECONDS = 1000

COMMUNICATION_DELAY = 5
MAX_RANGING_DELAY = 15
SLOT_FOR_RESET = 30
THRESHOLD_SYNCTIME = 15

SYNCHRONIZATION_PERIOD = 5000
NB_NODES = 40 # The number should bigger than the maximum ID sequence (not the practical amount of nodes involved. eg. ID 8228 (10000000100100 & 0xFF) UWB tag send message on slot 36. So the number should bigger than 36.)
SCHEDULING_SLOT_DURATION = 30
NB_SCHEDULING_CYCLES = 200

TASK_SLOT_DURATION = 25
NB_TASK_SLOTS = 40
NB_FULL_CYCLES = 20

# |<---------------------------------FULL_CYCLE_DURATION------------------------------------->|
# |<----------TASK_START_TIME------>|  + |-------FRAME_DURATION * NB_FULL_CYCLES------------->|

TASK_START_TIME = SYNCHRONIZATION_PERIOD + SCHEDULING_SLOT_DURATION * NB_NODES * NB_SCHEDULING_CYCLES
FRAME_DURATION = TASK_SLOT_DURATION * NB_TASK_SLOTS
FULL_CYCLE_DURATION = TASK_START_TIME + FRAME_DURATION * NB_FULL_CYCLES


class Timing:
    def __init__(self):
        self.synchronization_offset_mean = 20
        self.clock_differential_stat = []
        self.logical_clock = LogicalClock()
        self.current_time_in_cycle = 0 # This is counted for task cycle. Updated by substute the timestampt(self.cycle_start) entering task phase
        self.synchronized = False
        self.current_slot_id = -1
        self.frame_id = 0
        self.cycle_start = self.logical_clock.clock
        self.sync_timestamp = self.logical_clock.clock # This is the logical timestampt after sync, which means is the same clock value for all neighbors. Used to count the condition of scheduling.
        self.hist_list = []
        self.task_start_time = TASK_START_TIME - SYNCHRONIZATION_PERIOD # Exclude the time used for sync, because 1) do not know exact time used for sync. 2 system is unite after sync.
        self.task_process_time = FRAME_DURATION* NB_FULL_CYCLES

    def get_full_cycle_duration(self):
        return FULL_CYCLE_DURATION

    def update_task_start_time(self, nb):
        if nb == 0: nb = 2
        self.task_start_time = SCHEDULING_SLOT_DURATION * nb * NB_SCHEDULING_CYCLES
        pass

    def in_cycle(self) -> bool:
        self.update_current_time()
        return (self.current_time_in_cycle < self.task_process_time - SLOT_FOR_RESET)

    def in_taskslot(self, assigned_list) -> bool:
        self.update_current_time()
        return (self.current_slot_id in assigned_list)

    def update_current_time(self):
        self.logical_clock.update_clock()
        self.current_time_in_cycle = self.logical_clock.clock - self.cycle_start
        self.frame_id = floor(self.current_time_in_cycle / FRAME_DURATION)
        self.current_slot_id = floor((self.current_time_in_cycle % FRAME_DURATION) / TASK_SLOT_DURATION)

    def enough_time_left(self) -> bool:
        return (self.current_time_in_cycle % TASK_SLOT_DURATION) < MAX_RANGING_DELAY

    def clear_synchronization_info(self):
        self.clock_differential_stat = []
        self.synchronization_offset_mean = 20
        self.synchronized = False
