from math import floor
from ..logicalClock import LogicalClock

# All timing constants and LogicalClock values are milliseconds.
COMMUNICATION_DELAY = 5
MAX_RANGING_DELAY = 15
SLOT_FOR_RESET = 30
THRESHOLD_SYNCTIME = 15

# Phase 1: nodes exchange clock information before scheduling begins.
SYNCHRONIZATION_PERIOD = 5000

# Phase 2: distributed scheduling.
# A scheduling round has this many control-message slots.  A tag owns slot
# (tag_id & 0xFF), so this count must exceed every deployed tag's low-byte ID.
# Low-byte IDs must also be unique, otherwise two tags transmit together.
#
# This sets the period at which a node may propose a task slot.  It does not
# directly set the scheduling-phase duration; that is calculated at runtime
# from the discovered neighbour count in Timing.update_scheduling_window().
SCHEDULING_SLOT_COUNT = 20
SCHEDULING_SLOT_DURATION = 30
SCHEDULING_ROUND_DURATION = SCHEDULING_SLOT_COUNT * SCHEDULING_SLOT_DURATION

# The scheduling window has this many 30 ms units per discovered neighbour.
# It is a duration multiplier, not necessarily the number of complete
# scheduling rounds: round count also depends on SCHEDULING_SLOT_COUNT.
SCHEDULING_WINDOW_MULTIPLIER = 200

# Phase 3: localization work. A task frame contains task slots; a TDMA cycle
# contains several frames, then nodes return to synchronization.
TASK_SLOT_DURATION = 25
NB_TASK_SLOTS = 40
TASK_FRAME_DURATION = TASK_SLOT_DURATION * NB_TASK_SLOTS
NB_TASK_FRAMES_PER_CYCLE = 20
TASK_PHASE_DURATION = TASK_FRAME_DURATION * NB_TASK_FRAMES_PER_CYCLE

# Maximum configured cycle estimate, retained for get_full_cycle_duration().
# Runtime scheduling is normally shorter/longer because it is neighbour-based.
MAX_SCHEDULING_WINDOW_DURATION = SCHEDULING_SLOT_DURATION * SCHEDULING_SLOT_COUNT * SCHEDULING_WINDOW_MULTIPLIER
FULL_CYCLE_DURATION = SYNCHRONIZATION_PERIOD + MAX_SCHEDULING_WINDOW_DURATION + TASK_PHASE_DURATION


class Timing:
    def __init__(self):
        self.synchronization_offset_mean = 20
        self.clock_differential_stat = []
        self.logical_clock = LogicalClock()
        # This clock begins when scheduling ends. Sync and scheduling time are
        # deliberately excluded from task-frame and task-slot calculations.
        self.current_time_in_cycle = 0
        self.synchronized = False
        self.current_slot_id = -1
        self.frame_id = 0
        self.cycle_start = self.logical_clock.clock
        # Shared logical timestamp recorded on entry to scheduling.
        self.sync_timestamp = self.logical_clock.clock
        self.hist_list = []
        # Replaced after synchronization using the actual neighbour count.
        self.scheduling_window_duration = MAX_SCHEDULING_WINDOW_DURATION
        self.task_phase_duration = TASK_PHASE_DURATION

    def get_full_cycle_duration(self):
        return FULL_CYCLE_DURATION

    def update_scheduling_window(self, neighbor_count: int):
        """Set the scheduling duration for the next TDMA cycle.

        The duration starts at sync_timestamp.  The zero-neighbour fallback
        retains the prior two-neighbour behaviour if community scheduling is
        entered without known neighbours.
        """
        effective_neighbor_count = max(neighbor_count, 2)
        self.scheduling_window_duration = (
            SCHEDULING_SLOT_DURATION
            * effective_neighbor_count
            * SCHEDULING_WINDOW_MULTIPLIER
        )

    def in_cycle(self) -> bool:
        self.update_current_time()
        return self.current_time_in_cycle < self.task_phase_duration - SLOT_FOR_RESET

    def in_taskslot(self, assigned_list) -> bool:
        self.update_current_time()
        return (self.current_slot_id in assigned_list)

    def update_current_time(self):
        self.logical_clock.update_clock()
        self.current_time_in_cycle = self.logical_clock.clock - self.cycle_start
        self.frame_id = floor(self.current_time_in_cycle / TASK_FRAME_DURATION)
        self.current_slot_id = floor((self.current_time_in_cycle % TASK_FRAME_DURATION) / TASK_SLOT_DURATION)

    def enough_time_left(self) -> bool:
        return (self.current_time_in_cycle % TASK_SLOT_DURATION) < MAX_RANGING_DELAY

    def clear_synchronization_info(self):
        self.clock_differential_stat = []
        self.synchronization_offset_mean = 20
        self.synchronized = False
