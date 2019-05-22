SLOT_FOR_RESET = 30

# Schedule of times
syncTimeLen = 20000 # Time leave for syn (ms)
numNodes = 12 # number of node
tdmaSchSlotLen = 30 # broadcast slot lenght

tdmaSchTimes = 30 # broadcast slot times (number of scheduling cycles)
tdmaExcSlotLen = 30 # tdma slot lenght for workining
tdmaNumSlots = 13  # tdma slot numbers
nbTdmaFrameCycles = 1000 # tdma frame cycles

tdmaExcStartTime = syncTimeLen + tdmaSchSlotLen * numNodes * tdmaSchTimes     # Start of TASK slots
FCYCLE = tdmaExcStartTime + tdmaExcSlotLen * tdmaNumSlots * nbTdmaFrameCycles # Full cycle length
FrameLen = tdmaExcSlotLen * tdmaNumSlots

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

    def update_frame_id(self):
        self.frame_id = int((self.current_time_in_cycle - tdmaExcStartTime) / FrameLen)

    def update_slot_id(self):
        self.current_slot_id = int(((self.current_time_in_cycle - tdmaExcStartTime) % FrameLen) / tdmaExcSlotLen)
