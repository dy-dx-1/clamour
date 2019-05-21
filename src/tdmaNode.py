from states import Initialization, Synchronization, Scheduling, Task, Listen, State
from interfaces import Neighborhood, SlotAssignment, Timing

# ENCAPSULATE INFO NECESSARY TO STATES IN OBJECTS (ex: TimingInfo could contain logical_clock, clock_diff, etc.)
# Info passed with objects is passed by reference


class TDMANode():
    def __init__(self):
        self.neighborhood = Neighborhood()
        self.slot_assignment = SlotAssignment()
        self.timing = Timing()

        self.states = { State.INITIALIZATION: Initialization(self.neighborhood), 
                        State.SYNCHRONIZATION: Synchronization(self.neighborhood, self.slot_assignment, self.timing),
                        State.SCHEDULING: Scheduling(self.neighborhood, self.slot_assignment),
                        State.TASK: Task(self.timing),
                        State.LISTEN: Listen(self.neighborhood, self.slot_assignment, self.timing) }
        
        self.current_state = self.states[State.INITIALIZATION]
    
    def run(self):
        while True:
            self.current_state = self.current_state.execute()
