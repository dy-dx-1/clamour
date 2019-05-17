from states import Initialization, Synchronization, Scheduling, Task, Listen, State


# ENCAPSULATE INFO NECESSARY TO STATES IN OBJECTS (ex: TimingInfo could contain logical_clock, clock_diff, etc.)
# Info passed with objects is passed by reference


class TDMANode():
    def __init__(self):
        self.states = { State.INITIALIZATION: Initialization(), 
                        State.SYNCHRONIZATION: Synchronization(),
                        State.SCHEDULING: Scheduling(),
                        State.TASK: Task(),
                        State.LISTEN: Listen() }
        
        self.current_state = self.states[State.INITIALIZATION]
    
    def run(self):
        while True:
            self.current_state = self.current_state.execute()
