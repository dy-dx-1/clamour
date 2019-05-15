from states import Initialization, Synchronization, Scheduling, Task, Listen, State


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
            self.current_state.execute()
