from socket import socket
from pypozyx import PozyxSerial, get_first_pozyx_serial_port

from states import Initialization, Synchronization, Scheduling, Task, Listen, State
from interfaces import Neighborhood, SlotAssignment, Timing
from messages import MessageBox

# ENCAPSULATE INFO NECESSARY TO STATES IN OBJECTS (ex: TimingInfo could contain logical_clock, clock_diff, etc.)
# Info passed with objects is passed by reference


class TDMANode():
    def __init__(self):
        self.pozyx = None
        self.socket = socket()

        self.neighborhood = Neighborhood()
        self.slot_assignment = SlotAssignment()
        self.timing = Timing()
        self.message_box = MessageBox()

        self.states = { State.INITIALIZATION: Initialization(self.neighborhood, self.message_box), 
                        State.SYNCHRONIZATION: Synchronization(self.neighborhood, self.slot_assignment, self.timing, self.message_box),
                        State.SCHEDULING: Scheduling(self.neighborhood, self.slot_assignment),
                        State.TASK: Task(self.timing),
                        State.LISTEN: Listen(self.neighborhood, self.slot_assignment, self.timing, self.message_box) }
        
        self.current_state = self.states[State.INITIALIZATION]

    def __enter__(self):
        serial_port = get_first_pozyx_serial_port()
        if serial_port is not None:
            self.pozyx = PozyxSerial(serial_port)
        else:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")

    def __exit__(self, exception_type, exception_value, traceback):
        self.socket.close()
    
    def run(self):
        while True:
            self.current_state = self.current_state.execute()
