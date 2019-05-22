from socket import socket
from pypozyx import PozyxSerial, get_first_pozyx_serial_port

from states import Initialization, Synchronization, Scheduling, Task, Listen, State
from interfaces import Neighborhood, SlotAssignment, Timing, Anchors
from messages import MessageBox


class TDMANode():
    def __init__(self):
        self.id = 0
        self.pozyx = self.connect_pozyx()
        self.socket = socket()

        self.neighborhood = Neighborhood()
        self.slot_assignment = SlotAssignment()
        self.timing = Timing()
        self.message_box = MessageBox()
        self.anchors = Anchors()

        self.states = { State.INITIALIZATION: Initialization(self.neighborhood, self.message_box, self.anchors, self.id, self.pozyx), 
                        State.SYNCHRONIZATION: Synchronization(self.neighborhood, self.slot_assignment, self.timing, self.message_box, self.id),
                        State.SCHEDULING: Scheduling(self.neighborhood, self.slot_assignment, self.anchors, self.id),
                        State.TASK: Task(self.timing, self.anchors, self.id, self.socket),
                        State.LISTEN: Listen(self.neighborhood, self.slot_assignment, self.timing, self.message_box) }
        
        self.current_state = self.states[State.INITIALIZATION]

    def __enter__(self):
        if self.pozyx is None:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")
        
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.socket.close()
    
    def run(self):
        while True:
            self.current_state = self.current_state.execute()

    def connect_pozyx(self):
        serial_port = get_first_pozyx_serial_port()
        return PozyxSerial(serial_port) if serial_port is not None else None
