import socket

from pypozyx import Data, PozyxSerial, get_first_pozyx_serial_port
from pypozyx.definitions.registers import POZYX_NETWORK_ID

from interfaces import Anchors, Neighborhood, SlotAssignment, Timing
from messenger import Messenger
from states import (TDMAState, Initialization, Listen, Scheduling, State, Synchronization, Task)


class TDMANode:
    def __init__(self):
        self.id = 0
        self.socket = socket.socket()

        self.neighborhood = Neighborhood()
        self.slot_assignment = SlotAssignment()
        self.timing = Timing()
        self.anchors = Anchors()

        # These attributes need to be set after setup
        self.pozyx = None
        self.messenger = None
        self.states = None
        self.current_state = None

    def __enter__(self):
        self.setup()
        self.initialize_states()

        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.socket.close()

    def run(self) -> None:
        while True:
            self.timing.update_current_time()
            print("Time: ", self.timing.current_time_in_cycle)
            self.current_state = self.states[self.current_state.execute()]

    def setup(self) -> None:
        try:
            print("Attempting connection to a local service...")
            self.socket.connect((socket.gethostname(), 10555))
        except ConnectionRefusedError:
            print("The connection was either refused, or the service you are trying to reach is unavailable.")
        
        self.pozyx = self.connect_pozyx()
        self.pozyx.clearDevices()
        self.set_id()

    def initialize_states(self) -> None:
        self.messenger = Messenger(self.id, self.pozyx, self.neighborhood, self.slot_assignment)

        self.states = {
            State.INITIALIZATION: Initialization(self.neighborhood, self.anchors, self.id, self.pozyx, self.messenger),
            State.SYNCHRONIZATION: Synchronization(self.neighborhood, self.slot_assignment, self.timing, self.messenger,
                                                   self.id),
            State.SCHEDULING: Scheduling(self.neighborhood, self.slot_assignment, self.timing, self.id, self.messenger),
            State.TASK: Task(self.timing, self.anchors, self.neighborhood, self.id, self.socket, self.pozyx),
            State.LISTEN: Listen(self.slot_assignment, self.timing, self.messenger)}

        self.current_state = self.states[State.INITIALIZATION]

    @staticmethod
    def connect_pozyx() -> PozyxSerial:
        serial_port = get_first_pozyx_serial_port()

        if serial_port is None:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")

        return PozyxSerial(serial_port)

    def set_id(self) -> None:
        data = Data([0] * 2)
        self.pozyx.getRead(POZYX_NETWORK_ID, data)
        self.id = data[1] * 256 + data[0]
        print("Device ID: ", self.id)
