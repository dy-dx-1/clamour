from socket import socket

from pypozyx import Data, PozyxSerial, get_first_pozyx_serial_port
from pypozyx.definitions.registers import POZYX_NETWORK_ID

from interfaces import Anchors, Neighborhood, SlotAssignment, Timing
from messenger import Messenger
from mockPozyx import MockPozyx
from states import (TDMAState, Initialization, Listen, Scheduling, State, Synchronization, Task)


class TDMANode:
    def __init__(self):
        self.id = 0
        self.pozyx = self.connect_pozyx()
        self.socket = socket()

        self.neighborhood = Neighborhood()
        self.slot_assignment = SlotAssignment()
        self.timing = Timing()
        self.anchors = Anchors()
        self.messenger = Messenger(self.id, self.pozyx, self.neighborhood, self.slot_assignment)

        self.states = {
            State.INITIALIZATION: Initialization(self.neighborhood, self.anchors, self.id, self.pozyx, self.messenger),
            State.SYNCHRONIZATION: Synchronization(self.neighborhood, self.slot_assignment, self.timing, self.messenger,
                                                   self.id),
            State.SCHEDULING: Scheduling(self.neighborhood, self.slot_assignment, self.timing, self.id, self.messenger),
            State.TASK: Task(self.timing, self.anchors, self.neighborhood, self.id, self.socket, self.pozyx),
            State.LISTEN: Listen(self.slot_assignment, self.timing, self.messenger)}

        self.current_state: TDMAState = self.states[State.INITIALIZATION]

    def __enter__(self):
        self.setup()

        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.socket.close()

    def run(self) -> None:
        while True:
            self.current_state = self.states[self.current_state.execute()]

    def setup(self) -> None:
        self.socket.connect(("192.168.10.2", 10555))
        self.pozyx = self.connect_pozyx()
        self.pozyx.clearDevices()
        self.set_id()

    @staticmethod
    def connect_pozyx() -> PozyxSerial:
        serial_port = get_first_pozyx_serial_port()

        if serial_port is None:
            return MockPozyx()
            # TODO: put back exception
            # raise Exception("No Pozyx connected. Check your USB cable or your driver.")

        return PozyxSerial(serial_port)

    def set_id(self) -> None:
        data = Data([0] * 2)
        self.pozyx.getRead(POZYX_NETWORK_ID, data, None)
        self.id = data[1] * 256 + data[0]
