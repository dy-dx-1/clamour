import sys
import traceback as tb
from multiprocessing import Lock
from pypozyx import PozyxSerial

from interfaces import Anchors, Neighborhood, SlotAssignment, Timing
from messenger import Messenger
from states import (TDMAState, Initialization, Listen, Scheduling, State, Synchronization, Task)


class TDMANode:
    def __init__(self, multiprocess_communication_queue, shared_pozyx: PozyxSerial,
                 shared_pozyx_lock: Lock, pozyx_id: int):
        self.id = pozyx_id
        self.multiprocess_communication_queue = multiprocess_communication_queue
        self.pozyx = shared_pozyx
        self.pozyx_lock = shared_pozyx_lock

        self.neighborhood = Neighborhood()
        self.slot_assignment = SlotAssignment()
        self.timing = Timing()
        self.anchors = Anchors()

        # These attributes need to be set after setup
        self.messenger = None
        self.states = None
        self.current_state = None

    def __enter__(self):
        print("Setting up TDMA node...")
        self.setup()
        self.initialize_states()

        return self

    def __exit__(self, exception_type, exception_value, traceback):
        print(exception_type, exception_value)
        tb.print_tb(traceback, file=sys.stdout)
        print("Finished with TDMA node.")

    def run(self) -> None:
        while True:
            self.timing.update_current_time()
            self.current_state = self.states[self.current_state.execute()]

    def setup(self) -> None:
        with self.pozyx_lock:
            self.pozyx.clearDevices()

    def initialize_states(self) -> None:
        self.messenger = Messenger(self.id, self.pozyx, self.neighborhood, self.slot_assignment,
                                   self.pozyx_lock, self.multiprocess_communication_queue)

        self.states = {
            State.INITIALIZATION: Initialization(self.neighborhood, self.anchors, self.id, self.pozyx,
                                                 self.messenger, self.multiprocess_communication_queue,
                                                 self.pozyx_lock),
            State.SYNCHRONIZATION: Synchronization(self.neighborhood, self.slot_assignment, self.timing, self.messenger,
                                                   self.id, self.multiprocess_communication_queue),
            State.SCHEDULING: Scheduling(self.neighborhood, self.slot_assignment, self.timing, self.id, self.messenger),
            State.TASK: Task(self.timing, self.anchors, self.neighborhood, self.id,
                             self.pozyx, self.pozyx_lock, self.messenger),
            State.LISTEN: Listen(self.slot_assignment, self.timing, self.messenger, self.neighborhood)}

        self.current_state = self.states[State.INITIALIZATION]
