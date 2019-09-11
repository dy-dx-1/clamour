import sys
import traceback as tb
from multiprocessing import Lock
from pypozyx import PozyxSerial
from time import sleep, time

from interfaces import Anchors, Neighborhood, SlotAssignment, Timing
from messenger import Messenger
from states import (TDMAState, Initialization, Listen, Scheduling, State, Synchronization, Task)


class TDMANode:
    def __init__(self, multiprocess_communication_queue, shared_pozyx: PozyxSerial,
                 shared_pozyx_lock: Lock, pozyx_id: int):

        self.clear_devices(shared_pozyx, shared_pozyx_lock)

        neighborhood = Neighborhood()
        slot_assignment = SlotAssignment()
        anchors = Anchors()
        messenger = Messenger(pozyx_id, shared_pozyx, neighborhood, slot_assignment,
                              shared_pozyx_lock, multiprocess_communication_queue)

        self.timing = Timing()
        self.loop_start_time = time()

        self.states = self.states = {
            State.INITIALIZATION: Initialization(neighborhood, anchors, pozyx_id, shared_pozyx, messenger,
                                                 multiprocess_communication_queue, shared_pozyx_lock),
            State.SYNCHRONIZATION: Synchronization(neighborhood, slot_assignment, self.timing, messenger,
                                                   pozyx_id, multiprocess_communication_queue),
            State.SCHEDULING: Scheduling(neighborhood, slot_assignment, self.timing, pozyx_id, messenger),
            State.TASK: Task(self.timing, anchors, neighborhood, pozyx_id, shared_pozyx, shared_pozyx_lock, messenger),
            State.LISTEN: Listen(slot_assignment, self.timing, messenger, neighborhood)}

        self.current_state = self.states[State.INITIALIZATION]

    def __enter__(self):
        print("Setting up TDMA node...")
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        print(exception_type, exception_value)
        tb.print_tb(traceback, file=sys.stdout)
        print("Finished with TDMA node.")

    def run(self) -> None:
        while True:
            start_time = time()
            self.timing.update_current_time()
            self.current_state = self.states[self.current_state.execute()]
            self.wait(start_time)

            # if int(1 / (time() - start_time)) < 59.0:
            #     print(f"WARNING: --- {int(1 / (time() - start_time))} Hz ---")

    @staticmethod
    def wait(start_time: float):
        frequency = 60.0  # Hz
        period = 1.0 / frequency

        while (time() - start_time) <= period:
            sleep(0.00000001)

    @staticmethod
    def clear_devices(pozyx: PozyxSerial, pozyx_lock: Lock()) -> None:
        with pozyx_lock:
            pozyx.clearDevices()
