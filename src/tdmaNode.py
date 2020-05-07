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
            State.TASK: Task(self.timing, anchors, neighborhood, pozyx_id, shared_pozyx, shared_pozyx_lock, messenger,
                             slot_assignment),
            State.LISTEN: Listen(slot_assignment, self.timing, messenger, neighborhood)}

        self.current_state = self.states[State.INITIALIZATION]
        self.last_state_id = State.INITIALIZATION
        self.current_state_id = State.INITIALIZATION


    def run(self) -> None:
        while True:
            start_time = time()
            self.timing.update_current_time()
            self.current_state_id = self.current_state.execute()
            self.current_state = self.states[self.current_state_id]

            if(self.last_state_id == State.LISTEN and self.current_state_id == State.SYNCHRONIZATION):
                self.current_state.first_exec_time = None
                print("Enter Synchronization, new Full Cycle starts")
            self.last_state_id = self.current_state_id

            self.wait(start_time)

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
