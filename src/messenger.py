import random
from multiprocessing import Lock
from struct import error as StructError
from time import perf_counter, time

from pypozyx import Data, PozyxSerial, RXInfo, SingleRegister, Coordinates

from contextManagedQueue import ContextManagedQueue
from interfaces import Neighborhood, SlotAssignment, State
from interfaces.timing import NB_TASK_SLOTS
from messages import (MessageBox, MessageFactory, UWBSynchronizationMessage, UWBTDMAMessage,
                      UWBTopologyMessage, UpdateMessage, UpdateType)


class Messenger:
    def __init__(self, id: int, shared_pozyx: PozyxSerial, neighborhood: Neighborhood,
                 slot_assignment: SlotAssignment, shared_pozyx_lock: Lock,
                 multiprocess_communication_queue: ContextManagedQueue):
        self.id = id
        self.message_box = MessageBox()
        self.pozyx = shared_pozyx
        self.pozyx_lock = shared_pozyx_lock
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment
        self.multiprocess_communication_queue = multiprocess_communication_queue
        self.received_messages = set()
        self.should_go_back_to_sync = 0

    def send_new_measurement(self, update_type: UpdateType, measured_position: Coordinates, yaw: float,
                             neighbors: list = None, topology: dict = None) -> None:
        message = UpdateMessage(update_type, time(), yaw, measured_position, neighbors, topology)
        self.multiprocess_communication_queue.put(UpdateMessage.save(message))

    def broadcast_synchronization_message(self, timestamp: int, synchronized: bool) -> None:
        message = UWBSynchronizationMessage(sender_id=self.id, synchronized=synchronized)
        message.synchronized_clock = timestamp
        message.encode()

        with self.pozyx_lock:
            self.pozyx.sendData(destination=0, data=Data([0xAA, message.data], 'Bi'))

    def broadcast_control_message(self) -> None:
        if self.message_box.empty():
            # No priority message to broadcast (such as rejection). Proposal can be made.
            code = -1
            if self.should_chose_from_non_block():
                broadcast_origin = 1
                # Propose new slot by randomly choosing from non_block
                slot = random.randint(0, len(self.slot_assignment.non_block) - 1)
                self.slot_assignment.send_list[slot] = self.id
            elif self.should_chose_from_subpriority():
                broadcast_origin = 2
                # Propose new slot by randomly choosing from subpriority_slots
                slot = random.choice(self.slot_assignment.subpriority_slots)
                self.slot_assignment.send_list[slot] = self.id
            else:
                broadcast_origin = 3
                slot = random.choice(self.slot_assignment.pure_send_list)
        else:
            broadcast_origin = 4
            message = self.message_box.popleft()
            slot, code = message.slot, message.code

        if slot > len(self.slot_assignment.receive_list):
            print("NEXT SCHEDULING MSG MIGHT BE WRONG. Origin:", broadcast_origin, "Slot:", slot, "Code:", code)
        self.broadcast(slot, code)

    def broadcast_topology_message(self):
        message = UWBTopologyMessage(sender_id=self.id, topology=self.neighborhood.current_neighbors.keys())
        message.encode()

        with self.pozyx_lock:
            self.pozyx.sendData(destination=0, data=Data([0xAA, message.data], 'Bi'))
        
    def should_chose_from_non_block(self) -> bool:
        return len(self.slot_assignment.pure_send_list) < \
               int((NB_TASK_SLOTS + 1) / (len(self.neighborhood.current_neighbors) + 1)) \
               and len(self.slot_assignment.non_block) > 0

    def should_chose_from_subpriority(self) -> bool:
        return len(self.slot_assignment.pure_send_list) < \
               2 * (NB_TASK_SLOTS + 1) / (3 * (len(self.neighborhood.current_neighbors) + 1)) \
               and len(self.slot_assignment.subpriority_slots) > 1

    def clear_non_scheduling_messages(self) -> None:
        while not self.message_box.empty() and not isinstance(self.message_box.peek_first(), UWBTDMAMessage):
            self.message_box.popleft()

    def broadcast(self, slot: int, code: int) -> None:
        message = UWBTDMAMessage(sender_id=self.id, slot=slot, code=code)
        message.encode()

        with self.pozyx_lock:
            self.pozyx.sendData(0, Data([0xAA, message.data], 'Bi'))

    def receive_message(self, state: State) -> bool:
        is_new_message, should_go_to_sync = self.receive_new_message(state)
        if is_new_message:
            self.update_topology(state)
            if isinstance(self.message_box.peek_last(), UWBTDMAMessage):
                self.handle_control_message(self.message_box.pop())

        return should_go_to_sync

    def handle_control_message(self, control_message: UWBTDMAMessage) -> None:
        if control_message.slot > len(self.slot_assignment.receive_list):
            print("INVALID SLOT, SKIPPING MSG", control_message.slot, self.slot_assignment.receive_list)
            return

        if control_message.code == -1:
            self.handle_assignment_request(control_message)
        elif control_message.code == self.id:
            self.handle_feedback(control_message)
        elif control_message.code == self.slot_assignment.receive_list[control_message.slot]:
            self.handle_assignment_correction(control_message)

    def handle_assignment_request(self, message: UWBTDMAMessage) -> None:
        if self.slot_assignment.block[message.slot] == -1:
            self.accept_proposal(message)
        elif self.slot_assignment.send_list[message.slot] == -2:
            self.accept_receiving(message)
        elif self.slot_assignment.receive_list[message.slot] != message.sender_id:
            self.reject_proposal(message)

    def accept_proposal(self, message: UWBTDMAMessage) -> None:
        """Assigns requested slot to the message's sender."""

        self.slot_assignment.receive_list[message.slot] = message.sender_id

    def accept_receiving(self, message: UWBTDMAMessage) -> None:
        """Since this slot is unavailable for sending message,
        the current node will listen while this slot is active."""

        self.slot_assignment.send_list[message.slot] = -1
        self.slot_assignment.receive_list[message.slot] = message.sender_id

    def reject_proposal(self, message: UWBTDMAMessage) -> None:
        """This slot was already occupied, so the proposal must be rejected."""

        message.code = message.sender_id
        message.id = self.id

        if message not in self.message_box:
            self.message_box.append(message)

    def handle_feedback(self, message: UWBTDMAMessage) -> None:
        """If code == id, it is a feedback from the receiver. 
        It means the current node made a request for assignment which was rejected. 
        It is necessary to delete the send schedule in this slot 
        and to mark this slot as no-sending slot (-2)."""

        if message not in self.message_box:
            self.message_box.append(message)

        self.slot_assignment.send_list[message.slot] = -2

    def handle_assignment_correction(self, message: UWBTDMAMessage) -> None:
        """A previous assignment was wrong
        and there was a request for it to be corrected."""

        self.slot_assignment.receive_list[message.slot] = -1

    def receive_new_message(self, state: State) -> (bool, bool):
        """Attempts to get a message from the Pozyx tag.
        If the attempt fails or if the same message was received before,
        returns False."""

        is_new_message = False
        sender_id, data = self.obtain_message_from_pozyx()

        if sender_id != 0 and data[1] != 0:
            received_message = MessageFactory.create(sender_id, data)
            if isinstance(received_message, UWBTopologyMessage):
                received_message.decode()
                self.update_topology(State.LISTEN, topology_info=received_message.neighbors)
            elif received_message not in self.received_messages:
                self.received_messages.add(received_message)
                self.message_box.append(received_message)
                is_new_message = True
                self.should_go_back_to_sync += int(state != State.SYNCHRONIZATION and isinstance(received_message, UWBSynchronizationMessage))

            if self.should_go_back_to_sync > max(len(self.neighborhood.current_neighbors) * 3, 10):
                print("Received sync messages, going back to sync.")

        return is_new_message, (self.should_go_back_to_sync > max(len(self.neighborhood.current_neighbors) * 3, 10))

    def obtain_message_from_pozyx(self) -> (int, Data, int):
        data = Data([0, 0], 'Bi')
        sender_id, message_byte_size = self.get_message_metadata()

        if message_byte_size == data.byte_size:
            with self.pozyx_lock:
                self.pozyx.readRXBufferData(data)

        return sender_id, data

    def get_message_metadata(self) -> (int, int):
        info = RXInfo()

        try:
            with self.pozyx_lock:
                self.pozyx.getRxInfo(info)
        except StructError as s:
            print("RxInfo crashes! ", str(s))

        return info[0], info[1]

    def update_topology(self, state: State, device_list: list = None, topology_info: dict = None) -> None:
        if device_list is not None:
            for device in device_list:
                self.neighborhood.add_neighbor(device, perf_counter(), state)
        elif topology_info is not None:
            neighbor_id = next(iter(topology_info))  # Gets the first key (the dict contains only one neighbor.)
            self.neighborhood.add_neighbor(neighbor_id, perf_counter(), state, topology_info[neighbor_id])
        else:
            new_message = self.message_box.peek_last()
            new_message.decode()
            self.neighborhood.add_neighbor(new_message.sender_id, perf_counter(), state)

            if isinstance(new_message, UWBSynchronizationMessage):
                self.update_synced_neighbors(new_message)

    def update_synced_neighbors(self, message: UWBSynchronizationMessage) -> None:
        if message.synchronized:
            self.neighborhood.add_synced_neighbor(message.sender_id)
        else:
            self.neighborhood.remove_synced_neighbor(message.sender_id)

    def handle_error(self, function_name: str) -> None:
        error_code = SingleRegister()

        try:
            with self.pozyx_lock:
                self.pozyx.getErrorCode(error_code)
                message = self.pozyx.getErrorMessage(error_code)
        except StructError as s:
            print(str(s))

        if error_code != 0x0:
            print("Error in", function_name, ":", message)
            with self.pozyx_lock:
                self.pozyx.resetSystem()
