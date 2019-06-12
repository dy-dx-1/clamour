import random
from time import perf_counter

from pypozyx import POZYX_SUCCESS, Data, PozyxSerial, RXInfo, SingleRegister

from interfaces import Neighborhood, SlotAssignment
from interfaces.timing import NB_TASK_SLOTS
from messages import (MessageBox, MessageFactory, MessageType,
                       UWBSynchronizationMessage, UWBTDMAMessage)


class Messenger:
    def __init__(self, id: int, pozyx: PozyxSerial, neighborhood: Neighborhood, slot_assignment: SlotAssignment):
        self.id = id
        self.message_box = MessageBox()
        self.pozyx = pozyx
        self.neighborhood = neighborhood
        self.slot_assignment = slot_assignment

    def broadcast_synchronization_message(self, time: int) -> None:
        message = UWBSynchronizationMessage(sender_id=self.id)
        message.synchronized_clock = time
        message.encode()

        self.pozyx.sendData(destination=0, data=Data([message.data], "I"))

    def broadcast_control_message(self) -> None:
        if self.message_box.empty():
            # No priority message to broadcast (such as rejection). Proposal can be made.
            if len(self.slot_assignment.pure_send_list) < \
                    int((NB_TASK_SLOTS + 1) / self.neighborhood.synchronized_active_neighbor_count) \
                    and len(self.slot_assignment.non_block) > 0:
                # Propose new slot by randomly choosing from non_block
                slot = random.randint(0, len(self.slot_assignment.non_block))
                code = -1
                self.slot_assignment.send_list[slot] = self.id
            elif len(self.slot_assignment.pure_send_list) < 2 * (NB_TASK_SLOTS + 1) / (
                    3 * self.neighborhood.synchronized_active_neighbor_count) \
                    and len(self.slot_assignment.subpriority_slots) > 1:
                # Propose new slot by randomly choosing from subpriority_slots
                slot = random.choice(self.slot_assignment.subpriority_slots)
                code = -1
                self.slot_assignment.send_list[slot] = self.id
            else:
                # Repetitively broadcast one of own slot. TODO: why?
                slot = random.choice(self.slot_assignment.pure_send_list)
                code = -1
        else:
            message = self.message_box.get()
            slot, code = message.slot, message.code

        self.broadcast(slot, code)

    def broadcast(self, slot: int, code: int) -> None:
        message = UWBTDMAMessage(slot=slot, code=code)
        message.encode()
        self.pozyx.sendData(0, Data([message.data], 'I'))

    def receive_message(self) -> None:
        if self.receive_new_message():
            self.update_neighbor_dictionary()
            if self.message_box.peek_last().message_type == MessageType.TDMA:
                self.handle_control_message(self.message_box.peek_last())

    def handle_control_message(self, control_message: UWBTDMAMessage) -> None:
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
        elif self.slot_assignment.receive_list[message.slot] != message.id:
            self.reject_proposal(message)

    def accept_proposal(self, message: UWBTDMAMessage) -> None:
        """Assigns requested slot to the message's sender."""

        self.slot_assignment.receive_list[message.slot] = message.id

    def accept_receiving(self, message: UWBTDMAMessage) -> None:
        """Since this slot is unavailable for sending message,
        the current node will listen while this slot is active."""

        self.slot_assignment.send_list[message.slot] = -1
        self.slot_assignment.receive_list[message.slot] = message.id

    def reject_proposal(self, message: UWBTDMAMessage) -> None:
        """This slot was already occupied, so the proposal must be rejected."""

        if message not in self.message_box:
            self.message_box.put(message)

    def handle_feedback(self, message: UWBTDMAMessage) -> None:
        """If code == id, it is a feedback from the receiver. 
        It means the current node made a request for assignment which was rejected. 
        It is necessary to delete the send schedule in this slot 
        and to mark this slot as no-sending slot (-2)."""

        if message not in self.message_box:
            self.message_box.put(message)

        self.slot_assignment.send_list[message.slot] = -2

    def handle_assignment_correction(self, message: UWBTDMAMessage) -> None:
        """A previous assignment was wrong
        and there was a request for it to be corrected."""

        self.slot_assignment.receive_list[message.slot] = -1

    def receive_new_message(self) -> bool:
        """Attempts to get a message from the Pozyx tag.
        If the attempt fails or if the same message was received before,
        returns False."""

        is_new_message = False
        sender_id, data, status = self.obtain_message_from_pozyx()

        if status == POZYX_SUCCESS and sender_id != 0 and data != 0:
            received_message = MessageFactory.create(sender_id, data)
            if self.message_box.empty or received_message == self.message_box.peek_last():
                self.message_box.put(MessageFactory.create(sender_id, data))
                is_new_message = True
        else:
            self.handle_error()

        return is_new_message

    def obtain_message_from_pozyx(self) -> (int, int, int):
        info = RXInfo()
        data = Data([0], 'i')
        self.pozyx.getRxInfo(info)
        status = self.pozyx.readRXBufferData(data)

        return info[0], data[0], status

    def update_neighbor_dictionary(self) -> None:
        new_message = self.message_box.peek_last()
        new_message.decode()
        self.neighborhood.current_neighbors[new_message.sender_id] = (new_message.sender_id,
                                                                      perf_counter(),
                                                                      new_message.message_type,
                                                                      new_message)
        self.neighborhood.synchronized_neighbors.append(len(self.neighborhood.current_neighbors))

    def handle_error(self) -> None:
        error_code = SingleRegister()
        message = self.pozyx.getErrorMessage(error_code)
        status = self.pozyx.getErrorCode(error_code)

        # if status == POZYX_SUCCESS:
        #     print("An empty message was received.")
        # else:
        #     print("Error while retrieving message from Pozyx tag: " + str(status) + " occurred: " + message)
