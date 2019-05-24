import random
from time import perf_counter
from interfaces import Neighborhood, SlotAssignment
from interfaces.timing import tdmaNumSlots
from messages import MessageBox, MessageFactory, MessageType, TDMAControlMessage, UWBTDMAMessage, UWBSynchronizationMessage
from pypozyx import PozyxSerial, Data, RXInfo, SingleRegister, POZYX_SUCCESS

class Messenger():
    def __init__(self, id: int, pozyx: PozyxSerial, neighborhood: Neighborhood, slot_assigment: SlotAssignment):
        self.id = id
        self.message_box = MessageBox()
        self.pozyx = pozyx
        self.neighborhood = neighborhood
        self.slot_assigment = slot_assigment

    def broadcast_synchronization_message(self, time: int) -> None:
        message = UWBSynchronizationMessage()
        message.synchronized_clock = time
        message.encode()

        self.pozyx.sendData(Data([message.data], "I"))

    def broadcast_control_message(self):
        if self.message_box.empty():
            # No prioritary message to broadcast (such as rejection). Proposal can be made.
            if len(self.slot_assigment.pure_send_list) < (int((tdmaNumSlots+1)/self.neighborhood.synchronized_active_neighbor_count)) and len(self.slot_assigment.non_block) > 0:
                # Propose new slot by randomly choosing from non_block
                slot = random.randint(0, len(self.slot_assigment.non_block))
                code = -1
                self.slot_assigment.send_list[slot] = self.id
            elif len(self.slot_assigment.pure_send_list) < 2*(tdmaNumSlots+1)/(3*self.neighborhood.synchronized_active_neighbor_count) and len(self.slot_assigment.subpriority_slots) > 1:
                # Propose new slot by randomly choosing from subpriority_slots
                slot = random.choice(self.slot_assigment.subpriority_slots)
                code = -1
                self.slot_assigment.send_list[slot] = self.id
            else:
                # Repetitively broadcast one of own slot. TODO: why?
                slot = random.choice(self.slot_assigment.pure_send_list)
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
        sender_id, data, status = self.obtain_message_from_pozyx()

        if status == POZYX_SUCCESS and self.is_new_message(sender_id, data):
            self.update_neighbor_dictionary()
            received_data = self.neighborhood.current_neighbors[sender_id][3]
            if received_data.message_type == MessageType.TDMA:
                received_control_message = TDMAControlMessage(sender_id, received_data.slot, received_data.code)
                self.handle_control_message(received_control_message)

    def handle_control_message(self, control_message: TDMAControlMessage) -> None:
        if control_message.code == -1:
            self.handle_assignement_request(control_message)
        elif control_message.code == self.id:
            self.handle_feedback(control_message)
        elif control_message.code == self.slot_assigment.receive_list[control_message.slot]:
            self.handle_assigment_correction(control_message)

    def handle_assignement_request(self, message: TDMAControlMessage) -> None:
        if self.slot_assigment.block[message.slot] == -1:
            self.accept_proposal(message)
        elif self.slot_assigment.send_list[message.slot] == -2:
            self.accept_receiving(message)
        elif self.slot_assigment.receive_list[message.slot] != message.id:
            self.reject_proposal(message)

    def accept_proposal(self, message: TDMAControlMessage) -> None:
        """Assigns requested slot to the message's sender."""

        self.slot_assigment.receive_list[message.slot] = message.id

    def accept_receiving(self, message: TDMAControlMessage) -> None:
        """Since this slot is unavailable for sending message,
        the current node will listen while this slot is active."""
        
        self.slot_assigment.send_list[message.slot] = -1
        self.slot_assigment.receive_list[message.slot] = message.id

    def reject_proposal(self, message: TDMAControlMessage) -> None:
        """This slot was already occupied, so the proposal must be rejected."""

        if message not in self.message_box:
            self.message_box.put(message)

    def handle_feedback(self, message: TDMAControlMessage) -> None:
        """If code == id, it is a feedback from the receiver. 
        It means the current node made a request for assignment which was rejected. 
        It is necessary to delete the send schedule in this slot 
        and to mark this slot as no-sending slot (-2)."""

        if message not in self.message_box:
            self.message_box.put(message)
        
        self.slot_assigment.send_list[message.slot] = -2

    def handle_assigment_correction(self, message: TDMAControlMessage) -> None:
        """A previous assignment was wrong 
        and there was a request for it to be corrected."""

        self.slot_assigment.receive_list[message.slot] = -1

    def is_new_message(self, sender_id: int, message_data: int) -> bool:
        if sender_id != 0 and message_data != 0:
            if id != self.message_box.peek_last().id or message_data != self.message_box.peek_last().data:
                self.message_box.peek_last().id = sender_id
                self.message_box.peek_last().data = message_data
                return True
        return False

    def obtain_message_from_pozyx(self):
        info = RXInfo()
        data = Data([0], 'i')
        self.pozyx.getRxInfo(info)
        status = self.pozyx.readRXBufferData(data)

        return info[0], data[0], status
    
    def update_neighbor_dictionary(self):
        #TODO: This function should not be here -> untagle dependancies with neighborhood

        new_message = MessageFactory.create(self.message_box.peek_last().data)
        new_message.decode()
        self.neighborhood.current_neighbors[self.message_box.peek_last().id] = (self.message_box.peek_last().id,
                                                                                perf_counter(),
                                                                                new_message.message_type,
                                                                                new_message)
        self.message_box.put(new_message)
        self.neighborhood.synchronized_active_neighbor_count.append(len(self.neighborhood.current_neighbors))

    def handle_error(self):
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code)
        print("Error occured: " + status)
