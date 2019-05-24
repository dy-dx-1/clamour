from time import perf_counter
from interfaces import Neighborhood, SlotAssignment
from messages import MessageBox, MessageFactory, MessageType, TDMAControlMessage
from pypozyx import PozyxSerial, Data, RXInfo, POZYX_SUCCESS

class Messenger():
    def __init__(self, id: int, message_box: MessageBox, pozyx: PozyxSerial, neighborhood: Neighborhood, slot_assigment: SlotAssignment):
        self.id = id
        self.message_box = message_box
        self.pozyx = pozyx
        self.neighborhood = neighborhood
        self.slot_assigment = slot_assigment

    def broadcast_control_message(self):
        pass

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

        if not self.message_box.contains(message):
            self.message_box.message_queue.append(message)

    def handle_feedback(self, message: TDMAControlMessage) -> None:
        """If code == id, it is a feedback from the receiver. 
        It means the current node made a request for assignment which was rejected. 
        It is necessary to delete the send schedule in this slot 
        and to mark this slot as no-sending slot (-2)."""

        if not self.message_box.contains(message):
            self.message_box.message_queue.append(message)
        
        self.slot_assigment.send_list[message.slot] = -2

    def handle_assigment_correction(self, message: TDMAControlMessage) -> None:
        """A previous assignment was wrong 
        and there was a request for it to be corrected."""

        self.slot_assigment.receive_list[message.slot] = -1

    def is_new_message(self, sender_id: int, message_data: int) -> bool:
        if sender_id != 0 and message_data != 0:
            if id != self.message_box.last_received_message_id or message_data != self.message_box.last_received_message_data:
                self.message_box.last_received_message_id = sender_id
                self.message_box.last_received_message_data = message_data
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

        self.message_box.current_message = MessageFactory.create(self.message_box.last_received_message_data)
        self.message_box.current_message.decode()
        self.neighborhood.current_neighbors[self.message_box.last_received_message_id] = (self.message_box.last_received_message_id,
                                                                                        perf_counter(),
                                                                                        self.message_box.current_message.message_type,
                                                                                        self.message_box.current_message)
        self.neighborhood.synchronized_active_neighbor_count.append(len(self.neighborhood.current_neighbors))
