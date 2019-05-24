from typing import List
from messages import UWBMessage

class MessageBox():
    # TODO: turn into a queue
    def __init__(self):
        self.last_received_message_id = 0
        self.last_received_message_data = None
        self.current_message: UWBMessage = None
        self.message_queue: List[UWBMessage] = []

    def contains(self, message):
        for x in range(len(self.message_queue)):
            if self.message_queue[x] == message:
                return True
        return False