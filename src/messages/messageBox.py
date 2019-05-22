from messages import UWBMessage

class MessageBox():
    # TODO: turn into a queue
    def __init__(self):
        self.last_received_message_id = 0
        self.last_received_message_data = None
        self.current_message: UWBMessage = None
        self.next_message: UWBMessage = None
