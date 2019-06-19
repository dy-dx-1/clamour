from collections import deque

from .uwbMessage import UWBMessage


class MessageBox(deque):
    def __init__(self):
        super(MessageBox, self).__init__()
        self.last_received_message = None

    def append(self, message: UWBMessage) -> None:
        super(MessageBox, self).append(message)
        self.last_received_message = message  # Necessary since deque does not provide peek operations

    def peek_last(self) -> UWBMessage:
        return self.last_received_message

    def empty(self) -> bool:
        return not bool(self)  # If the queue is empty, converting it to bool will give False.
