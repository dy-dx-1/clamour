from collections import deque
from typing import Union

from .uwbMessage import UWBMessage


class MessageBox(deque):
    def __init__(self):
        super(MessageBox, self).__init__()
        self.last_received_message = None

    def append(self, message: UWBMessage) -> None:
        #print('Adding message to box')
        super(MessageBox, self).append(message)
        self.last_received_message = message  # Necessary since deque does not provide peek operations
        #print('New current length:', len(self))

    def peek_first(self) -> Union[UWBMessage, None]:
        return None if self.empty() else self[0]

    def peek_last(self) -> UWBMessage:
        return self.last_received_message

    def empty(self) -> bool:
        return not bool(self)  # If the queue is empty, converting it to bool will give False.
