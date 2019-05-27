from queue import Queue

from .uwbMessage import UWBMessage


class MessageBox(Queue):
    def __contains__(self, message: UWBMessage) -> bool:
        for m in self.queue:
            if m == message:
                return True
        return False

    def peek_first(self) -> UWBMessage:
        return self.queue[0]

    def peek_last(self) -> UWBMessage:
        return self.queue[len(self.queue) - 1]
