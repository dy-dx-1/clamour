from queue import Queue

from messages import UWBMessage


class MessageBox(Queue):
    """This class is inspired by a standard queue.
    However, it needed to be based on a regular array
    because traversal is sometimes necessary."""

    def __contains__(self, message: UWBMessage) -> bool:
        for m in self.queue:
            if m == message:
                return True
        return False

    def peek_first(self) -> UWBMessage:
        return self.queue[0]

    def peek_last(self) -> UWBMessage:
        return self.queue[len(self.queue) - 1]
