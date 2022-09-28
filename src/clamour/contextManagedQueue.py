import sys
import traceback as tb
from multiprocessing import Queue


class ContextManagedQueue:
    def __init__(self):
        self.queue = Queue(maxsize=20)

    def __enter__(self):
        return self.queue

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.queue.close()
        self.queue.join_thread()
        print(exc_type, exc_val)
        tb.print_tb(exc_tb, file=sys.stdout)