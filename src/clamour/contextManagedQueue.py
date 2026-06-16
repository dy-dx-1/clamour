from .custom_terminal import print 
import traceback as tb
from multiprocessing import Queue

class ContextManagedQueue:
    def __init__(self):
        self.queue = Queue(maxsize=20)

    def __enter__(self):
        return self.queue

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type:
            print(f"{exc_type.__name__}: {exc_val}", 'error', 'gen')
            tb.print_exception(exc_type, exc_val, exc_tb)

        self.queue.close()
        self.queue.join_thread()

        if not exc_type:
            print("Queue context exited cleanly", 'ok', 'gen')

    def put(self, message): 
        self.queue.put(message) 

    def empty(self):
        return self.queue.empty() 
    
    def get_nowait(self):
        return self.queue.get_nowait()