from multiprocessing import Queue


class ContextManagedQueue:
    def __init__(self):
        self.queue = Queue()

    def __enter__(self):
        return self.queue

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.queue.close()
        self.queue.join_thread()
