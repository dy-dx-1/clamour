from multiprocessing import Process


class ContextManagedProcess(Process):
    def __init__(self, target):
        super(ContextManagedProcess, self).__init__(target=target)
    
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.join()
