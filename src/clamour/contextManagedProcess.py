from .custom_terminal import print 
import traceback as tb
from multiprocessing import Process

class ContextManagedProcess(Process):
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.join()
        if exc_type: 
            print(f"{exc_type.__name__}: {exc_val}", status='error', type='gen')
            tb.print_exception(exc_type, exc_val, exc_tb)
        else:
            print("Process completed without exception", status='ok', type='gen')