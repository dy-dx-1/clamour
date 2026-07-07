from .custom_terminal import print 
import traceback as tb
from multiprocessing import Process
import os 

class ContextManagedProcess(Process):
    def __enter__(self):
        # Record the PID of the parent process that entered the context
        self._parent_pid = os.getpid()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # ONLY join and log if we are still in the parent process
        if os.getpid() == self._parent_pid:
            self.join()
            if exc_type: 
                print(f"{exc_type.__name__}: {exc_val}", status='error', type='gen')
                tb.print_exception(exc_type, exc_val, exc_tb)
            else:
                print("Process completed without exception", status='ok', type='gen')
        else:
            # We are inside the child process; do not join or print parent logs.
            # Just let the child exit silently so the parent can handle it.
            pass