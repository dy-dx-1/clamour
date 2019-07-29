import sys
import traceback as tb
from multiprocessing import Process


class ContextManagedProcess(Process):
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.join()
        print(exc_type, exc_val)
        tb.print_tb(exc_tb, file=sys.stdout)
