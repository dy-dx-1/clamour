import sys
import traceback as tb
from multiprocessing import Process

class ContextManagedProcess(Process):
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.join()
        if exc_type: 
            print(f"[ERROR] {exc_type.__name__}: {exc_val}")
            tb.print_exception(exc_type, exc_val, exc_tb)
        else:
            print("[OK] Process completed without exception")