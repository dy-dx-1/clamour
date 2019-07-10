#!/usr/bin/python3

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

from tdmaNode import TDMANode
from contextManagedQueue import ContextManagedQueue
from contextManagedProcess import ContextManagedProcess
from pedometer import Pedometer

from multiprocessing import Process


def main():
    with ContextManagedQueue() as multiprocess_communication_queue:
        pedometer = Pedometer(multiprocess_communication_queue)

        process = Process(target=pedometer.run)
        process.start()

        # with ContextManagedProcess(target=pedometer.run) as side_process:
        #     side_process.start()

        with TDMANode(multiprocess_communication_queue) as node:
            node.run()

        process.join()


if __name__ == "__main__":
    main()
