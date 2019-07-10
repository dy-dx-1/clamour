#!/usr/bin/python3

import os
import sys
from pypozyx import PozyxSerial, get_first_pozyx_serial_port
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

from tdmaNode import TDMANode
from contextManagedQueue import ContextManagedQueue
from contextManagedProcess import ContextManagedProcess
from pedometer import Pedometer

from multiprocessing import Process, Lock


def connect_pozyx() -> PozyxSerial:
    serial_port = get_first_pozyx_serial_port()

    if serial_port is None:
        raise Exception("No Pozyx connected. Check your USB cable or your driver.")

    return PozyxSerial(serial_port)


def main():
    # The different levels of context managers are required to ensure everything starts and stops cleanly.

    with ContextManagedQueue() as multiprocess_communication_queue:
        shared_pozyx = connect_pozyx()
        shared_pozyx_lock = Lock()

        pedometer = Pedometer(multiprocess_communication_queue, shared_pozyx, shared_pozyx_lock)

        with ContextManagedProcess(target=pedometer.run) as side_process:
            side_process.start()

            with TDMANode(multiprocess_communication_queue, shared_pozyx, shared_pozyx_lock) as node:
                node.run()


if __name__ == "__main__":
    main()
