#!/usr/bin/python3

import os
import sys
from pypozyx import PozyxSerial, get_first_pozyx_serial_port, Data
from pypozyx.definitions.registers import POZYX_NETWORK_ID
from time import sleep

from pypozyx.structures.device_information import DeviceDetails

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

from ekf import EKFManager
from tdmaNode import TDMANode
from contextManagedQueue import ContextManagedQueue
from contextManagedProcess import ContextManagedProcess
from pedometer import Pedometer

from multiprocessing import Lock


def connect_pozyx() -> PozyxSerial:
    serial_port = get_first_pozyx_serial_port()

    if serial_port is None:
        raise Exception("No Pozyx connected. Check your USB cable or your driver.")

    return PozyxSerial(serial_port)


def connect_and_reset() -> PozyxSerial:
    temp_pozyx = connect_pozyx()
    print("Connected first pozyx")

    temp_pozyx.resetSystem()
    system_details = DeviceDetails()
    print(system_details)

    sleep(10)

    second_pozyx = connect_pozyx()
    print("Connected second pozyx")
    system_details = DeviceDetails()
    print(system_details)

    return second_pozyx


def get_pozyx_id(pozyx) -> int:
    data = Data([0] * 2)
    pozyx.getRead(POZYX_NETWORK_ID, data)

    return data[1] * 256 + data[0]


def main():
    # The different levels of context managers are required to ensure everything starts and stops cleanly.

    with ContextManagedQueue() as multiprocess_communication_queue:
        shared_pozyx = connect_and_reset()
        shared_pozyx_lock = Lock()
        pozyx_id = get_pozyx_id(shared_pozyx)

        ekf_manager = EKFManager(multiprocess_communication_queue, pozyx_id)
        pedometer = Pedometer(multiprocess_communication_queue, shared_pozyx, shared_pozyx_lock)

        with ContextManagedProcess(target=ekf_manager.run) as ekf_manager_process:
            ekf_manager_process.start()
            with ContextManagedProcess(target=pedometer.run) as pedometer_process:
                pedometer_process.start()
                with TDMANode(multiprocess_communication_queue, shared_pozyx, shared_pozyx_lock, pozyx_id) as node:
                    node.run()


if __name__ == "__main__":
    main()
