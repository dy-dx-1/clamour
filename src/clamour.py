#!/usr/bin/python3

import os
import sys
from multiprocessing import Lock
from pypozyx import PozyxSerial, get_first_pozyx_serial_port, Data
from pypozyx.definitions.registers import POZYX_NETWORK_ID

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

from ekf import EKFManager
from tdmaNode import TDMANode
from contextManagedQueue import ContextManagedQueue
from contextManagedProcess import ContextManagedProcess
from pedometer import Pedometer
from runnableProcess import RunnableProcess
from soundmanager import SoundManager


def connect_pozyx() -> PozyxSerial:
    serial_port = get_first_pozyx_serial_port()

    if serial_port is None:
        raise Exception("No Pozyx connected. Check your USB cable or your driver.")

    return PozyxSerial(serial_port)


def get_pozyx_id(pozyx) -> int:
    data = Data([0] * 2)
    pozyx.getRead(POZYX_NETWORK_ID, data)

    return data[1] * 256 + data[0]


def keep_alive(process: RunnableProcess) -> None:
    while True:
        try:
            process.run()
        except Exception as e:
            print("A process that needs to be kept alive died and will be restarted. Error:", str(e))


def main(sound: bool):
    # The different levels of context managers are required to ensure everything starts and stops cleanly.
    with ContextManagedQueue() as sound_queue:
        with ContextManagedQueue() as communication_queue:
            shared_pozyx = connect_pozyx()
            shared_pozyx_lock = Lock()
            pozyx_id = get_pozyx_id(shared_pozyx)

            ekf_manager = EKFManager(sound_queue, communication_queue, shared_pozyx, shared_pozyx_lock, pozyx_id, sound)
            pedometer = Pedometer(communication_queue, shared_pozyx, shared_pozyx_lock)
            tdma_node = TDMANode(communication_queue, shared_pozyx, shared_pozyx_lock, pozyx_id)

            if sound:
                sound_player = SoundManager(sound_queue)

            with ContextManagedProcess(target=ekf_manager.run) as ekf_manager_process:
                ekf_manager_process.start()
                with ContextManagedProcess(target=tdma_node.run) as tdma_process:
                    tdma_process.start()
                    with ContextManagedProcess(target=pedometer.run) as pedometer_process:
                        pedometer_process.start()

                        if sound:
                            keep_alive(sound_player)


if __name__ == "__main__":
    # An argument of anything else than 0 sets debug to True.
    sound = False
    if len(sys.argv) > 1:
        sound = bool(int(sys.argv[1]))

    main(sound)
