#!/usr/bin/python3

import os
import sys
from multiprocessing import Lock, Manager

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

from ekf import EKFManager, CustomOdometry
from tdmaNode import TDMANode
from multiprocessing import Queue
from contextManagedQueue import ContextManagedQueue
from contextManagedProcess import ContextManagedProcess
from pedometer import Pedometer
from messages import PoseMessage, CustomOdometryMessage
from runnableProcess import RunnableProcess
#from soundmanager import SoundManager

from interfaces import PozyxTag, BitcrazeTag
TAG_TYPE = "BitCraze" # TODO: to be put in a config file later 


def keep_alive(process: RunnableProcess) -> None:
    while True:
        try:
            process.run()
        except Exception as e:
            print("A process that needs to be kept alive died and will be restarted. Error:", str(e))

class Clamour:
    def __init__(self, custom_odometries):
        self.custom_odometries = custom_odometries

    def start(self, sound: bool, pose_callback, communication_queue):
        # The different levels of context managers are required to ensure everything starts and stops cleanly.
        with ContextManagedQueue() as sound_queue:
            if TAG_TYPE == "BitCraze":
                shared_tag = BitcrazeTag("/dev/ttyACM0", 123) # placeholder values, to be replaced with config file values
            elif TAG_TYPE == "Pozyx":
                shared_tag = PozyxTag()
            else: 
                raise Exception("Invalid TAG_TYPE, check your config file")
            shared_tag_lock = Lock()
            tag_id = shared_tag.tag_id

            ekf_manager = EKFManager(pose_callback, sound_queue, communication_queue, shared_tag, shared_tag_lock, tag_id, sound)
            #pedometer = Pedometer(communication_queue, shared_pozyx, shared_pozyx_lock)
            tdma_node = TDMANode(communication_queue, shared_tag, shared_tag_lock, tag_id)

            #if sound:
            #    sound_player = SoundManager(sound_queue)

            with ContextManagedProcess(target=ekf_manager.run) as ekf_manager_process:
                ekf_manager_process.start()
                with ContextManagedProcess(target=tdma_node.run) as tdma_process:
                    tdma_process.start()
                    #with ContextManagedProcess(target=pedometer.run) as pedometer_process:
                        #pedometer_process.start()

                    #    if sound:
                    #        keep_alive(sound_player)

    def start_non_blocking(self, sound: bool, pose_callback):
        self.communication_queue = Queue()
        for custom_odometry in self.custom_odometries:
            custom_odometry.set_pose_listener(self._on_custom_pose_update)

        clamour_process = ContextManagedProcess(target=self.start, args=[sound, pose_callback, self.communication_queue])
        clamour_process.start()

    def _on_custom_pose_update(self, custom_odometry: CustomOdometry, pose: PoseMessage, timestamp: float):
        if(self.communication_queue is not None):
            message = CustomOdometryMessage(pose, custom_odometry.get_R(), timestamp)
            self.communication_queue.put(CustomOdometryMessage.save(message))
            
