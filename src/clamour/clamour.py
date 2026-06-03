#!/usr/bin/python3

import os
import sys
import yaml 
from multiprocessing import Lock, Queue

from .ekf import EKFManager, CustomOdometry
from .tdmaNode import TDMANode
from .contextManagedQueue import ContextManagedQueue
from .contextManagedProcess import ContextManagedProcess
from .pedometer import Pedometer
from .messages import PoseMessage, CustomOdometryMessage
from .runnableProcess import RunnableProcess
#from .soundmanager import SoundManager

from .interfaces import PozyxTag, BitcrazeTag

#################################################### Loading config file 
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
CONFIG_PATH = os.path.join(PROJECT_ROOT, 'configuration', 'clamour_config.yaml')

with open(CONFIG_PATH, 'r') as f: # NOTE: maybe interesting to add error handling/checks in future. For now assuming easy enough to read and debug. 
    cfg = yaml.safe_load(f) 

match cfg['tag_type']:
    case "Bitcraze": 
        TAG_FACTORY = lambda: BitcrazeTag(tag_id = cfg['tag_id'], dw1000_bus = cfg['dw1000_bus'], dw1000_cs = cfg['dw1000_cs'])
    case "Pozyx": 
        # TODO NOTE: pozyx doesn't currently support setting IDs through config file. 
        TAG_FACTORY = lambda: PozyxTag() 
    case _: 
        raise ValueError(f"Invalid tag type: {cfg['tag_type']}. Check your config file.")

#################################################### Clamour class 
def keep_alive(process: RunnableProcess) -> None:
    while True:
        try:
            process.run()
        except Exception as e:
            print("[ERROR] Clamour.keep_alive(): A process that needs to be kept alive died and will be restarted. Error:", str(e))

class Clamour:
    def __init__(self, custom_odometries):
        self.custom_odometries = custom_odometries

    def start(self, sound: bool, pose_callback, communication_queue):
        # The different levels of context managers are required to ensure everything starts and stops cleanly.
        with TAG_FACTORY() as shared_tag: # Type of tag defined in config file 
            shared_tag_lock = Lock()
            tag_id = shared_tag.tag_id
            with ContextManagedQueue() as sound_queue:
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
            
