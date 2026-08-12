import os 
import csv
import math 
import numpy.linalg as linalg
from time import sleep, time 
from typing import Literal
from struct import error as StructError
from multiprocessing.synchronize import Lock

from ..custom_terminal import print 
from ..config import SAVE_TO_CSV
from ..interfaces import Tag, Coordinates
from .ekf import CustomEKF, DT_THRESHOLD
from ..contextManagedQueue import ContextManagedQueue
from ..messages.updateMessage import UpdateMessage
from ..messages.soundMessage import SoundMessage
from ..messages.types import UpdateType
from ..messages.poseMessage import PoseMessage
from ..rooms import Floorplan

WAIT_TIME_DURING_INIT = 0.001 # s to wait at each iter while waiting for com queue to load first message

class StateEstimator: 
    """
    General interface for pose estimation. 
    Provides the essential methods to use and connects them to the appropriate manager.
    """
    def __init__(self, shared_tag: Tag, shared_tag_lock: Lock, estimator_type: Literal['ekf', 'fg'], communication_queue: ContextManagedQueue, sound_queue: ContextManagedQueue):
        self.tag = shared_tag 
        self.tag_lock = shared_tag_lock 

        self.estimator = None 
        self.estimator_type = estimator_type # validity check done in config TODO 
        
        self.com_queue = communication_queue
        self.state_csv, self.writer = self.initialize_csv()

    def initialize_estimator(self): 
        while self.estimator is None: 
            if not self.com_queue.empty():
                msg = UpdateMessage.load(*self.com_queue.get_nowait())
                if msg.update_type == UpdateType.TRILATERATION:  # Currently only starting estimator from fully constrained update (trilateration) 
                    measured_pos, measured_yaw = msg.measured_xyz, self.correct_yaw(msg.measured_yaw)
                    self.yaw_offset = measured_yaw

                    if self.estimator_type == 'ekf': 
                        self.estimator = CustomEKF(measured_pos, measured_yaw)
                        self.estimator.trilateration_update(measured_pos, measured_yaw, msg.timestamp)
                    elif self.estimator_type == 'fg':
                        # TODO 
                        pass 

                    post_pos, post_yaw = self.estimator.get_position(), self.estimator.get_yaw() # posterior position and yaw from estimator
                    self.save_to_csv(msg.timestamp, msg, post_pos, post_yaw)
                    self.pose_callback(PoseMessage(post_pos.x, post_pos.y, post_pos.z, post_yaw))
            else:
                sleep(WAIT_TIME_DURING_INIT)  
        print("ESTIMATOR INITIALIZATION DONE", 'ok', 'loc')

    def process_latest_state_info(self): 
        if not self.com_queue.empty(): 
            msg = UpdateMessage.load(*self.com_queue.get_nowait())

            match msg.update_type: # NOTE TODO, figure out how to deal with trilateration vs taking all measuremetns for FG. Abstract trilat choice into the ekf and just pass all ranges to FG?
                case UpdateType.PEDOMETER: 
                    update_info = (self.infer_coordinates(msg.measured_yaw), self.correct_yaw(msg.measured_yaw), msg.timestamp)
                    self.estimator.pedometer_update(*update_info) 
                case UpdateType.TRILATERATION: 
                    self.update_neighbors(msg.topology) 

                    update_info = (msg.measured_xyz, self.correct_yaw(msg.measured_yaw), msg.timestamp)
                    self.estimator.trilateration_update(*update_info)
                case UpdateType.RANGING: 
                    self.update_neighbors(msg.topology) 

                    update_info = (msg.measured_xyz, self.correct_yaw(msg.measured_yaw), msg.timestamp, msg.neighbors)
                    self.estimator.ranging_update(*update_info)
                case UpdateType.ZERO_MOVEMENT: 
                    update_info = (msg.measured_xyz, self.correct_yaw(msg.measured_yaw), msg.timestamp)
                    self.estimator.zero_movement_update(*update_info)
                case UpdateType.TOPOLOGY:  
                    self.update_neighbors(msg.topology) 
                case UpdateType.CUSTOM_POSE: # TODO remove / replace by IMU factor? 
                    update_info = (Coordinates(msg.pose.x, msg.pose.y, msg.pose.z), msg.pose.yaw, msg.R, msg.timestamp)
                    self.estimator.custom_odometry_update(*update_info) 

            try: 
                with self.tag_lock:
                    self.tag.coordinates = self.estimator.get_position() 
            except StructError as s: 
                print(f"Estimator couldn't get new state estimate in process_latest_state_info(): {str(s)}", 'error', 'loc')

            raw_coords, raw_yaw = (update_info[0], update_info[1]) if msg.update_type != UpdateType.TOPOLOGY else (self.estimator.get_position(), self.estimator.get_yaw())
            self.save_to_csv(self.estimator.last_measurement_time, msg, raw_coords, raw_yaw) 
            self.pose_callback(PoseMessage(raw_coords.x, raw_coords.y, raw_coords.z, raw_yaw))

            if self.sound: 
                sound_message = SoundMessage(self.estimator.get_position())
                self.sound_queue.put(SoundMessage.save(sound_message))

        elif (time() - self.last_measurement_time) > DT_THRESHOLD: # TODO add last_measurement time parameter, orignally from ekf
            # Add DT_THRESHOLD TODO, 
            # If too much time goes by without any updates, do a zero mvt one to prevent drift 
            # NOTE TODO: I believe this can be removed in future with addition of IMU 
            self.zero_movement_update(self.estimator.get_position(), 
                                      self.estimator.get_yaw(), 
                                      self.last_measurement_time + DT_THRESHOLD)
        else: 
            sleep(WAIT_TIME_DURING_INIT) 

    def infer_coordinates(self, measured_yaw: float) -> Coordinates:
        """When new information arrives from the pedometer, it is in the form of a yaw and timestamp.
        Since the step length is constant, we can infer cartesian coordinates from yaw and last know position."""

        step_length = 750  # millimeters

        delta_position_x = step_length * -math.cos(math.radians(self.correct_yaw(measured_yaw)))
        delta_position_y = step_length * math.sin(math.radians(self.correct_yaw(measured_yaw)))

        # The pedometer cannot measure height; we assumed it is constant.
        return Coordinates(self.estimator.x[0] + delta_position_x, self.estimator.x[2] + delta_position_y, self.estimator.x[4])

    def correct_yaw(self, measured_yaw: float) -> float:
        new_yaw = measured_yaw - self.yaw_offset
        return new_yaw if new_yaw > 0 else 360 + new_yaw

    def validate_new_state(self, new_coordinates: Coordinates) -> bool:
        """
        Makes sure the proposed coordinates stay within the same room or a logically accessible room.
        TODO NOTE: This is currently unused, previously, there was a commented check in process_latest_state_info
        that used to do a zero mvt update if out of bounds. In the future, evaluate if can still do something interesting
        with this information. 
        2026-08-12 
        """
        if self.current_room.within_bounds(new_coordinates):
            return True

        new_neighbor = self.current_room.within_neighbor_bounds(new_coordinates, self.floorplan.rooms)
        if new_neighbor is not None:
            print("Changed room.", 'info', 'loc')
            self.current_room = self.floorplan.rooms[new_neighbor]
            return True

        return False
    
    def run(self) -> None: 
        try: 
            self.initialize_estimator() 
            while True: 
                self.process_latest_state_info()
        except Exception as e: 
            print(f'State Estimator crashed! Error: {str(e)}', 'error', 'loc')
            raise e 

    @staticmethod 
    def initialize_csv(): 
        if not SAVE_TO_CSV: 
            return None, None 
        filepath = 'pose_estimation.csv'
        is_new_file = os.path.exists(filepath)
        fieldnames = ['tag_id', 'timestamp', 'synchronized_clock', 'offset', 'update_type',
                      'raw_x', 'estimator_x', 'raw_y', 'estimator_y', 'raw_z', 'estimator_z', 'raw_yaw', 'estimator_yaw', 
                      'covariance_matrix', 'slots', 'two_hop_neighbors']

        state_csv = open(filepath, 'w')
        writer = csv.DictWriter(state_csv, delimiter=',', fieldnames=fieldnames)
        if is_new_file:
            writer.writeheader()

        return state_csv, writer
    
    def save_to_csv(self, timestamp: float, message: UpdateMessage, coordinates: Coordinates, yaw: float) -> None:
        if not SAVE_TO_CSV: 
            return 
        if coordinates is not None and message.update_type != UpdateType.CUSTOM_POSE:
            csv_data = {
                'tag_id': self.tag.tag_id,
                'timestamp': timestamp,
                'synchronized_clock': message.synchronized_clock,
                'offset': message.offset,
                'update_type': message.update_type,
                'raw_x': coordinates.x,
                'estimator_x': self.estimator.get_position().x,
                'raw_y': coordinates.y,
                'estimator_y': self.estimator.get_position().y,
                'raw_z': coordinates.z,
                'estimator_z': self.estimator.get_position().z,
                'raw_yaw': yaw,
                'ekf_yaw': self.estimator.get_yaw(),
                'ekf_covariance_matrix': linalg.det(self.estimator.P),
                'slots': message.slots,
                'two_hop_neighbors': self.last_know_neighbors
            }

            self.writer.writerow(csv_data)
            self.state_csv.flush()