import math
import os.path
import csv
from multiprocessing import Lock
from numpy import linalg
from pypozyx import Coordinates, PozyxSerial
from time import time

from .ekf import CustomEKF, DT_THRESHOLD
from contextManagedQueue import ContextManagedQueue
from contextManagedSocket import ContextManagedSocket
from messages import UpdateMessage, UpdateType
from rooms import Floorplan
from soundhandler import SoundSegFaultHandler


class EKFManager:
    def __init__(self, communication_queue: ContextManagedQueue, shared_pozyx: PozyxSerial, shared_pozyx_lock: Lock, pozyx_id: int):
        self.pozyx_id = pozyx_id
        self.ekf = None
        self.debug = 0  # TODO connect to main argv
        self.start_time = 0  # Needed for live graph
        self.yaw_offset = 0  # Measured  in degrees relative to global coordinates X-Axis
        self.last_know_neighbors = {}
        self.communication_queue = communication_queue
        self.floorplan = Floorplan()
        self.current_room = self.floorplan.rooms['24']
        self.pozyx = shared_pozyx
        self.pozyx_lock = shared_pozyx_lock

        self.sh = SoundSegFaultHandler(['python3', 'soundhandler/cyclic_thread.py'])

        filepath = 'broadcast_state.csv'
        isnewfile = os.path.exists(filepath)
        fieldnames = ['pozyx_id', 'timestamp', 'coords_pos_x', 'ekf_pos_x', 'coords_pos_y', 'ekf_pos_y', 'raw_yaw',
                      'ekf_yaw', 'ekf_covariance_matrix', 'two_hop_neighbors']
        self.state_csv = open(filepath, 'w')
        self.writer = csv.DictWriter(self.state_csv, delimiter=',', fieldnames=fieldnames)  
        if isnewfile:
            self.writer.writeheader()

    def run(self) -> None:
        remote_host = "192.168.4.120" if self.debug else None

        self.sh.connect()

        with ContextManagedSocket(remote_host=remote_host, port=10555) as socket:
            self.initialize_ekf(socket)

            while True:
                self.sh.check()
                self.process_latest_state_info(socket)

    def initialize_ekf(self, socket: ContextManagedSocket) -> None:
        while self.ekf is None:
            if not self.communication_queue.empty():
                message = UpdateMessage.load(*self.communication_queue.get_nowait())
                if message.update_type == UpdateType.TRILATERATION:
                    self.start_time = message.timestamp  # This is the first timestamp to be received
                    self.yaw_offset = message.measured_yaw
                    self.ekf = CustomEKF(message.measured_xyz, self.correct_yaw(message.measured_yaw))
                    self.ekf.trilateration_update(message.measured_xyz,
                                                  self.correct_yaw(message.measured_yaw), message.timestamp)

                    self.broadcast_state(socket, message.timestamp, self.ekf.get_position(), self.ekf.get_yaw())
                    self.save_to_csv(message.timestamp, self.ekf.get_position(), self.ekf.get_yaw())
                else:
                    print(message.update_type)

    def process_latest_state_info(self, socket: ContextManagedSocket) -> None:
        update_functions = {UpdateType.PEDOMETER: self.ekf.pedometer_update,
                            UpdateType.TRILATERATION: self.ekf.trilateration_update,
                            UpdateType.RANGING: self.ekf.ranging_update,
                            UpdateType.ZERO_MOVEMENT: self.ekf.zero_movement_update}

        if not self.communication_queue.empty():
            message = UpdateMessage.load(*self.communication_queue.get_nowait())
            update_info = self.extract_update_info(message)

            if message.update_type in [UpdateType.TRILATERATION, UpdateType.RANGING]:
                self.last_know_neighbors = message.topology

            if not self.validate_new_state(update_info[0]):
                update_info = self.generate_zero_update_info(update_info[2])
                message.update_type = UpdateType.ZERO_MOVEMENT
            update_functions[message.update_type](*update_info)

            with self.pozyx_lock:
                self.pozyx.setCoordinates([int(self.ekf.get_position().x), int(self.ekf.get_position().y), int(self.ekf.get_position().z)])

            self.broadcast_state(socket, self.ekf.last_measurement_time, update_info[0], update_info[1])
            self.save_to_csv(self.ekf.last_measurement_time, update_info[0], update_info[1])
            print(self.ekf.get_position())
            self.sh.send_player([int(self.ekf.get_position().x / 10), int(self.ekf.get_position().y / 10), int(self.ekf.get_position().z / 10)])

        elif time() - self.ekf.last_measurement_time > DT_THRESHOLD:
            update_functions[UpdateType.ZERO_MOVEMENT](*self.generate_zero_update_info(self.ekf.last_measurement_time + DT_THRESHOLD))

    def extract_update_info(self, msg: UpdateMessage) -> tuple:
        if msg.update_type == UpdateType.PEDOMETER:
            return self.infer_coordinates(msg.measured_yaw), self.correct_yaw(msg.measured_yaw), msg.timestamp
        elif msg.update_type == UpdateType.TRILATERATION:
            return msg.measured_xyz, self.correct_yaw(msg.measured_yaw), msg.timestamp
        elif msg.update_type == UpdateType.RANGING:
            return msg.measured_xyz, self.correct_yaw(msg.measured_yaw), msg.timestamp, msg.neighbors

    def infer_coordinates(self, measured_yaw: float) -> Coordinates:
        """When new information arrives from the pedometer, it is in the form of a yaw and timestamp.
        Since the step length is constant, we can infer cartesian coordinates from yaw and last know position."""

        step_length = 750  # millimeters

        delta_position_x = step_length * -math.cos(math.radians(self.correct_yaw(measured_yaw)))
        delta_position_y = step_length * math.sin(math.radians(self.correct_yaw(measured_yaw)))

        # The pedometer cannot measure height; we assumed it is constant.
        return Coordinates(self.ekf.x[0] + delta_position_x, self.ekf.x[2] + delta_position_y, self.ekf.x[4])

    def correct_yaw(self, measured_yaw: float) -> float:
        new_yaw = measured_yaw - self.yaw_offset
        return new_yaw if new_yaw > 0 else 360 + new_yaw

    def validate_new_state(self, new_coordinates: Coordinates) -> bool:
        """Makes sure the proposed coordinates stay within the same room or a logically accessible room."""

        if self.current_room.within_bounds(new_coordinates):
            return True

        new_neighbor = self.current_room.within_neighbor_bounds(new_coordinates)
        if new_neighbor is not None:
            print("Changed room.")
            self.current_room = self.floorplan[new_neighbor]
            return True

        return False

    def generate_zero_update_info(self, timestamp: float) -> tuple:
        return self.ekf.get_position(), self.ekf.get_yaw(), timestamp

    def broadcast_state(self, socket: ContextManagedSocket, timestamp: float, coordinates: Coordinates, yaw: float) -> None:
        if self.debug and coordinates is not None:
            socket.send([timestamp - self.start_time,
                         self.ekf.get_position().x, coordinates.x,
                         self.ekf.get_position().y, coordinates.y,
                         self.ekf.get_yaw(), self.correct_yaw(yaw),
                         linalg.det(self.ekf.P), self.pozyx_id])
    
    def save_to_csv(self, timestamp: float, coordinates: Coordinates, yaw: float) -> None:
        if coordinates is not None:
            csv_data = {
                'pozyx_id': self.pozyx_id,
                'timestamp': timestamp,
                'coords_pos_x': coordinates.x,
                'ekf_pos_x': self.ekf.get_position().x,
                'coords_pos_y': coordinates.y,
                'ekf_pos_y': self.ekf.get_position().y,
                'raw_yaw': yaw,
                'ekf_yaw': self.ekf.get_yaw(), 
                'ekf_covariance_matrix': linalg.det(self.ekf.P),
                'two_hop_neighbors': self.last_know_neighbors
            }

            self.writer.writerow(csv_data)
            self.state_csv.flush()
