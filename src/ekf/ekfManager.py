import math
import os.path
import csv
from numpy import linalg
from pypozyx import Coordinates
from time import time

from .ekf import CustomEKF, DT_THRESHOLD
from contextManagedQueue import ContextManagedQueue
from contextManagedSocket import ContextManagedSocket
from messages import UpdateMessage, UpdateType
from rooms import Floorplan

class EKFManager:
    def __init__(self, communication_queue: ContextManagedQueue, pozyx_id: int):
        self.pozyx_id = pozyx_id
        self.ekf = None
        self.start_time = 0  # Needed for live graph
        self.yaw_offset = 0  # Measured  in degrees relative to global coordinates X-Axis
        self.communication_queue = communication_queue
        self.floorplan = Floorplan()
        self.current_room = self.floorplan.rooms['Arena']

        filepath = '/dev/csv/broadcast_state.csv'
        isnewfile = os.path.exists(filepath))
        fieldnames = ['elapsed_time', 'efk_posx', 'coords_x', 'efk_posy', 'coords_y', 'efk_yaw', 'poszyx_id']
        self.state_csv = open(filepath, 'a')
        self.writer = csv.DictWriter(self.state_csv, delimiter=',', fieldnames=fieldnames)  
        if isnewfile:
            self.writer.writeheader()
        

    def run(self) -> None:
        with ContextManagedSocket(remote_host="192.168.4.120", port=10555) as socket:
            self.initialize_ekf(socket)

            while True:
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

    def process_latest_state_info(self, socket: ContextManagedSocket) -> None:
        update_functions = {UpdateType.PEDOMETER: self.ekf.pedometer_update,
                            UpdateType.TRILATERATION: self.ekf.trilateration_update,
                            UpdateType.RANGING: self.ekf.ranging_update,
                            UpdateType.ZERO_MOVEMENT: self.ekf.zero_movement_update}

        if not self.communication_queue.empty():
            message = UpdateMessage.load(*self.communication_queue.get_nowait())
            update_info = self.extract_update_info(message)
            if not self.validate_new_state(update_info[0]):
                update_info = self.generate_zero_update_info(update_info[2])
                message.update_type = UpdateType.ZERO_MOVEMENT
            update_functions[message.update_type](*update_info)
            print("[", message.timestamp, "] Updated position by ", message.update_type, ". New position: ")
            # TODO: update pozyx position value with EKF result?
            self.broadcast_state(socket, self.ekf.last_measurement_time, update_info[0], update_info[1])
        elif time() - self.ekf.last_measurement_time > DT_THRESHOLD:
            update_functions[UpdateType.ZERO_MOVEMENT](*self.generate_zero_update_info(self.ekf.last_measurement_time + DT_THRESHOLD))

    def extract_update_info(self, msg: UpdateMessage) -> tuple:
        if msg.update_type == UpdateType.PEDOMETER:
            return self.infer_coordinates(msg.measured_yaw), self.correct_yaw(msg.measured_yaw), msg.timestamp
        elif msg.update_type in [UpdateType.TRILATERATION, UpdateType.RANGING]:
            return msg.measured_xyz, self.correct_yaw(msg.measured_yaw), msg.timestamp

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
        # TODO: Find source of None coordinates
        if coordinates is not None:
            socket.send([timestamp - self.start_time,
                         self.ekf.get_position().x, coordinates.x,
                         self.ekf.get_position().y, coordinates.y,
                         self.ekf.get_yaw(), self.correct_yaw(yaw),
                         linalg.det(self.ekf.P), self.pozyx_id])

            csv_data = {
                'elapsed_time': timestamp - self.start_time,
                'efk_posx'  : self.ekf.get_position().x, 
                'coords_x'  : coordinates.x, 
                'efk_posy'  : self.ekf.get_position().y, 
                'coords_y'  : coordinates.y,
                'efk_yaw'   : self.ekf.get_yaw(), 
                'poszyx_id' : self.pozyx_id
            }

            writer.writerow(csv_data)
            self.state_csv.flush()