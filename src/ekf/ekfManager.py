import math
from numpy import linalg
from pypozyx import Coordinates

from .ekf import CustomEKF, DT_THRESHOLD
from contextManagedQueue import ContextManagedQueue
from contextManagedSocket import ContextManagedSocket
from messages import UpdateMessage, UpdateType


class EKFManager:
    def __init__(self, communication_queue: ContextManagedQueue):
        self.ekf = None
        self.start_time = 0  # Needed for live graph
        self.yaw_offset = 0  # Measured  in degrees relative to global coordinates X-Axis
        self.communication_queue = communication_queue

    def run(self) -> None:
        with ContextManagedSocket(remote_host="192.168.2.107", port=10555) as socket:
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
                    self.ekf = CustomEKF(message.measured_xyz, message.measured_yaw - self.yaw_offset)
                    self.ekf.trilateration_update(message.measured_xyz, message.measured_yaw, message.timestamp)

                    self.broadcast_latest_state(socket, message.timestamp, message.measured_xyz, message.measured_yaw)

    def process_latest_state_info(self, socket: ContextManagedSocket) -> None:
        update_functions = {UpdateType.PEDOMETER: self.ekf.pedometer_update,
                            UpdateType.TRILATERATION: self.ekf.trilateration_update,
                            UpdateType.RANGING: self.ekf.ranging_update,
                            UpdateType.ZERO_MOVEMENT: self.ekf.zero_movement_update}

        if not self.communication_queue.empty():
            message = UpdateMessage.load(*self.communication_queue.get_nowait())
            update_functions[message.update_type](self.extract_update_info(message))
            print(message.update_type)
        elif self.ekf.dt > DT_THRESHOLD:
            update_functions[UpdateType.ZERO_MOVEMENT](self.generate_zero_update_info())

        self.broadcast_latest_state(socket, self.ekf.last_measurement_time, self.ekf.get_position(), self.ekf.get_yaw)

    def extract_update_info(self, msg: UpdateMessage) -> tuple:
        if msg.update_type == UpdateType.PEDOMETER:
            return self.infer_coordinates(msg.measured_yaw), msg.measured_yaw - self.yaw_offset, msg.timestamp
        elif msg.update_type in [UpdateType.TRILATERATION, UpdateType.RANGING]:
            return msg.measured_xyz, msg.measured_yaw - self.yaw_offset, msg.timestamp

    def infer_coordinates(self, measured_yaw: float) -> Coordinates:
        """When new information arrives from the pedometer, it is in the form of a yaw and timestamp.
        Since the step length is constant, we can infer cartesian coordinates from yaw and last know position."""

        step_length = 750  # millimeters

        delta_position_x = step_length * math.cos(math.radians(measured_yaw - self.yaw_offset))
        delta_position_y = step_length * -math.sin(math.radians(measured_yaw - self.yaw_offset))

        # The pedometer cannot measure height; we assumed it is constant.
        return Coordinates(self.ekf.x[0] + delta_position_x, self.ekf.x[2] + delta_position_y, self.ekf.x[4])

    def generate_zero_update_info(self) -> tuple:
        return self.ekf.get_position(), self.ekf.get_yaw(), self.ekf.last_measurement_time + DT_THRESHOLD

    def broadcast_latest_state(self, socket: ContextManagedSocket, timestamp: float, coordinates: Coordinates, yaw: float) -> None:
        socket.send([timestamp - self.start_time,
                     self.ekf.x[0], coordinates.x,
                     self.ekf.x[2], coordinates.y,
                     self.ekf.x[6], yaw - self.yaw_offset,
                     linalg.det(self.ekf.P)])
