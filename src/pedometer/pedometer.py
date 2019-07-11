import math

import matplotlib.pyplot as plt
import numpy as np

from multiprocessing import Lock
from pypozyx import PozyxSerial, LinearAcceleration, EulerAngles, Coordinates
from time import perf_counter, sleep
from mpl_toolkits.mplot3d import Axes3D
from .ekf import PedometerEKF
from messages import UpdateMessage, UpdateType
from pedometerMeasurement import PedometerMeasurement


class Pedometer:
    def __init__(self, communication_queue, pozyx: PozyxSerial, pozyx_lock: Lock):
        print("init pedometer")
        self.pozyx = pozyx
        self.pozyx_lock = pozyx_lock
        self.steps = []
        self.buffer = np.array([PedometerMeasurement(0, 0, 0)] * 20)
        self.ekf = PedometerEKF(Coordinates())
        self.ekf_positions = []
        self.communication_queue = communication_queue

    def display(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        t = [step.x for step in self.steps]
        ekf_x, ekf_y = [pos.x for pos in self.ekf_positions], [pos.y for pos in self.ekf_positions]

        ax.scatter(ekf_x, ekf_y, t, s=10, c='b', marker="o")

        ax.set_xlabel('X coordinate')
        ax.set_ylabel('Y coordinate')
        ax.set_zlabel('Time')
        plt.grid()
        plt.show()

    def run(self):
        print("running pedometer")
        start_time = perf_counter()
        previous_angles = np.array([0.0, 0.0, 0.0, 0.0])

        for i in range(2000):
            for j in range(20):
                linear_acceleration = self.get_acceleration_measurement()
                yaw, previous_angles = self.get_filtered_yaw_measurement(previous_angles, i)
                vertical_acceleration = self.vertical_acceleration(self.holding_angle(), linear_acceleration)

                self.buffer = np.append(self.buffer[1:],
                                        [PedometerMeasurement(perf_counter() - start_time, vertical_acceleration, yaw)])

                self.detect_step()
                sleep(0.01)

        self.display()

    def process_latest_state_info(self):
        # While not trilateration received, wait. (We want to init EKF with precise trilateration coordinates.)
        message = UpdateMessage.load(self.communication_queue.get())

        if message.update_type == UpdateType.PEDOMETER:
            self.ekf.pedometer_update(message.measured_xyz, message.measured_yaw, message.delta_time)
        elif message.update_type == UpdateType.TRILATERATION:
            self.ekf.trilateration_update(message.measured_xyz, message.delta_time)
        elif message.update_type == UpdateType.RANGING:
            self.ekf.ranging_update(message.measured_xyz, message.delta_time, message.neighbors)

    def get_acceleration_measurement(self) -> LinearAcceleration:
        linear_acceleration = LinearAcceleration()
        with self.pozyx_lock:
            self.pozyx.getAcceleration_mg(linear_acceleration)

        return linear_acceleration

    def get_filtered_yaw_measurement(self, previous_angles: np.ndarray, i: int) -> (np.ndarray, np.ndarray):
        angles = EulerAngles()
        with self.pozyx_lock:
            self.pozyx.getEulerAngles_deg(angles)
        yaw = angles[0]

        if self.jump(previous_angles[-1], yaw):
            previous_angles = [yaw] * 4

        filtered_yaw = self.filter(previous_angles, yaw) \
            if self.jump(previous_angles[-1], yaw) and i >= len(previous_angles) - 1 \
            else yaw

        return filtered_yaw, np.append(previous_angles[1:], filtered_yaw)

    @staticmethod
    def jump(prev, new):
        """Checks whether the orientation has changed by more than 20 degrees"""
        return abs(prev - new) > 20

    @staticmethod
    def filter(previous_yaws: np.ndarray, new_yaw: float):
        filtering_weights = np.array([0.01, 0.02, 0.03, 0.04, 0.9])

        return np.dot(filtering_weights, np.append(previous_yaws, new_yaw))

    def detect_step(self) -> None:
        min_delay = 0.2
        min_acc = 1.175

        local_max_index = np.argmax(self.buffer)
        local_max = self.buffer[local_max_index]

        last_time = 0 if len(self.steps) == 0 else self.steps[-1].x
        delta_time = local_max.x - last_time

        if local_max.y > min_acc and delta_time >= min_delay and self.zero_crossing(self.buffer, local_max_index):
            self.steps.append(local_max)
            self.update_trajectory()

    @staticmethod
    def zero_crossing(local_acc: np.ndarray, local_max: int) -> bool:
        previous_smaller = [previous < local_acc[local_max] for previous in local_acc[:local_max]]
        subsequent_smaller = [subsequent < local_acc[local_max] for subsequent in local_acc[local_max + 1:]]

        # If the local_max is at 0 or at len(local_acc), one of the 2 lists will be empty
        return (all(previous_smaller) or len(previous_smaller) == 0) \
            and (all(subsequent_smaller) or len(subsequent_smaller) == 0)

    def holding_angle(self) -> float:
        gravity = LinearAcceleration()
        with self.pozyx_lock:
            self.pozyx.getGravityVector_mg(gravity)

        return math.atan(abs(gravity[2]/gravity[1])) if gravity[1] != 0 else 0

    @staticmethod
    def vertical_acceleration(holding_angle: float, user_acceleration: LinearAcceleration) -> float:
        """Calculates the vertical acceleration of the device in (g), minus Earth gravitation"""

        return (user_acceleration[2] * math.sin(holding_angle) + user_acceleration[1] * math.cos(holding_angle)) / 981

    def update_trajectory(self):
        step_length = 0.75

        delta_position_x = step_length * -math.cos(math.radians(self.steps[-1].z))
        delta_position_y = step_length * math.sin(math.radians(self.steps[-1].z))

        measured_position = Coordinates(self.ekf.x[0] + delta_position_x, self.ekf.x[2] + delta_position_y, 0)
        measured_yaw = self.steps[-1].z
        delta_time = self.steps[-1].x - (self.steps[-2].x if len(self.steps) > 1 else 0)

        message = UpdateMessage(UpdateType.PEDOMETER, measured_position, delta_time, measured_yaw)
        self.communication_queue.put(UpdateMessage.save(message))

        self.ekf_positions.append(Coordinates(self.ekf.x[0], self.ekf.x[2], self.ekf.x[3]))
