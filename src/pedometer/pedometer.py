import math

import numpy as np

from multiprocessing import Lock
from pypozyx import PozyxSerial, LinearAcceleration, EulerAngles
from time import sleep, time
from struct import error as StructError
from messages import UpdateMessage, UpdateType
from .pedometerMeasurement import PedometerMeasurement


class Pedometer:
    def __init__(self, communication_queue, pozyx: PozyxSerial, pozyx_lock: Lock):
        self.pozyx = pozyx
        self.pozyx_lock = pozyx_lock
        self.steps = []
        self.buffer = np.array([PedometerMeasurement(0, 0, 0)] * 20)
        self.communication_queue = communication_queue

    def run(self):
        print("Running pedometer")
        start_time = time()
        previous_angles = np.array([0.0, 0.0, 0.0, 0.0])
        nb_measurements = 0

        while True:
            linear_acceleration = self.get_acceleration_measurement()
            yaw, previous_angles = self.get_filtered_yaw_measurement(previous_angles, nb_measurements)
            vertical_acceleration = self.vertical_acceleration(self.holding_angle(), linear_acceleration)

            # Only used to verify if previous_angles has been filled before using it for smoothing.
            if nb_measurements < 5:
                nb_measurements += 1

            self.buffer = np.append(self.buffer[1:],
                                    [PedometerMeasurement(time() - start_time, vertical_acceleration, yaw)])

            self.detect_step()
            sleep(0.01)

    def get_acceleration_measurement(self) -> LinearAcceleration:
        linear_acceleration = LinearAcceleration()
        with self.pozyx_lock:
            self.pozyx.getAcceleration_mg(linear_acceleration)

        return linear_acceleration

    def get_filtered_yaw_measurement(self, previous_angles: np.ndarray, i: int) -> (np.ndarray, np.ndarray):
        angles = EulerAngles()
        try:
            with self.pozyx_lock:
                self.pozyx.getEulerAngles_deg(angles)
        except StructError as s:
            print(s)

        yaw = angles.heading

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
        message = UpdateMessage(UpdateType.PEDOMETER, time(), self.steps[-1].z)
        self.communication_queue.put(UpdateMessage.save(message))
