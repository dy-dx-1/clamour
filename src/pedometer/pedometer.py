import math

import matplotlib.pyplot as plt
import numpy as np

from pypozyx import PozyxSerial, get_first_pozyx_serial_port, LinearAcceleration, EulerAngles
from time import perf_counter, sleep
from mpl_toolkits.mplot3d import Axes3D



class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __gt__(self, other):
        return self.y > other.y

    def __ge__(self, other):
        return self.y >= other.y

    def __le__(self, other):
        return self.y <= other.y

    def __lt__(self, other):
        return self.y < other.y

    def __eq__(self, other):
        return self.y == other.y

    def __repr__(self):
        return f"x: {round(self.x, 3)} y: {round(self.y, 3)} z: {round(self.z, 3)}"


class Pedometer:
    def __init__(self):
        self.pozyx = self.connect_pozyx()
        self.position = Point(0, 0, 0)
        self.positions = []
        self.steps = []
        self.buffer = np.array([Point(0, 0, 0)] * 20)

    def display(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        x, y, t = [pos.x for pos in self.positions], [pos.y for pos in self.positions], [step.x for step in self.steps]
        print(len(x), len(y), len(t))

        ax.scatter(x, y, t, s=10, c='r', marker="o")
        ax.set_xlabel('X coordinate')
        ax.set_ylabel('Y coordinate')
        ax.set_zlabel('Time')
        plt.grid()
        plt.show()

    def run(self):
        start_time = perf_counter()
        previous_angles = np.array([0.0, 0.0, 0.0, 0.0])

        for i in range(200):
            for j in range(10):
                linear_acceleration = self.get_acceleration_measurement()
                yaw, previous_angles = self.get_filtered_yaw_measurement(previous_angles, i)

                vertical_acceleration = self.vertical_acceleration(self.holding_angle(), linear_acceleration)
                self.buffer = np.append(self.buffer[1:], [Point(perf_counter() - start_time, vertical_acceleration, yaw)])

                self.detect_step()
                sleep(0.01)

        self.display()

    def get_acceleration_measurement(self) -> LinearAcceleration:
        linear_acceleration = LinearAcceleration()
        self.pozyx.getAcceleration_mg(linear_acceleration)

        return linear_acceleration

    def get_filtered_yaw_measurement(self, previous_angles: np.ndarray, i: int) -> (np.ndarray, np.ndarray):
        angles = EulerAngles()
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
        min_delay = 0.175
        min_acc = 1.175

        local_max_index = np.argmax(self.buffer)
        local_max = self.buffer[local_max_index]

        last_time = 0 if len(self.steps) == 0 else self.steps[-1].x
        delta_time = local_max.x - last_time

        if local_max.y > min_acc and delta_time >= min_delay and self.zero_crossing(self.buffer, local_max_index):
            print("step")
            self.steps.append(local_max)
            self.update_trajectory(local_max)

    @staticmethod
    def zero_crossing(local_acc: np.ndarray, local_max: int) -> bool:
        previous_smaller = [previous < local_acc[local_max] for previous in local_acc[:local_max]]
        subsequent_smaller = [subsequent < local_acc[local_max] for subsequent in local_acc[local_max + 1:]]

        # If the local_max is at 0 or at len(local_acc), one of the 2 lists will be empty
        return (all(previous_smaller) or len(previous_smaller) == 0) \
            and (all(subsequent_smaller) or len(subsequent_smaller) == 0)

    def holding_angle(self) -> float:
        gravity = LinearAcceleration()
        self.pozyx.getGravityVector_mg(gravity)

        return math.atan(abs(gravity[2]/gravity[1])) if gravity[1] != 0 else 0

    @staticmethod
    def vertical_acceleration(holding_angle: float, user_acceleration: LinearAcceleration) -> float:
        """Calculates the vertical acceleration of the device in (g), minus Earth gravitation"""

        return (user_acceleration[2] * math.sin(holding_angle) + user_acceleration[1] * math.cos(holding_angle)) / 981

    def update_trajectory(self, step: Point):
        step_length = 0.75

        self.position.x += step_length * -math.cos(math.radians(step.z))
        self.position.y += step_length * math.sin(math.radians(step.z))

        self.positions.append(Point(self.position.x, self.position.y, self.position.z))

    @staticmethod
    def connect_pozyx() -> PozyxSerial:
        serial_port = get_first_pozyx_serial_port()

        if serial_port is None:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")

        return PozyxSerial(serial_port)
