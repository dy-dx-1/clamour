import math

import matplotlib.pyplot as plt
import numpy as np

from pypozyx import PozyxSerial, get_first_pozyx_serial_port, LinearAcceleration, EulerAngles
from scipy.signal import butter, lfilter, freqz
from scipy.fftpack import fft
from time import perf_counter, sleep, time
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
        self.buffer = [Point(0, 0, 0)] * 20

    def run(self):
        print("Getting samples...")
        self.get_acceleration_samples()

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

    def get_acceleration_samples(self):
        start_time = perf_counter()

        previous_angles = np.array([0.0, 0.0, 0.0, 0.0])

        for i in range(100):
            for j in range(20):
                linear_acceleration = LinearAcceleration()
                self.pozyx.getAcceleration_mg(linear_acceleration)

                angles = EulerAngles()
                self.pozyx.getEulerAngles_deg(angles)
                yaw = angles[0]

                if self.jump(previous_angles[-1], yaw):
                    previous_angles = [yaw] * 4

                filtered_yaw = self.filter(previous_angles, yaw) \
                    if self.jump(previous_angles[-1], yaw) and i >= len(previous_angles) - 1 \
                    else yaw
                previous_angles = np.append(previous_angles[1:], filtered_yaw)

                current_acceleration = self.vertical_acceleration(self.holding_angle(), linear_acceleration)
                self.buffer = np.append(self.buffer[1:], [Point(perf_counter() - start_time, current_acceleration, yaw)])

                sleep(0.01)

            start = time()
            self.get_peaks_from_accelerations()
            delta = time() - start
            print(f"Time for finding steps within buffer: {delta}")

        self.display()

    @staticmethod
    def jump(prev, new):
        """Checks whether the orientation has changed by more than 20 degrees"""
        return abs(prev - new) > 20

    @staticmethod
    def filter(previous_yaws: np.ndarray, new_yaw: float):
        filtering_weights = np.array([0.01, 0.02, 0.03, 0.04, 0.9])

        return np.dot(filtering_weights, np.append(previous_yaws, new_yaw))

    def get_peaks_from_accelerations(self):
        current_step = 0
        next_step = Pedometer.next_index(self.buffer, self.buffer[current_step])

        while next_step:
            self.steps.append(self.buffer[next_step])
            self.update_trajectory(self.buffer[next_step])

            current_step = next_step
            next_step = Pedometer.next_index(self.buffer, self.buffer[current_step])

    @staticmethod
    def next_index(accelerations: list, current: Point) -> int:
        min_delay = 0.175

        for i, acc in enumerate(accelerations):
            local_accelerations = accelerations[i-4:i+5]
            if acc.x >= current.x + min_delay and acc.y > 1.175 and Pedometer.zero_crossing(local_accelerations, i):
                return i

    @staticmethod
    def zero_crossing(local_accelerations: list, current_index: int) -> bool:
        if current_index < 4:
            return False

        previous_smaller = [previous.y < local_accelerations[4].y for previous in local_accelerations[1:4]]
        subsequent_smaller = [subsequent.y < local_accelerations[4].y for subsequent in local_accelerations[5:8]]

        return all(previous_smaller) and all(subsequent_smaller)

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
