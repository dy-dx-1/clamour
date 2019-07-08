import math

import matplotlib.pyplot as plt
import numpy as np

from pypozyx import PozyxSerial, get_first_pozyx_serial_port, LinearAcceleration, EulerAngles
from scipy.signal import butter, lfilter, freqz
from scipy.fftpack import fft
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

    def run(self):
        print("Getting samples...")
        accelerations = self.get_acceleration_samples()
        print("Done.\nExtracting peaks...")
        peaks = self.get_peaks_from_accelerations(accelerations)
        print(f"Done. {len(peaks)} steps detected.")

        self.calculate_trajectory(peaks)

        self.display(accelerations, peaks)

    def display(self, accelerations, peaks):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        # axes = fig.add_subplot(111)
        #
        # acc_times, acc_vals = [acc.x for acc in accelerations], [acc.y for acc in accelerations]
        # peak_times, peak_vals = [acc.x for acc in peaks], [acc.y for acc in peaks]
        #
        # axes.scatter(acc_times, acc_vals, s=10, c='b', marker="s", label='accelerations')
        # axes.scatter(peak_times, peak_vals, s=10, c='r', marker="o", label='peaks')
        # plt.legend(loc='upper left')
        # plt.grid()
        # plt.show()

        x, y, time = [pos.x for pos in self.positions], [pos.y for pos in self.positions], [peak.x for peak in peaks]

        ax.scatter(x, y, time, s=10, c='r', marker="o")
        ax.set_xlabel('X coordinate')
        ax.set_ylabel('Y coordinate')
        ax.set_zlabel('Time')
        plt.grid()
        plt.show()

    def get_acceleration_samples(self):
        accelerations = np.array([], dtype=np.object)
        start_time = perf_counter()

        previous_angles = np.array([0.0, 0.0, 0.0, 0.0])

        for i in range(4000):
            linear_acceleration = LinearAcceleration()
            self.pozyx.getAcceleration_mg(linear_acceleration)

            angles = EulerAngles()
            self.pozyx.getEulerAngles_deg(angles)
            # print(angles.data)
            yaw = angles[0]

            if self.jump(previous_angles[-1], yaw):
                previous_angles = [yaw] * 4

            filtered_yaw = self.filter(previous_angles, yaw) \
                if self.jump(previous_angles[-1], yaw) and i >= len(previous_angles) - 1 \
                else yaw
            previous_angles = np.append(previous_angles[1:], filtered_yaw)

            current_acceleration = self.vertical_acceleration(self.holding_angle(), linear_acceleration)
            accelerations = np.append(accelerations, [Point(perf_counter() - start_time, current_acceleration, yaw)])

            sleep(0.01)

        return accelerations

    @staticmethod
    def jump(prev, new):
        return abs(prev - new) > 20

    @staticmethod
    def filter(previous_yaws: np.ndarray, new_yaw: float):
        filtering_weights = np.array([0.02, 0.03, 0.05, 0.05, 0.85])

        return np.dot(filtering_weights, np.append(previous_yaws, new_yaw))

    @staticmethod
    def get_peaks_from_accelerations(accelerations):
        peaks = []
        current_step = 0
        next_step = Pedometer.next_index(accelerations, accelerations[current_step])

        while next_step:
            peaks.append(accelerations[next_step])

            current_step = next_step
            next_step = Pedometer.next_index(accelerations, accelerations[current_step])

        return peaks

    @staticmethod
    def next_index(accelerations: np.ndarray, current: Point) -> int:
        min_delay = 0.175

        for i, acc in enumerate(accelerations):
            local_accelerations = accelerations[i-4:i+5]
            if acc.x >= current.x + min_delay and acc.y > 1.175 and Pedometer.zero_crossing(local_accelerations, i):
                return i

    @staticmethod
    def zero_crossing(local_accelerations: np.ndarray, current_index: int) -> bool:
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

    def calculate_trajectory(self, steps: list):
        step_length = 0.75

        for step in steps:
            self.position.x += step_length * -math.cos(math.radians(step.z))
            self.position.y += step_length * math.sin(math.radians(step.z))

            self.positions.append(Point(self.position.x, self.position.y, self.position.z))

    @staticmethod
    def connect_pozyx() -> PozyxSerial:
        serial_port = get_first_pozyx_serial_port()

        if serial_port is None:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")

        return PozyxSerial(serial_port)
