import math

import matplotlib.pyplot as plt
import numpy as np
from pypozyx import PozyxSerial, get_first_pozyx_serial_port, LinearAcceleration
from time import perf_counter


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __gt__(self, other):
        return self.y > other.y

    def __ge__(self, other):
        return self.y > other.y


class Pedometer:
    def __init__(self):
        self.pozyx = self.connect_pozyx()

    def run(self):
        print("Getting samples...")
        accelerations = self.get_acceleration_samples()
        print("Done.\nExtracting peaks...")
        peaks = self.get_peaks_from_accelerations(accelerations)
        print(f"Done. {len(peaks)} steps detected.")

        self.display(accelerations, peaks)

    @staticmethod
    def display(accelerations, peaks):
        fig = plt.figure()
        axes = fig.add_subplot(111)

        acc_times, acc_vals = [acc.x for acc in accelerations], [acc.y for acc in accelerations]
        peak_times, peak_vals = [acc.x for acc in peaks], [acc.y for acc in peaks]

        axes.scatter(acc_times, acc_vals, s=10, c='b', marker="s", label='accelerations')
        axes.scatter(peak_times, peak_vals, s=10, c='r', marker="o", label='peaks')
        plt.legend(loc='upper left')
        plt.show()

    def get_acceleration_samples(self):
        accelerations = np.array([], dtype=np.object)
        start_time = perf_counter()

        for _ in range(40000):
            linear_acceleration = LinearAcceleration()
            self.pozyx.getAcceleration_mg(linear_acceleration)

            current_acceleration = self.vertical_acceleration(self.holding_angle(), linear_acceleration)
            accelerations = np.append(accelerations, [Point(perf_counter() - start_time, current_acceleration)])

        return accelerations

    @staticmethod
    def get_peaks_from_accelerations(accelerations):
        peaks = []
        current_step = 0
        next_step = Pedometer.next_index(accelerations, accelerations[current_step])

        while next_step:
            subsample = accelerations[current_step:next_step]
            peaks.append(subsample.max())

            current_step = next_step
            next_step = Pedometer.next_index(accelerations, accelerations[current_step])

        return peaks

    @staticmethod
    def next_index(accelerations, current) -> int:
        min_delay = 0.175

        for i, acc in enumerate(accelerations):
            if acc.x >= current.x + min_delay and acc.y > 1.15:
                return i

    def holding_angle(self) -> float:
        gravity = LinearAcceleration()
        self.pozyx.getGravityVector_mg(gravity)

        return math.atan(abs(gravity[2]/gravity[1]))

    @staticmethod
    def vertical_acceleration(holding_angle: float, user_acceleration: LinearAcceleration) -> float:
        """Calculates the vertical acceleration of the device in (g), minus Earth gravitation"""

        return (user_acceleration[2] * math.sin(holding_angle) + user_acceleration[1] * math.cos(holding_angle)) / 981

    @staticmethod
    def connect_pozyx() -> PozyxSerial:
        serial_port = get_first_pozyx_serial_port()

        if serial_port is None:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")

        return PozyxSerial(serial_port)
