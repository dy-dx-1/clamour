import queue
import socket
import struct
from threading import Thread

import numpy as np
import matplotlib.animation
import matplotlib.pyplot as plt

MAX_INDEX = 12


class Animation:
    def __init__(self):
        self.fig = plt.figure(num="Live CLAMOUR visualization", figsize=(14, 11))

        self.ax01 = plt.subplot2grid((2, 2), (0, 0))
        self.ax02 = plt.subplot2grid((2, 2), (0, 1))
        self.ax03 = plt.subplot2grid((2, 2), (1, 0))
        self.ax04 = plt.subplot2grid((2, 2), (1, 1))

        self.axes_limits = {"x": (-10000.0, 10000.0), "y": (-10000.0, 10000.0), "yaw": (0.0, 360), "time": (0.0, 20.0)}

        self.set_plot_presentation()

        # Placeholder data
        self.t = np.zeros(0)
        self.x_unfiltered = np.zeros(0)
        self.x_filtered = np.zeros(0)
        self.y_unfiltered = np.zeros(0)
        self.y_filtered = np.zeros(0)
        self.yaw_unfiltered = np.zeros(0)
        self.yaw_filtered = np.zeros(0)

        self.p000, = self.ax01.plot(self.x_filtered, self.y_filtered, "b-")
        self.p010, = self.ax02.plot(self.t, self.x_filtered, "b-")
        self.p011, = self.ax02.plot(self.t, self.x_unfiltered, "g-")
        self.p100, = self.ax03.plot(self.t, self.y_filtered, "b-")
        self.p101, = self.ax03.plot(self.t, self.y_unfiltered, "g-")
        self.p110, = self.ax04.plot(self.t, self.yaw_filtered, "b-")
        self.p111, = self.ax04.plot(self.t, self.yaw_unfiltered, "g-")

        self._queue = None
        self.stop = False

    def set_plot_presentation(self):
        self.fig.suptitle("Live data from tag")
        self.set_plot_titles()
        self.set_plot_grids()
        self.set_plot_axes()
        self.set_axes_limits()

    def set_plot_titles(self):
        self.ax01.set_title("Cartesian position")
        self.ax02.set_title("Filtered vs Unfiltered X")
        self.ax03.set_title("Filtered vs Unfiltered Y")
        self.ax04.set_title("Filtered vs Unfiltered Yaw")

    def set_plot_grids(self):
        self.ax01.grid(True)
        self.ax02.grid(True)
        self.ax03.grid(True)
        self.ax04.grid(True)

    def set_plot_axes(self):
        self.ax01.set_xlabel("X")
        self.ax02.set_xlabel("X")
        self.ax03.set_xlabel("Y")
        self.ax04.set_xlabel("Yaw")

        self.ax01.set_xlabel("Y")
        self.ax02.set_xlabel("Time")
        self.ax03.set_xlabel("Time")
        self.ax04.set_xlabel("Time")

    def set_axes_limits(self):
        self.ax01.set_xlim(self.axes_limits["x"])
        self.ax02.set_xlim(self.axes_limits["time"])
        self.ax03.set_xlim(self.axes_limits["time"])
        self.ax04.set_xlim(self.axes_limits["time"])

        self.ax01.set_ylim(self.axes_limits["y"])
        self.ax02.set_ylim(self.axes_limits["x"])
        self.ax03.set_ylim(self.axes_limits["y"])
        self.ax04.set_ylim(self.axes_limits["yaw"])

    def data_gen(self):
        while not self.stop:
            data = []
            for _ in range(MAX_INDEX):
                if not self._queue.empty():
                    data.append(struct.unpack('ffffffff', self._queue.get(block=False)))

            yield data

    def run(self, data):
        for d in data:
            if d[-1] > 0:
                print("WARNING: Filter might be diverging, because det(P) = ", d[-1], " > 0.")

            self.append_data(d)
            self.update_time_axis_limits(d[0])
            self.set_data()

            return self.p000, self.p010, self.p011, self.p010, self.p011, self.p100, self.p101, self.p110, self.p111

    def append_data(self, data):
        self.t = np.append(self.t, data[0])
        self.x_filtered = np.append(self.x_filtered, data[1])
        self.x_unfiltered = np.append(self.x_unfiltered, data[2])
        self.y_filtered = np.append(self.y_filtered, data[3])
        self.y_unfiltered = np.append(self.y_unfiltered, data[4])
        self.yaw_filtered = np.append(self.yaw_filtered, data[5])
        self.yaw_unfiltered = np.append(self.yaw_unfiltered, data[6])

    def set_data(self):
        self.p000.set_data(self.x_filtered, self.y_filtered)
        self.p010.set_data(self.t, self.x_filtered)
        self.p011.set_data(self.t, self.x_unfiltered)
        self.p100.set_data(self.t, self.y_filtered)
        self.p101.set_data(self.t, self.y_unfiltered)
        self.p110.set_data(self.t, self.yaw_filtered)
        self.p111.set_data(self.t, self.yaw_unfiltered)

    def update_time_axis_limits(self, new_time: float):
        if new_time >= self.axes_limits["time"][1]:
            print(new_time, self.axes_limits["time"])
            self.axes_limits["time"] = (self.axes_limits["time"][0] + 1.0, self.axes_limits["time"][1] + 1.0)
            self.ax02.set_xlim(self.axes_limits["time"])
            self.ax03.set_xlim(self.axes_limits["time"])
            self.ax04.set_xlim(self.axes_limits["time"])

    def animate(self, receive_queue):
        self._queue = receive_queue
        _ = matplotlib.animation.FuncAnimation(self.fig, self.run, self.data_gen, interval=200, repeat=False)
        plt.show()

        self.stop = True


class Stopper:
    def __init__(self):
        self.stop = False


def client_thread(connection, queue, stopper):
    while not stopper.stop:
        data = connection.recv(255)
        if not data:
            break
        queue.put(data)


class Connector:
    def __init__(self, host, port, receive_queue):
        self._host = host
        self._port = port
        self._socket = None
        self._connection = None
        self._queue = receive_queue
        self._stop = False
        self._stopper = Stopper()

    def run(self):
        while not self._stop:
            connection, address = self._socket.accept()
            remote_ip, remote_port = str(address[0]), str(address[1])
            print("Connected with " + remote_ip + ":" + remote_port)

            Thread(target=client_thread, args=(connection, self._queue, self._stopper)).start()

    def stop(self):
        self._stop = True

    def __enter__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self._host, self._port))
        self._socket.listen(5)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._stopper.stop = True

        if self._connection:
            self._connection.close()

        if self._socket:
            self._socket.close()


if __name__ == '__main__':
    host = ''  # Symbolic name meaning all available interfaces
    port = 10555  # Arbitrary non-privileged port

    receive_queue = queue.Queue()
    with Connector(host, port, receive_queue) as connector:
        Thread(target=connector.run).start()

        animation_ = Animation()
        animation_.animate(receive_queue)

        connector.stop()
