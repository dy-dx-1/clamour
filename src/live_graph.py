import csv
import time
import queue
import socket
import struct
from threading import Thread

import matplotlib.animation as animation
import matplotlib.pyplot as plt

COLORS = ["red", "blue", "green", "black"]
MAX_INDEX = 12
CSV_FOLDER = "./EKFlogs"


class Line:
    def __init__(self):
        self.line = None
        self.xdata, self.ydata = [], []


class Animation:
    def __init__(self):
        self._fig, self._ax = plt.subplots()
        self._lines = []
        for i in range(MAX_INDEX):
            line = Line()
            line.line, = self._ax.plot([], [], lw=2, color=COLORS[i % (len(COLORS))])
            self._lines.append(line)
        self._ax.grid()
        self._ymin, self._ymax = 0, 0
        self._xmin, self._xmax = 0, 0
        self._max_points = 15
        self._queue = None
        self.stop = False
        self._csvfile = open("{}/log_{}.csv".format(CSV_FOLDER, time.strftime("%Y%m%d-%H%M%S")), 'w')
        self._writer = csv.writer(self._csvfile, delimiter=',', lineterminator='\n')

    def data_gen(self):
        while not self.stop:
            datas = []
            for _ in range(MAX_INDEX):
                try:
                    data = self._queue.get(False, None)
                except queue.Empty:
                    continue
                except KeyboardInterrupt:
                    break
                try:
                    state = struct.unpack('fff', data)
                except:
                    continue
                self._writer.writerow(state)
                if state[2] > 0:  # det(P) > means system is diverging
                    print(int(state[0]), int(state[1]), 1/state[2])
                x = state[0]
                y = state[1]

                datas.append([0, x, y])

            yield datas

    def init(self):
        self._ax.set_ylim(-1.1, 1.1)
        self._ax.set_xlim(0, 10)

        for line in self._lines:
            del line.xdata[:]
            del line.ydata[:]
            line.line.set_data(line.xdata, line.ydata)

        return self._lines,

    def run(self, datas):
        # update the data
        for data in datas:
            index, x, y = data

            line = self._lines[index]
            line.xdata.append(x)
            line.ydata.append(y)

            if len(line.xdata) > self._max_points:
                line.xdata.pop(0)
                line.ydata.pop(0)

            if y < self._ymin:
                self._ymin = y * 0.65 if y > 0 else y * 1.5
                self._ax.set_ylim(self._ymin, self._ymax)
                self._ax.figure.canvas.draw()

            if y > self._ymax:
                self._ymax = y * 1.5 if y > 0 else y * 0.65
                self._ax.set_ylim(self._ymin, self._ymax)
                self._ax.figure.canvas.draw()

            if x < self._xmin:
                self._xmin = x * 0.65 if x > 0 else x * 1.5
                self._ax.set_xlim(self._xmin, self._xmax)
                self._ax.figure.canvas.draw()

            if x > self._xmax:
                self._xmax = x * 1.5 if x > 0 else x * 0.65
                self._ax.set_xlim(self._xmin, self._xmax)
                self._ax.figure.canvas.draw()

            line.line.set_data(line.xdata, line.ydata)

        return self._lines,

    def animate(self, receive_queue):
        self._queue = receive_queue
        ani = animation.FuncAnimation(self._fig, self.run, self.data_gen, blit=False, interval=50,
                                      repeat=False, init_func=self.init)
        plt.show()

        self.stop = True

    def __del__(self):
        self._csvfile.close()


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
        self.stop = False
        self._stopper = Stopper()

    def run(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((HOST, PORT))
        self._socket.listen(5)

        while not self.stop:
            connection, address = self._socket.accept()
            ip, port = str(address[0]), str(address[1])
            print("Connected with " + ip + ":" + port)

            try:
                Thread(target=client_thread, args=(connection, self._queue, self._stopper)).start()
            except:
                print("Thread did not start.")

        self._stopper.stop = True

    def __del__(self):
        if self._connection:
            self._connection.close()

        if self._socket:
            self._socket.close()


if __name__ == '__main__':
    HOST = ''  # Symbolic name meaning all available interfaces
    PORT = 10555  # Arbitrary non-privileged port

    receive_queue = queue.Queue()
    connector = Connector(HOST, PORT, receive_queue)
    Thread(target=connector.run).start()

    animation_ = Animation()
    animation_.animate(receive_queue)

    connector.stop = True
