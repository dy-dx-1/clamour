#!/usr/bin/python3

import multiprocessing
from pedometer import Pedometer, Point


def writer(queue: multiprocessing.Queue):
    message = queue.get()
    while isinstance(message, Point):
        print(message.x, message.y)
        message = queue.get()


def main():
    communication_queue = multiprocessing.Queue()

    process = multiprocessing.Process(target=writer, args=(communication_queue,))
    process.start()

    pedometer = Pedometer(communication_queue)
    pedometer.run()

    communication_queue.close()
    communication_queue.join_thread()
    process.join()


if __name__ == '__main__':
    main()
