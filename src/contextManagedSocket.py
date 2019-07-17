import struct
from socket import socket


class ContextManagedSocket:
    def __init__(self, remote_host: str, port: int):
        self.socket = socket()
        self.remote_host = remote_host
        self.port = port

    def __enter__(self):
        self.socket.connect((self.remote_host, self.port))
        print("Socket connected.")

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.socket.close()

    def send(self, data: list) -> None:
        message = struct.pack('%sf' % len(data), *data)
        self.socket.send(message)
