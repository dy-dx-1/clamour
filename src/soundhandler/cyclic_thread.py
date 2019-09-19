import socket  # Import socket module
from pygame_funcs import PyGameManager

BUFFER_SIZE = 4480

class Position(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def main():
    pygame_manager = PyGameManager()
    s = socket.socket()  # Create a socket object
    old_position = None

    s.connect(("127.0.0.1", 50012))
    s.settimeout(0.1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, int(BUFFER_SIZE/2))
    received_size = s.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF) + 2
    print("Starting pygame manager with buffer size", received_size)
    while 1:
        pos = b""
        valid_pos = True
        position = old_position
        try:
            pos = s.recv(received_size)
        except:
            valid_pos = False

        if valid_pos and pos != b"":
            try:
                pos = pos.decode('utf-8')
                #print("Received: ", pos)
                pos = pos.split("ST")[-1].split(";")
                position = Position(float(pos[0]), float(pos[1]), float(pos[2]))
            except Exception as e:
                print(e)
                position = old_position

        if position is not None:
            pygame_manager.cyclic_call(position)


if __name__ == '__main__':
    main()
