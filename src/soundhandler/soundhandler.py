import socket
import psutil
import subprocess as sp


class SoundSegFaultHandler(object):
    def __init__(self, popen_call):
        self.popen_call = popen_call
        self.process = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(("127.0.0.1", 50012))
        self.socket.listen(1)
        self.conn = None

    def connect(self):
        self.process = sp.Popen(self.popen_call)
        self.conn, self.addr = self.socket.accept()
        self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2)
#        print(self.conn.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF))

    def check(self):
        if not self.is_process_alive():
            print("Process died")
            self.process.terminate()
            del self.process
            self.process = sp.Popen(self.popen_call)
            self.conn, _ = self.socket.accept()
            self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2)

        # pos = self.main_func()

    def send_player(self,pos):
        if pos is None:
            return
        else:
            pos_st = "ST{0};{1};{2}".format(int(pos[0]/10), int(pos[1]/10), int(pos[2]/10)).encode('utf-8')
        try:
            self.conn.sendall(pos_st)
#                print("Sent: ", pos)
        except:
            print("Thread Process died")

    def is_process_alive(self):
        try:
            process = psutil.Process(self.process.pid)
            # print(process.status())
            if process.status() in ["running", "sleeping", "disk-sleep"]:
                return True
        except:
            return False
        return False
