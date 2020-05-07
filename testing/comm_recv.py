
from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port, Data, RXInfo)

from struct import error as StructError
import time, sys


class comm():
    """Continuously performs ranging between the Pozyx and a destination and sets their LEDs"""

    def __init__(self, pozyx, destination_id=0, range_step_mm=1000,
                 remote_id=None):
        self.pozyx = pozyx
        self.destination_id = destination_id
        self.range_step_mm = range_step_mm
        self.remote_id = remote_id
        self.successCount = 1
        self.lastTime = time.time()
        self.f = open("data.txt", "w+")

    def setup(self):
        """Sets up both the ranging and destination Pozyx's LED configuration"""
        print("Setup a Pozyx tag to receiving msg broadcasted by other nodes")

        if self.remote_id is None:
            for device_id in [self.remote_id, self.destination_id]:
                self.pozyx.printDeviceInfo(device_id)
        else:
            for device_id in [None, self.remote_id, self.destination_id]:
                self.pozyx.printDeviceInfo(device_id)
        print("Setup Done")


    def loop(self):

        data = Data([0,0,0,0,0,0,0,0], 'BBBBBBBB')
        info = RXInfo()
        try:
            self.pozyx.getRxInfo(info)
        except StructError as s:
            print("RxInfo crashes! ", str(s))
        if info[1] == data.byte_size:
            self.pozyx.readRXBufferData(data)
            print("received from ", info[0] , data.data)
            value = (time.time(), info[0] , data.data)
            s = str(value)
            self.f.write(s+'\n')
            self.successCount += 1

        if self.successCount % 200 == 0:
            curtime = time.time()
            print("Frequency: ", 100/(curtime-self.lastTime))
            self.lastTime = curtime
            self.successCount += 1

        pass



if __name__ == "__main__":

    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    print("port",serial_port)
    pozyx = PozyxSerial(serial_port)
    r = comm(pozyx)
    r.setup()
    while True:
        try:
            r.loop()
        except KeyboardInterrupt:
            r.f.close()
            sys.exit()