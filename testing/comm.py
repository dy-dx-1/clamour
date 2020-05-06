
from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port, Data)

import time


class comm(object):
    """Continuously performs ranging between the Pozyx and a destination and sets their LEDs"""

    def __init__(self, pozyx, destination_id, range_step_mm=1000,
                 remote_id=None):
        self.pozyx = pozyx
        self.destination_id = destination_id
        self.range_step_mm = range_step_mm
        self.remote_id = remote_id
        self.count = 0
        self.lastTime = time.time()

    def setup(self):
        """Sets up both the ranging and destination Pozyx's LED configuration"""
        print("------------POZYX COMM V{} -------------".format(version))
        print("NOTES: ")
        print(" - Change the parameters: ")
        print("\tdestination_id(target device)")
        print("\trange_step(mm)")
        print("")
        print("- Approach target device to see range and")
        print("led control")
        print("")
        if self.remote_id is None:
            for device_id in [self.remote_id, self.destination_id]:
                self.pozyx.printDeviceInfo(device_id)
        else:
            for device_id in [None, self.remote_id, self.destination_id]:
                self.pozyx.printDeviceInfo(device_id)
        print("Setup Done")


    def loop(self):
        status = POZYX_SUCCESS
        self.count += 1;
        tosend = [255] * 8
        self.pozyx.sendData(destination=0, data=Data(tosend, 'BBBBBBBBB'))
        #print("send ", self.count)
        #status &= self.pozyx.setLed(3, True)
        #time.sleep(0.5)
        #status &= self.pozyx.setLed(3, False)
        time.sleep(0.003)
        if self.count%1000 == 0:
            curtime = time.time()
            print("Send frequency ", 1000/(curtime-self.lastTime ))
            self.lastTime = curtime

        pass



if __name__ == "__main__":
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.


    # the easier way
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    # hardcoded way to assign a serial port of the Pozyx
    #serial_port = '/dev/ttyACM0'

    destination_id = 0x1111      # network ID of the ranging destination

    print("port",serial_port)
    pozyx = PozyxSerial(serial_port)
    r = comm(pozyx, destination_id)
    r.setup()
    while True:
        r.loop()