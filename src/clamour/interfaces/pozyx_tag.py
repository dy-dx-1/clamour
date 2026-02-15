from .tag import Tag
from pypozyx import PozyxSerial, get_first_pozyx_serial_port, Data
from pypozyx.definitions.registers import POZYX_NETWORK_ID

def connect_pozyx() -> PozyxSerial:
    serial_port = get_first_pozyx_serial_port()

    if serial_port is None:
        raise Exception("No Pozyx connected. Check your USB cable or your driver.")

    return PozyxSerial(serial_port)


def get_pozyx_id(pozyx) -> int:
    """
    Read and return the Pozyx device's network ID as an int.

    POZYX_NETWORK_ID is a register address, not the ID value itself. This
    function reads the 2-byte network ID from the device, combines the
    little-endian bytes, and returns the resulting 16-bit integer.

    The returned value uniquely identifies the device within a Pozyx
    network and can be used as a stable application-level identifier.
    """
    data = Data([0] * 2)
    pozyx.getRead(POZYX_NETWORK_ID, data)

    return data[1] * 256 + data[0]

class PozyxTag(Tag):
    def __init__(self):
        self.pozyx_serial = connect_pozyx()
        self._id = get_pozyx_id(self.pozyx_serial)
        
    @property
    def tag_id(self) -> int:
        return self._id
    
    def setCoordinates(self, coord_list:list):
        """
        Takes in a list defining the position of the tag [x,y,z] 
        and stores the coords in the object. 
        Each coordinate is expected to be an int. 

        NOTE: this is passed in ekfManager.py as [int(self.ekf.get_position().x), int(self.ekf.get_position().y), int(self.ekf.get_position().z)] 
        where self.ekf is an instance of CustomEKF
        TODO: need to construct an alternative to Coordinates object from pozyx
        """
        self.pozyx_serial.setCoordinates(coord_list)

    def clearDevices(self):
        """
        Uses the PozyxSerial library to clear the devices
        """ 
        self.pozyx_serial.clearDevices() 
