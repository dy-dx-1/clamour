from .tag import Tag
from pypozyx import PozyxSerial, get_first_pozyx_serial_port
from pypozyx.definitions.registers import POZYX_NETWORK_ID

from pypozyx import (POZYX_3D, POZYX_ANCHOR_SEL_AUTO, POZYX_DISCOVERY_ALL_DEVICES,
                     POZYX_POS_ALG_UWB_ONLY, POZYX_SUCCESS, Coordinates, DeviceRange,
                     PozyxSerial, EulerAngles, SingleRegister, Data)

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
        self._pozyx_serial = connect_pozyx()
        self._id = get_pozyx_id(self.pozyx_serial)
        
    @property
    def tag_id(self) -> int:
        return self._id
    
    def getErrorCode(self, error_code:SingleRegister): 
        """
        Gets the error code for a pozyx device and writes it to a SingleRegister container
        NOTE: returns POZYX_SUCESS, need to address this 
        """
        return self._pozyx_serial.getErrorCode(error_code) 
    
    def getErrorMessage(self, error_code:SingleRegister): 
        """
        Returns the system error string for the given error code in the SingleRegister container
        """
        return self._pozyx_serial.getErrorMessage(error_code)
    
    def setCoordinates(self, coord_list:list):
        """
        Takes in a list defining the position of the tag [x,y,z] 
        and stores the coords in the object. 
        Each coordinate is expected to be an int. 

        NOTE: this is passed in ekfManager.py as [int(self.ekf.get_position().x), int(self.ekf.get_position().y), int(self.ekf.get_position().z)] 
        where self.ekf is an instance of CustomEKF
        TODO: need to construct an alternative to Coordinates object from pozyx
        """
        self._pozyx_serial.setCoordinates(coord_list)

    def getCoordinates(self, ref_coordinates:Coordinates): 
        """
        Stores the coordinates of the Pozyx in a Coordinates container
        NOTE: Returns a pozyx success or not. needs to be updates
        this is notably used in task.py similar situatin to doPositioning
        """
        return self._pozyx_serial.getCoordinates(ref_coordinates)

    def clearDevices(self):
        """
        Uses the PozyxSerial library to clear the devices
        """ 
        self._pozyx_serial.clearDevices() 

    def addDevice(self, device_coordinates): 
        """
        Adds an anchor or tag to the Pozyx device list
        see Pozyx lib for more detail on device_coordinates
        In task.py this is passed as self.anchors.anchors_dict[anchor]
        NOTE: same return problem as doPositioning
        """
        return self._pozyx_serial.addDevice()
    
    def setSelectionOfAnchors(self, mode, number_of_anchors):
        """
        Sets the anchor positioning for the anchors. 
        In task.py this is passed as self.tag.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors.available_anchors))
        NOTE: same return problem as doPositioning 
        """
        return self._pozyx_serial.setSelectionOfAnchors(mode, number_of_anchors)

    def sendData(self, destination:int, data:Data): 
        """  
        Uses PozyxSerial lib to send a Data object from pypozyx to a destination
        NOTE: will have to figure out how to adapt or replace Data object for general implementation
        in states/initialization.py, this destination is a certain id and data=Data([0], 'i')
        in states/task.py this is (destination=0, data=Data(tosend, 'BBBBBBBBB'))
        """
        self._pozyx_serial.sendData(destination=destination, data=data)

    def doPositioning(self, position:Coordinates, dimension:int, algorithm_type:int):
        """
        Uses pypozyx to position a UWB tag. This is very tightly coupled with pozyx. 
        In task.py, this is used with:
         - 'position' container of type Coordinates
         - dimension is POZYX3D (which is int(3))
         - algorithm is POZYX_POS_ALG_UWB_ONLY 

        IMPORTANT NOTE!!! this returns POZYX_SUCCESS and it's variations and seemingly just writes the result to the pozyx hardware. Need to find a decoupled way to track this. 
        """
        return self._pozyx_serial.doPositioning(position, dimension, algorithm_type)
    
    def getEulerAngles_deg(self, angles:EulerAngles): 
        """ 
        Uses pypozyx to read the angles of the tag and store them in a EulerAngles container
        IMPORTANT NOTE!!! this returns POZYX_SUCCESS and it's variations and seemingly just writes the result to the pozyx hardware. Need to find a decoupled way to track this. 
        """
        return self._pozyx_serial.getEulerAngles_deg(angles)
    
    def doRanging(self, ranging_target_id:int, measured_position:DeviceRange): 
        """
        Gets a ranging measurement and stores it in the DeviceRange object 
        NOTE: same situation for the return as doPositioning 
        """
        return self._pozyx_serial.doRanging(ranging_target_id, measured_position)
    
