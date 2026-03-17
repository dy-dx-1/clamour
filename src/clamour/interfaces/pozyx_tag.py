from .tag import Tag
from pypozyx import PozyxSerial, get_first_pozyx_serial_port
from pypozyx.definitions.registers import POZYX_NETWORK_ID

from pypozyx import (POZYX_3D, POZYX_ANCHOR_SEL_AUTO, POZYX_DISCOVERY_ALL_DEVICES,
                     POZYX_POS_ALG_UWB_ONLY, POZYX_SUCCESS, DeviceRange,
                     PozyxSerial, EulerAngles, SingleRegister, Data, RXInfo)
import struct 

from .containers import Coordinates, DeviceCoordinates

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
        serial_port = get_first_pozyx_serial_port()
        if serial_port is None:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")

        self._pozyx_serial = PozyxSerial(serial_port)
        self._id = get_pozyx_id(self._pozyx_serial)
        self._coordinates = None # Initialised by setCoordinates, NOTE: implement as property in future? 
        self._internal_device_list = [] # Used by addDevice, clearDevices

    @property
    def tag_id(self) -> int:
        return self._id
    
    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    def addDevice(self, device_coordinates:DeviceCoordinates): 
        """
        Adds an anchor or tag to the Pozyx device list
        see Pozyx lib for more detail on device_coordinates
        In task.py this is passed as self.anchors.anchors_dict[anchor]
        NOTE: same return problem as doPositioning
        """
        self._internal_device_list.append(device_coordinates) # Not needed for pozyx, putting it here to remind me general integration 
        return self._pozyx_serial.addDevice(device_coordinates)
    
    def clearDevices(self):
        """
        Uses the PozyxSerial library to clear the devices
        """ 
        self._internal_device_list = [] # Not needed for pozyx, putting it here for general integration 
        self._pozyx_serial.clearDevices() 

    def resetSystem(self): 
        """
        Resets the Pozyx device 
        """
        self._pozyx_serial.resetSystem()
    
    def printCurrentError(self, function_name:str) -> None: 
        """
        Gets the current error for a pozyx device and prints it out. 
        function_name allows to specify where the error happened 
        """
        try: 
            error_code = SingleRegister() 
            self._pozyx_serial.getErrorCode(error_code) 
            message = self._pozyx_serial.getErrorMessage(error_code) 
        except struct.error as s: 
            message = "" 
            print(str(s))  
        if error_code != 0x0: 
            print(f"Error in {function_name} : {message}")
            
                

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
    
    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------

    def sendData(self, destination:int, data:Data): 
        """  
        Uses PozyxSerial lib to send a Data object from pypozyx to a destination
        NOTE: will have to figure out how to adapt or replace Data object for general implementation
        in states/initialization.py, this destination is a certain id and data=Data([0], 'i')
        in states/task.py this is (destination=0, data=Data(tosend, 'BBBBBBBBB'))
        in messenger.py this is (destination=0, data=Data([0xAA, message.data], 'BI'))
        """
        self._pozyx_serial.sendData(destination=destination, data=data)

    def readRXBufferData(self, data:Data): 
        """
        Uses PozyxSerial to read the pozyx's buffer and put it in the data container 
        NOTE: same implementation notes 
        This seems only to be used in messenger.py? to check 
        """
        self._pozyx_serial.readRXBufferData(data) 

    def getRxInfo(self, info:RXInfo): 
        """
        Gets metadata on information the Pozyx received over UWB and writes it to an Rx Info container
        NOTE: Same implementation notes + this also only seems to be used in messenger.py? 
        """
        self._pozyx_serial.getRxInfo(info) 

    ### -------------------------------------------- LOCALIZATION --------------------------------------------
    
    def setSelectionOfAnchors(self, mode, number_of_anchors):
        """
        Sets the anchor positioning for the anchors. 
        In task.py this is passed as self.tag.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors.available_anchors))
        NOTE: same return problem as doPositioning 
        """
        return self._pozyx_serial.setSelectionOfAnchors(mode, number_of_anchors)
    
    def doPositioning(self, position:Coordinates, dimension:int, algorithm_type:int):
        """
        Uses pypozyx to position a UWB tag. This is very tightly coupled with pozyx. 
        Only used in task.py with:
         - 'position' container of type Coordinates
         - dimension is POZYX3D (which is int(3))
         - algorithm is POZYX_POS_ALG_UWB_ONLY 

        IMPORTANT NOTE!!! this returns POZYX_SUCCESS and it's variations and seemingly just writes the result to the pozyx hardware. Need to find a decoupled way to track this. 
        """
        status = self._pozyx_serial.doPositioning(position, dimension, algorithm_type)
        self._coordinates = position 
        return status # TODO: get rid of statuses 
    
    def setCoordinates(self, coord_list:list):
        """
        Takes in a list defining the position of the tag [x,y,z] 
        and stores the coords in the object. 
        Each coordinate is expected to be an int. 

        NOTE: this is passed in ekfManager.py as [int(self.ekf.get_position().x), int(self.ekf.get_position().y), int(self.ekf.get_position().z)] 
        where self.ekf is an instance of CustomEKF
        """
        self._coordinates = Coordinates(*coord_list)
        self._pozyx_serial.setCoordinates(coord_list)

    def getCoordinates(self, ref_coordinates:Coordinates): 
        """
        Stores the coordinates of the Pozyx in a Coordinates container
        NOTE: Returns a pozyx success or not. needs to be updates
        this is notably used in task.py similar situatin to doPositioning
        """
        # NOTE: This is only used in states/task.py
        # I am not fully sure of my implementation. Currently ASSUMING the self._coordinates value is up-to-date when this function is called.
        # Also not sure why pypozyx seems to only get the X position with this function. 
        # If precision is way off in the future, look into this. 
        status = self._pozyx_serial.getCoordinates(ref_coordinates) # success or not, this'll update the ref_coordinates value
        assert ref_coordinates == self._coordinates # NOTE: this should confirm above assumption, remove after verifying. else can setCoords right after to enforce it?
        return status # TODO: stop returning statuses 

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
    
