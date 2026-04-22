from .tag import Tag
from .containers import Coordinates, DeviceCoordinates, Angles
import struct 

from pypozyx import PozyxSerial 
from pypozyx.definitions.registers import POZYX_NETWORK_ID
from pypozyx import (get_first_pozyx_serial_port, POZYX_3D, POZYX_ANCHOR_SEL_AUTO,
                     POZYX_POS_ALG_UWB_ONLY, POZYX_SUCCESS, POZYX_DISCOVERY_ALL_DEVICES)

from pypozyx import Coordinates as pozyxCoordinates
from pypozyx import DeviceList as pozyxDeviceList
from pypozyx import DeviceRange, EulerAngles, SingleRegister, Data, RXInfo


def get_pozyx_id(pozyx:PozyxSerial) -> int:
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

def get_nb_devices(pozyx:PozyxSerial) -> tuple:
    """
    Get's the size of the pozyx internal list of added devices 
    """
    size = SingleRegister()

    try: 
        status = pozyx.getDeviceListSize(size)
    except struct.error as s:
        status = 0
        print(str(s))

    # returns status like POZYX_SUCCESS, this is handled by get_device_list
    return status, size[0]

class PozyxTag(Tag):
    """
    Defines the UWB tag interface for a PozyxDevice. 
    Methods are adapted from abstractclass Tag. 
    Refer to Tag class for typehints and docstrings, except when overwritten for more clarity. 
    """
    def __init__(self):
        serial_port = get_first_pozyx_serial_port()
        if serial_port is None:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")

        self._pozyx_serial = PozyxSerial(serial_port)
        self._id = get_pozyx_id(self._pozyx_serial)

    @property
    def tag_id(self):
        return self._id
    
    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id: int):
        return device_id < 0x500
    
    def addDevice(self, device_coordinates:DeviceCoordinates): 
        # Only used in task.py, this is passed as self.anchors.anchors_dict[anchor], without expecting a return
        self._pozyx_serial.addDevice(device_coordinates)
    
    def clearDevices(self):
        self._pozyx_serial.clearDevices() 

    def resetSystem(self): 
        self._pozyx_serial.resetSystem()
    
    def printCurrentError(self, function_name): 
        returned_error = False
        try: 
            error_code = SingleRegister() 
            self._pozyx_serial.getErrorCode(error_code) 
            message = self._pozyx_serial.getErrorMessage(error_code) 
        except struct.error as s: 
            message = "" 
            print(str(s))  
        if error_code != 0x0: 
            print(f"Error in {function_name} : {message}")
            returned_error = True 
        return returned_error

    def get_device_list(self, discovery_type):  
        status = self._pozyx_serial.doDiscovery(discovery_type=POZYX_DISCOVERY_ALL_DEVICES)
        if status != POZYX_SUCCESS:
            self.printCurrentError("discover")
            return [] # NOTE: there wasn't any return before, function would have just continued with an error
        
        status, size = get_nb_devices(self._pozyx_serial)
        devices = pozyxDeviceList(list_size=size)

        if (status == POZYX_SUCCESS) and (size > 0):
            try:
                self._pozyx_serial.getDeviceIds(devices)
            except struct.error as s:
                print(str(s))
        elif status != POZYX_SUCCESS:
            self.printCurrentError("get_nb_devices")

        if discovery_type == "tag":
            devices = [device_id for device_id in devices if not PozyxTag.is_anchor(device_id)]
        elif discovery_type == "anchor":
            devices = [device_id for device_id in devices if PozyxTag.is_anchor(device_id)]
        elif discovery_type == "all": 
            devices = [device_id for device_id in devices] 
        else: 
            raise ValueError("discovery_type arg must be 'all' or 'tag' or 'anchor'")

        return devices

    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------

    def sendData(self, destination, payload): 
        values = list(payload)
        data_to_send = Data(values, 'B'*len(values)) # converting the bytes into a pozyx Data object
        status = self._pozyx_serial.sendData(destination=destination, data=data_to_send)
        if status == POZYX_SUCCESS: 
            return True
        else:
            return False
        
    def receiveData(self) -> tuple[int, bytes]: 
        """
        Reads data received by a tag. 
        Returns: 
        - The sender id (int)
        - The data (in bytes)  
        """
        metadata = RXInfo() 
        try: 
            self._pozyx_serial.getRxInfo(metadata) 
        except struct.error as s: 
            print("RxInfo crashes! ", str(s))
            # NOTE: there wasn't any other error handling in the past 
        sender_id, message_byte_size = metadata[0], metadata[1] 

        if message_byte_size == 0:
            return sender_id, b''
        
        p_data = Data([0]*message_byte_size, 'B'*message_byte_size)
        if message_byte_size == p_data.byte_size:
            self._pozyx_serial.readRXBufferData(p_data)

        data = bytes(p_data.data) # .data is a list of ints representing bytes values

        return sender_id, data 

    ### -------------------------------------------- LOCALIZATION --------------------------------------------
    
    def setSelectionOfAnchors(self, number_of_anchors:int)->None:
        """
        Configures how many anchors are used for positioning and how they are selected.

        With pypozyx, we use automatic anchor selection 
        For more details, see https://ardupozyx.readthedocs.io/en/latest/api/pozyx_functions.html#group__positioning__functions_1ga41fc706bd9ffba1d8483cdbeb01d1a75

        Only used in task.py once if there's more than 3 available anchors, without needing a return value
          this is passed as self.tag.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors.available_anchors))
        NOTE: same return problem as doPositioning 
        """
        self._pozyx_serial.setSelectionOfAnchors(mode=POZYX_ANCHOR_SEL_AUTO, number_of_anchors=number_of_anchors)
    
    def doPositioning(self):
        """
        Positions the tag with respect to it's UWB anchors. 

        To use the pypozyx library to do this, we need:
         - 'position' container of type Coordinates
         - dimension is POZYX3D (which is int(3))
         - algorithm is POZYX_POS_ALG_UWB_ONLY 
        """
        pos = pozyxCoordinates() 
        try: 
            status = self._pozyx_serial.doPositioning(position=pos, dimension=POZYX_3D, algorithm=POZYX_POS_ALG_UWB_ONLY)
        except struct.error as s: 
            status = 0
            print(str(s))

        if status == POZYX_SUCCESS: 
            self._coordinates = Coordinates(pos.x, pos.y, pos.z) # TODO: still need to define if I need this internal attribute, maybe for other tags?
            return Coordinates(pos.x, pos.y, pos.z)
        else: 
            return None 
    
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

    def getCoordinates(self)->Coordinates: 
        """
        Stores the coordinates of the Pozyx in a Coordinates container. 
        Only used in task.py 
        """
        # NOTE: Figure out if/how to use self._coordinates here in general version 
        # Also not sure why pypozyx seems to only get the X position with this function. 
        # If precision is way off in the future, look into this. 
        coords_container = pozyxCoordinates() # Need to pass a pozyx Coords object in the pypozyx method 
        try:
            status = self._pozyx_serial.getCoordinates(coords_container) # if successful, this'll update the ref_coordinates value
        except struct.error as s: 
            status = 0 
            print(str(s)) 
        assert status == POZYX_SUCCESS # There's no status check in task.py where this is used so if it is not successful we should add one 
        return Coordinates(coords_container.x, coords_container.y, coords_container.z) # Converting to our general object         

    def doRanging(self, target_id:int)->Coordinates: 
        """
        Calculates a range measurement between the tag and another device
        Only used in task.py
        NOTE: in task.py, after the Coordinates object is returned, it is fed into the ekf, should check that our general Coordinates object does work 
        NOTE: check how to generalize target_id
        """
        range_measure = DeviceRange() 
        try: 
            status = self._pozyx_serial.doRanging(target_id, range_measure)
        except struct.error as s: 
            status = 0 
            print(s) 

        if status == POZYX_SUCCESS: 
            return Coordinates(range_measure.data[1], 0, 0) # idk why only along X 
        else: 
            return None

    def getOrientation(self)->Angles: 
        """ 
        Gets the device's current orientation in degrees (heading, roll, pitch) 
        Returns a general Angles object with the data
        """
        angles = EulerAngles() # pozyx object 
        try:
            status = self._pozyx_serial.getEulerAngles_deg(angles)
        except struct.error as s: 
            status = 0
            print(s) 
        
        if status == POZYX_SUCCESS: 
            return Angles(heading=angles.heading, roll=angles.roll, pitch=angles.pitch)
        else: 
            return None