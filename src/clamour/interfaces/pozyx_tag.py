from .tag import Tag
from .containers import Coordinates, DeviceCoordinates, Angles
import struct 

from pypozyx import PozyxSerial 
from pypozyx.definitions.registers import POZYX_NETWORK_ID
from pypozyx import (get_first_pozyx_serial_port, POZYX_3D, POZYX_ANCHOR_SEL_AUTO,
                     POZYX_POS_ALG_UWB_ONLY, POZYX_SUCCESS, POZYX_DISCOVERY_ALL_DEVICES)

from pypozyx import Coordinates as pozyxCoordinates
from pypozyx import DeviceCoordinates as pozyxDeviceCoordinates 
from pypozyx import DeviceList as pozyxDeviceList
from pypozyx import DeviceRange, EulerAngles, SingleRegister, Data, RXInfo

def get_pozyx_id(pozyx:PozyxSerial) -> int:
    """
    Read and return the Pozyx device's network ID (16bit based) as an int.
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
        print("[ERROR] in PozyxTag.get_nb_devices: ", str(s))

    # returns status like POZYX_SUCCESS, this is handled by get_device_list
    return status, size[0]

class PozyxTag(Tag):
    """
    Defines the UWB tag interface for a PozyxDevice. 
    Methods are adapted from abstractclass Tag. 
    Refer to Tag class for typehints and docstrings, except when overwritten for clarity. 
    """
    def __init__(self, id:int):
        serial_port = get_first_pozyx_serial_port()
        if serial_port is None:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")
        self._pozyx_serial = PozyxSerial(serial_port)
        self._id = id 
        if self._id != get_pozyx_id(self._pozyx_serial): 
            self._pozyx_serial.setNetworkId(id)
        print(f"[OK] Successfully initialized pozyx tag on port {serial_port} with id: {self._id}")
        print(f"This ID means that the constant NB_NODES in timing.py must >{self._id & 0xFF} for the node to do slot proposals.")

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.clearDevices() 
        self.resetSystem()
        print("PozyxTag.__exit__() completed.")

    @property
    def tag_id(self):
        return self._id
    
    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id):
        # NOTE: noticed this check doesn't really work (see pers. notes from ~2026-04-30 & 2026-05-01)
        # since moving away from pozyx, the following hardcoded ID checks are just to get through this and 
        # allow me to check localization features 
        testing_tags = [27182, 27199]
        if device_id in testing_tags: 
            return True 
        return device_id < 0x500
    
    def addDevice(self, device_coordinates:DeviceCoordinates): 
        # Only used in task.py, this is passed as self.anchors.anchors_dict[anchor], without expecting a return
        # DeviceCoordinates is our general version, needs to be formatted to pypozyx version 
        net_id = device_coordinates.network_id
        fl = device_coordinates.flag
        coords = device_coordinates.pos # General coordinates object, needs to be converted 
        pozyx_obj = pozyxDeviceCoordinates(network_id=net_id, flag=fl, pos = pozyxCoordinates(coords.x, coords.y, coords.z) )
        self._pozyx_serial.addDevice(pozyx_obj)
    
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
            print("[ERROR] in PozyxTag.printCurrentError: ", str(s))  

        if error_code != 0x0: 
            print(f"[ERROR] printCurrentError found an error in {function_name} : {message}")
            returned_error = True 
        return returned_error

    def get_device_list(self, discovery_type):  
        status = self._pozyx_serial.doDiscovery(discovery_type=POZYX_DISCOVERY_ALL_DEVICES)
        if status != POZYX_SUCCESS:
            self.printCurrentError("PozyxSerial.doDiscovery")
            return [] # NOTE: there wasn't any return before, function would have just continued with an error
        
        status, size = get_nb_devices(self._pozyx_serial)
        devices = pozyxDeviceList(list_size=size)

        if (status == POZYX_SUCCESS) and (size > 0):
            try:
                self._pozyx_serial.getDeviceIds(devices)
            except struct.error as s:
                print("[ERROR] in PozyxTag.getDeviceIds: ", str(s))
        elif status != POZYX_SUCCESS:
            self.printCurrentError("PozyxTag.get_nb_devices")

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
        
    def receiveData(self): 
        metadata = RXInfo() 
        try: 
            self._pozyx_serial.getRxInfo(metadata) 
        except struct.error as s: 
            print("[ERROR] in PozyxTag.receiveData, RxInfo crashes: ", str(s))
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
    def configureAnchorSelection(self, number_of_anchors):
        # With pypozyx, we use automatic anchor selection: https://ardupozyx.readthedocs.io/en/latest/api/pozyx_functions.html#group__positioning__functions_1ga41fc706bd9ffba1d8483cdbeb01d1a75
        # We tell the device how many anchors are available, and it automatically chooses from them to balance precision and performance
        self._pozyx_serial.setSelectionOfAnchors(mode=POZYX_ANCHOR_SEL_AUTO, number_of_anchors=number_of_anchors)
    
    def setCoordinates(self, coord_list):
        self._pozyx_serial.setCoordinates(coord_list)

    def getCoordinates(self): 
        # Not sure why pypozyx seems to only get the X position with this function. 
        # If precision is way off in the future, look into this. 
        coords_container = pozyxCoordinates() # Need to pass a pozyx Coords object in the pypozyx method 
        try:
            status = self._pozyx_serial.getCoordinates(coords_container) # if successful, this'll update the ref_coordinates value
        except struct.error as s: 
            status = 0 
            print("[ERROR] in PozyxTag.getCoordinates: ", str(s)) 
        assert status == POZYX_SUCCESS # There's no status check in task.py where this is used so if it is not successful we should add one 
        return Coordinates(coords_container.x, coords_container.y, coords_container.z) # Converting to our general object         

    def getOrientation(self): 
        angles = EulerAngles() # pozyx object 
        try:
            status = self._pozyx_serial.getEulerAngles_deg(angles)
        except struct.error as s: 
            status = 0
            print("[ERROR] in PozyxTag.getOrientation: ", str(s)) 
        
        if status == POZYX_SUCCESS: 
            return Angles(heading=angles.heading, roll=angles.roll, pitch=angles.pitch)
        else: 
            return None

    def doPositioning(self):
        pos = pozyxCoordinates() 
        try: 
            status = self._pozyx_serial.doPositioning(position=pos, dimension=POZYX_3D, algorithm=POZYX_POS_ALG_UWB_ONLY)
        except struct.error as s: 
            status = 0
            print("[ERROR] in PozyxTag.doPositioning: ", str(s))

        if status == POZYX_SUCCESS: 
            return Coordinates(pos.x, pos.y, pos.z)
        else: 
            return None 

    def doRanging(self, target_id): 
        range_measure = DeviceRange() 
        try: 
            status = self._pozyx_serial.doRanging(target_id, range_measure)
        except struct.error as s: 
            status = 0 
            print("[ERROR] in PozyxTag.doRanging: ", str(s)) 

        if status == POZYX_SUCCESS: 
            return Coordinates(range_measure.data[1], 0, 0) # idk why only along X 
        else: 
            return None