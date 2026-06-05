from .tag import Tag
from .containers import Coordinates, Angles, DeviceCoordinates
from .dw_1000 import DW1000

from typing import Literal


class BitcrazeTag(Tag):
    """
    Defines the UWB tag interface for a Bitcraze Loco Positioning Deck (Tag). 
    The LPD is effectively a DW1000 on a board, so this class is based on the DW1000 class. 
    However, it's designed to communicate with Loco Positioning Nodes from the BC ecosystem as anchors. 

    Methods are adapted from abstractclass Tag. Always instantiate with a context manager for graceful closing of DW1000 connection. 
    Refer to Tag class for typehints and docstrings, except when overwritten for clarity. 
    
    Attributes: 
        TODO 
    """
    def __init__(self, tag_id:int, dw1000_bus:int, dw1000_cs:int):
        if not tag_id>10: 
            raise Exception("Invalid tag_id for BitcrazeTag. Tag ID must > 10. Check your config file.")
        
        self._id = tag_id 
        self._dw = DW1000(dw1000_bus, dw1000_cs)
        self.device_list = [] 

        print(f"[OK] SUCCESSFULLY CONNECTED TO BC DEVICE") 
        print(f"[OK] TAG ID: {self._id}")

    def __enter__(self): 
        return self 
    
    def __exit__(self, exc_type, exc_val, exc_tb): 
        self._dw.close() 
        print("DW1000 closed with BitcrazeTag.__exit__()")

    @property
    def tag_id(self)->int:
        return self._id

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id: int) -> bool:
        # NOTE: for BC as of 2026-05-26, I am defining anchors as devices with IDs <=10 
        # Tags must have any other IDs. BC supports 0-255 in hexadecimal 
        return device_id<=10 

    def addDevice(self, device:DeviceCoordinates) -> None:
        self.device_list.append(device)

    def clearDevices(self) -> None:
        self.device_list = [] 

    def resetSystem(self) -> None:
        self._dw.soft_reset() 
        print("[INFO] BitcrazeTag was reset")

    def printCurrentError(self, function_name:str) -> bool:
        # NOTE: This function cannot be used in the same way as originally intended with Pozyx
        # as bitcraze does not allow for the same functionality in collecting errors through the firmware
        # To avoid modifying the rest of the code and altering pozyx compatibility, I'm keeping 
        # this function as a simple formatter that outputs the function name where the error occurred. 
        # Since we can't check if there was really an error, I assume this function is only called when there is one 
        # and so the function always returns True. 
        print(f"[ERROR] There was an error in the function: {function_name}")
        return True 
    
    def get_device_list(self, discovery_type:Literal["all", "anchor", "tag"]) -> list[int]:
        """
        Gets the list of IDs of devices seen by the tag. 

        Args: 
            discovery_type: String specifying what type of device to return
        
        Returns:
            List of corresponding device IDs (ints) 
        """

    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------
    def sendData(self, destination:int, payload:bytes) -> bool: 
        """
        Transmits data from the tag to a destination. 

        Args:
            destination: destination id (int)
            payload: data to send (bytes)

        Returns:
            Bool on whether it succeeded
        """
    
    def receiveData(self) -> tuple[int, bytes]:
        """
        Reads data received by the tag. 

        Returns: 
            tuple[sender_id (int), data (bytes)]
        """

    ### -------------------------------------------- LOCALIZATION --------------------------------------------
    def configureAnchorSelection(self, number_of_anchors:int) -> None:
        """
        Configures how many and which anchors are used for positioning the tag.
        Should be called with the total number of anchors that are currently available. We can then use them all or a more efficient subset. 
        Only used in task.py: it's updated if there's more than 3 available anchors

        Args:
            number_of_anchors: int specifying how many anchors are available for positioning
        """
    
    def setCoordinates(self, coord_list:list[int]) -> None:
        """
        Manually sets the position of the tag. 

        Args:
            coord_list: List of ints [x, y, z] representing the position of the tag
        """

    def getCoordinates(self) -> Coordinates | None: 
        """
        Gets the coordinates of the device. 
        Does not trigger positioning, only retrieves last known coordinates. 

        Returns:
            Coordinates object of the last known position 
        """

    def getOrientation(self) -> Angles | None: 
        """ 
        Gets the current orientation of the tag in degrees. 
        
        Returns:
            Angles object of the current orientation (heading, roll, pitch)
        """

    def doPositioning(self) -> Coordinates | None:
        """
        Positions the tag in space with UWB ranging. 
        This function computes and stores the position in the tag's memory. 

        Returns:
            Coordinates object with the position or None
        """

    def doRanging(self, target_id:int) -> Coordinates | None: 
        """
        Calculates a UWB range measurement between the tag and another device. 
        
        Args:
            target_id: ID of the target device (int) 
        
        Returns:
            Coordinates object with the position or None
        """ 