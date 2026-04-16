"""
Provides a tag interface reference for Clamour
This allows the code to be hardware agnostic. 
All tag classes should follow this blueprint to work with the rest of the codebase.
"""
from abc import ABC, abstractmethod
from .containers import Coordinates, DeviceCoordinates, Angles
class Tag(ABC):
    @abstractmethod
    def __init__(self):
        pass 

    @property
    @abstractmethod
    def tag_id(self) -> int:
        """
        Return the tag's unique identifier as an integer.
        """
        pass

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @abstractmethod
    @staticmethod
    def is_anchor(device_id: int) -> bool:
        """
        Return True if the device_id corresponds to an anchor
        """
        pass

    @abstractmethod
    def addDevice(self, device) -> None:
        """
        Adds an anchor or tag to the tag's internal device list
        """
        pass

    @abstractmethod
    def clearDevices(self) -> None:
        """
        Clears the tag's internal device list
        """
        pass 

    @abstractmethod
    def resetSystem(self) -> None:
        """
        Resets the tag
        """
        pass 

    @abstractmethod
    def printCurrentError(self, function_name:str) -> bool:
        """
        Verifies if the tag has experienced an error and retrieves it. 
        Then, prints the error to the terminal & the function name where it happened.
        Returns True if there was indeed an error, and False if not.
        """
        pass 
    
    # TODO: pozyxdiscoverer stuff

    ### -------------------------------------------- COMMUNICATION --------------------------------------------
    @abstractmethod
    def sendData(self, destination:int, payload:bytes) -> bool: 
        """
        Transmits data from the tag to a destination. 
        Args:
        - destination: destination id (int)
        - payload: data to send (bytes)
        Returns:
        - True if succeeded and False if not. 
        """
        pass 
    
    @abstractmethod
    def receiveData(self) -> tuple[int, bytes]:
        """
        Reads data received by the tag. 
        Returns: 
        - Sender id (int) 
        - Received data (bytes) 
        """
        pass 

    ### -------------------------------------------- LOCALIZATION --------------------------------------------

    @abstractmethod
    def setSelectionOfAnchors(self, number_of_anchors:int)->None:
        """
        TODO
        """
    
    @abstractmethod
    def doPositioning(self)->Coordinates|None:
        """
        Gets the positioning of the tag. 
        Returns:
        - Coordinates object with the position or None
        """
        pass 

    @abstractmethod
    def setCoordinates(self, coord_list:list)->None:
        """
        Defines the position of the tag
        Args:
        - coord_list: List of ints defining the position of the tag [x,y,z]
        """
        pass 
