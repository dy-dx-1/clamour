"""
Provides a tag interface reference for Clamour
This allows the code to be hardware agnostic. 
All tag classes should follow this blueprint to work with the rest of the codebase.
"""
from abc import ABC, abstractmethod
from .containers import Coordinates, DeviceCoordinates, Angles
from typing import Literal

class Tag(ABC):
    """
    Reference class for interfacing with UWB tags in Clamour. 
    All tag classes need to follow this blueprint to work with the rest of the code.
    NOTE: All of these methods, except is_anchor, need to be called with the appropriate lock context manager to ensure thread safety. 
    """

    @property
    @abstractmethod
    def tag_id(self) -> int:
        """
        Return the tag's unique identifier as an integer.
        """

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    @abstractmethod
    def is_anchor(device_id: int) -> bool:
        """
        Evaluates if a particular device is a UWB anchor. 
        Does not require the use of Lock. 

        Args: 
            device_id: Int ID of the device to check 

        Returns:
            Bool of the result
        """

    @abstractmethod
    def addDevice(self, device:DeviceCoordinates) -> None:
        """
        Adds an anchor or tag to the tag's internal device list.

        Args:
            device: DeviceCoordinates object representing a UWB device. 
        """

    @abstractmethod
    def clearDevices(self) -> None:
        """
        Clears the tag's internal device list.
        """

    @abstractmethod
    def resetSystem(self) -> None:
        """
        Resets the tag. 
        """

    @abstractmethod
    def printCurrentError(self, function_name:str) -> bool:
        """
        Checks if the tag experienced an error and retrieves it. 
        If there was an error, prints it to the terminal & the function name where it happened.
        
        Args:
            function_name: String indicating the function name where this was checked
        Returns: 
            Bool on whether there was really an error or not 
        """
    
    @abstractmethod
    def get_device_list(self, discovery_type:Literal["all", "anchor", "tag"]) -> list[int]:
        """
        Gets the list of IDs of devices seen by the tag. 

        Args: 
            discovery_type: String specifying what type of device to return
        
        Returns:
            List of corresponding device IDs (ints) 
        """

    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------
    @abstractmethod
    def sendData(self, destination:int, payload:bytes) -> bool: 
        """
        Transmits data from the tag to a destination. 

        Args:
            destination: destination id (int)
            payload: data to send (bytes)

        Returns:
            Bool on whether it succeeded
        """
    
    @abstractmethod
    def receiveData(self) -> tuple[int, bytes]:
        """
        Reads data received by the tag. 

        Returns: 
            tuple[sender_id (int), data (bytes)]
        """

    ### -------------------------------------------- LOCALIZATION --------------------------------------------

    @abstractmethod
    def configureAnchorSelection(self, number_of_anchors:int) -> None:
        """
        Configures how many and which anchors are used for positioning the tag.
        Should be called with the total number of anchors that are currently available. We can then use them all or a more efficient subset. 
        Only used in task.py: it's updated if there's more than 3 available anchors

        Args:
            number_of_anchors: int specifying how many anchors are available for positioning
        """
    
    @abstractmethod
    def setCoordinates(self, coord_list:list[int]) -> None:
        """
        Manually sets the position of the tag. 

        Args:
            coord_list: List of ints [x, y, z] representing the position of the tag
        """

    @abstractmethod
    def getCoordinates(self) -> Coordinates | None: 
        """
        Gets the coordinates of the device. 
        Does not trigger positioning, only retrieves last known coordinates. 

        Returns:
            Coordinates object of the last known position 
        """

    @abstractmethod
    def getOrientation(self) -> Angles | None: 
        """ 
        Gets the current orientation of the tag in degrees. 
        
        Returns:
            Angles object of the current orientation (heading, roll, pitch)
        """

    @abstractmethod
    def doPositioning(self) -> Coordinates | None:
        """
        Positions the tag in space with UWB ranging. 
        This function computes and stores the position in the tag's memory. 

        Returns:
            Coordinates object with the position or None
        """

    @abstractmethod
    def doRanging(self, target_id:int) -> Coordinates | None: 
        """
        Calculates a UWB range measurement between the tag and another device. 
        
        Args:
            target_id: ID of the target device (int) 
        
        Returns:
            Coordinates object with the position or None
        """