"""
Provides a tag interface reference for Clamour
This allows the code to be hardware agnostic. 
All tag classes should follow this blueprint to work with the rest of the codebase.
"""
from abc import ABC, abstractmethod
from .containers import Coordinates, Angles
from typing import Literal

class Tag(ABC):
    """
    Reference class for interfacing with UWB tags in Clamour. 
    All tag classes need to follow this blueprint to work with the rest of the code.
    NOTE: All of these methods, except is_anchor, need to be called with the appropriate lock context manager to ensure thread safety. 
    """

    @abstractmethod
    def __enter__(self): 
        """
        Tags must be used with context managers to ensure proper disconnection.
        """
    
    @abstractmethod
    def __exit__(self, exc_type, exc_val, exc_tb): 
        """
        Tags must be used with context managers to ensure proper disconnection.
        """

    @property
    @abstractmethod
    def tag_id(self) -> int:
        """
        Tag's unique network identifier (int).
        """

    @property
    @abstractmethod
    def active_tags(self) -> set[int]: 
        """
        Set of the IDs of the current active tags in the neighborhood.

        This method can implement a DISCOVERY_TIMEOUT that allows it to return 
        only devices that have been active recently. See BitcrazeTag implementation.  
        """
    
    @property 
    @abstractmethod 
    def available_anchors(self) -> set[int]: 
        """ 
        Set of the IDs of the anchors that this tag can currently reach. 
        """

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    @abstractmethod
    def is_anchor(device_id: int) -> bool:
        """
        Evaluates if a particular device is a UWB anchor. 
        Does not require the use of Lock. 

        ARGS: 
            - device_id: Int ID of the device to check 
        """

    @abstractmethod
    def addNeighborTag(tag_id: int) -> None: 
        """
        Adds a neighboring tag to the tag's internal active tags dictionary. 

        Used in Messenger.receive_new_message(), whenever we receive a valid message. 
        The dictionary should keep timestamp info to allow for correct return of property active_tags. 
        """

    @abstractmethod
    def clearAnchors(self) -> None:
        """
        Clears the list of available anchors. 
        """

    @abstractmethod
    def resetSystem(self) -> None:
        """
        Resets the tag. 
        """
    
    @abstractmethod
    def get_device_list(self, discovery_type:Literal["all", "anchor", "tag"]) -> set[int]:
        """
        Updates the internal sets of available anchors and tags and returns a list of their IDs. 
        This must never return duplicated elements. 

        ARGS: 
            - discovery_type: String specifying what type of device to return
        
        RETURNS:
            - Set of corresponding device IDs (ints) 
        """

    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------
    @abstractmethod
    def broadcast(self, payload:list) -> bool: 
        """
        Transmits data from the tag to all in-range devices. 
        To do this, the tag should transmit a message with '0' as destination ID. 

        ARGS:
            - payload: list of bytes to send in LSB format 

        RETURNS:
            - Bool on whether it succeeded
        """
    
    @abstractmethod
    def receive_data(self) -> tuple[int, bytes]:
        """
        Reads data received by the tag. 

        RETURNS: 
            - tuple[sender_id (int), data (bytes)]
        """

    ### -------------------------------------------- LOCALIZATION --------------------------------------------

    @property
    @abstractmethod
    def coordinates(self) -> Coordinates: 
        """
        The last-known position of the tag. 
        """

    @coordinates.setter 
    @abstractmethod
    def coordinates(self, new_coords:Coordinates) -> None: 
        pass

    @property
    def orientation(self) -> Angles: 
        """ 
        Gets the current orientation of the tag in degrees. 
        
        RETURNS: 
            - Angles object of the current orientation (heading, roll, pitch)
        """
    
    @abstractmethod
    def trilaterate_position(self) -> Coordinates | None:
        """
        Uses trilateration (or multilateration if possible) to position the tag in 3D space. 
        This should only be called if >3 anchors are available. 
        Stores the new position in the tag's memory and returns it. 

        RETURNS:
            - Coordinates object with the position or None
        """

    @abstractmethod
    def doRanging(self, target_id:int) -> Coordinates | None: 
        """
        Calculates a UWB range measurement between the tag and another device. 
        Measurement must be in mm. 
        
        ARGS:
            - target_id: ID of the target device (int) 
        
        RETURNS:
            - Coordinates(x=distance_to_device, y=0, z=0) or None
        """