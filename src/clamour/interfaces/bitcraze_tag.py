from .tag import Tag
from .containers import Coordinates, Angles
from typing import Literal

class BitcrazeTag(Tag):
    """
    Defines the UWB tag interface for a Bitcraze Loco Positioning Node/Tag. 
    Methods are adapted from abstractclass Tag. 
    Refer to Tag class for typehints and docstrings, except when overwritten for clarity. 
    """
    def __init__(self, serial_port: str, tag_id: int):
        self.serial_port = serial_port
        self.tag_id = tag_id

    @property
    def tag_id(self): 
        return self._id 

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id: int) -> bool:
        """
        Evaluates if a particular device is a UWB anchor. 
        Does not require the use of Lock. 

        Args: 
            device_id: Int ID of the device to check 

        Returns:
            Bool of the result
        """

    def addDevice(self, device:object) -> None:
        """
        Adds an anchor or tag to the tag's internal device list.

        Args:
            device: Any appropriate object that can be handled by the tag interface in it's internal device list.
        """

    def clearDevices(self) -> None:
        """
        Clears the tag's internal device list.
        """

    def resetSystem(self) -> None:
        """
        Resets the tag. 
        """

    def printCurrentError(self, function_name:str) -> bool:
        """
        Checks if the tag experienced an error and retrieves it. 
        If there was an error, prints it to the terminal & the function name where it happened.
        
        Args:
            function_name: String indicating the function name where this was checked
        Returns: 
            Bool on whether there was really an error or not 
        """
    
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