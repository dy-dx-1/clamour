from ..custom_terminal import print 

from .tag import Tag
from .containers import Coordinates, Angles, DeviceCoordinates
from .dw_1000 import DW1000
from ..config import ANCHORS

from typing import Literal
import time  

TWR_POLL   = 0x01
TWR_ANSWER = 0x02 
TWR_FINAL  = 0x03 
TWR_REPORT = 0x04

DW_PAUSE_DELAY = 0.005    # Delay used to give some time for the DW to reset between loops 
DW_LISTEN_TIMEOUT = 0.015 # Delay used to wait during RX 

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
    def __init__(self, tag_id:int, dw1000_bus:int, dw1000_cs:int, channel:int, PRF:int, bitrate:int, preamble_length:int, preamble_code:int):
        if not tag_id>10: 
            raise Exception("Invalid tag_id for BitcrazeTag. Tag ID must > 10. Check your config file.")
        
        self._id = tag_id 
        self._twr_seq = 0  # keeps track of TWR Sequence, only access through property. 
        self._dw = DW1000(dw1000_bus, dw1000_cs, channel, PRF, bitrate, preamble_length, preamble_code)
        self.device_list = [] 

        print(f"SUCCESSFULLY CONNECTED TO BC DEVICE", 'ok', 'device') 
        print(f"TAG ID: {self._id}", 'ok', 'device')

    def __enter__(self): 
        return self 
    
    def __exit__(self, exc_type, exc_val, exc_tb): 
        self._dw.close() 
        print("DW1000 closed with BitcrazeTag.__exit__()", 'ok', 'device')

    @property
    def tag_id(self)->int:
        return self._id
    
    @property
    def TWR_seq(self)->int: 
        """ 
        TWR sequence identifier. Keeps track of each TWR transaction between 2 particular devices. 
        Should be kept constant for the entirety of a TWR exchange between the devices. 
        Only access through this property to ensure the counter prevents duplicate identifiers for separate transactions.
        """
        self._twr_seq = (self._twr_seq + 1) & 0xFF  # Go from 0 to 255 and then restart 
        return self._twr_seq 

    def gen_message_header(self, target_id:int, msg_type:Literal['POLL', 'ANSWER', 'FINAL', 'REPORT'], TWR_seq:int)->list: 
        """ 
        Generates the first 7 sections needed for a message from this tag to another BC device.

        These are: [Frame Control (2 bytes), SEQ (1byte), PAN ID (2bytes), Destination Addr (8bytes), Source Addr (8bytes), Msg Type (1 byte), TWR SEQ (1 byte)].

        It's expected the caller of this function will add any remaining info if using REPORT type. 

        Returns None if unsupported args are passed. 
        """
        BC_MESSAGE_HEADER = [0x41, 0xDC, 0x00, 0xCF, 0xBC] 
        SOURCE_ADDR =      [self.tag_id>>(shift*8) & 0xFF for shift in range(6)] + [0xCF, 0xBC] 
        DESTINATION_ADDR = [target_id  >>(shift*8) & 0xFF for shift in range(6)] + [0xCF, 0xBC]
        if msg_type == 'POLL': 
            MSG_TYPE = [TWR_POLL] 
        elif msg_type == 'ANSWER': 
            MSG_TYPE = [TWR_ANSWER] 
        elif msg_type == 'FINAL':
            MSG_TYPE = [TWR_FINAL]
        elif msg_type == 'REPORT': 
            MSG_TYPE = [TWR_REPORT]
        else:
            return None 
        if TWR_seq>0xFF: 
            return None 
        return BC_MESSAGE_HEADER + DESTINATION_ADDR + SOURCE_ADDR + MSG_TYPE + [TWR_seq]

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id: int) -> bool:
        # NOTE: for BC as of 2026-05-26, I am defining anchors as devices with IDs <=10 
        # Tags must have any other IDs. 
        return device_id<=10 

    def addDevice(self, device:DeviceCoordinates) -> None:
        self.device_list.append(device)

    def clearDevices(self) -> None:
        self.device_list = [] 

    def resetSystem(self) -> None:
        self._dw.soft_reset() 
        print("BitcrazeTag was reset", 'info', 'device')

    def printCurrentError(self, function_name:str) -> bool:
        # NOTE: This function cannot be used in the same way as originally intended with Pozyx
        # as bitcraze does not allow for the same functionality in collecting errors through the firmware
        # To avoid modifying the rest of the code and altering pozyx compatibility, I'm keeping 
        # this function as a simple formatter that outputs the function name where the error occurred. 
        # Since we can't check if there was really an error, I assume this function is only called when there is one 
        # and so the function always returns True. 
        print(f"There was an error in the function: {function_name}", 'error', 'gen')
        return True 
    
    def get_device_list(self, discovery_type:Literal["all", "anchor", "tag"]) -> list[int]:
        """
        Gets the list of IDs of devices seen by the tag. 

        Args: 
            discovery_type: String specifying what type of device to return
        
        Returns:
            List of corresponding device IDs (ints) 
        """
        device_list = [] 
        # Iterating through known (config.py) anchors and checking who responds 
        twr_seq = self.TWR_seq 
        if discovery_type == "all" or discovery_type == "anchor": 
            for anchor in ANCHORS: 
                poll_msg = self.gen_message_header(target_id=anchor['id'], msg_type='POLL', TWR_seq=twr_seq)
                self._dw.transmit(data=poll_msg, ranging=False) 
                resp = self._dw.listen(timeout=DW_LISTEN_TIMEOUT) 
                if resp:
                    # Truncating to expected size because if the anchor had a position set internally, it'll make the message longer to include it. We discard this as we don't care, our anchor positions are set in config file ONLY.  
                    resp = resp[:23] 
                    # If we got a proper response, it should be the same structure, but with 
                    # source address and destination address swapped & with msg_type answer 
                    expected_msg = poll_msg[:5] + poll_msg[13:21] + poll_msg[5:13] + [TWR_ANSWER] + [twr_seq]
                    if resp == expected_msg: 
                        device_list.append(anchor['id'])
                # Adding a little pause to make sure the DW properly resets. Else, we miss anchors. 
                time.sleep(DW_PAUSE_DELAY)

        if discovery_type == "all" or discovery_type == "tag":
            pass # TODO 
        return device_list 

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