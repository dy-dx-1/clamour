from ..custom_terminal import print 

from .tag import Tag
from .containers import Coordinates, Angles
from .dw_1000 import DW1000
from ..config import ANCHORS

from typing import Literal
import time  
import struct

SPEED_OF_LIGHT = 299_792_458
ANTENNA_TICK_DELAY_ANCHORS = -16395 # Antenna delay to apply to anchor range measurements in ticks. This value was roughly calibrated 2026-06-24 (CalibratingAntennaDelay.xlsx) in my backyard. TODO better calib in future.  

ANCHOR_TAG_BC_HEADER = [0x41, 0xDC, 0x00, 0xCF, 0xBC]  # format expected by BC anchors. Used to generate TWR messages to them. 
INTER_TAG_BC_HEADER =  [0x41, 0x98, 0x00, 0xCF, 0xBC]  # for our internal communication, based on default tag-anchor message, but with 16bit addresses 

TWR_POLL   = 0x01
TWR_ANSWER = 0x02 
TWR_FINAL  = 0x03 
TWR_REPORT = 0x04

DW_PAUSE_DELAY = 0.005    # Delay used to give some time for the DW to reset between loops 

DISCOVERY_TIMEOUT = 5     # If we don't hear from another tag after this time, we consider it inactive 

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
        self._twr_seq = 0        # keeps track of TWR Sequence, only access through property. 
        self._dw = DW1000(dw1000_bus, dw1000_cs, channel, PRF, bitrate, preamble_length, preamble_code)

        self._active_tags = {}       # keeps track of nearby tags and when they were last seen 
        self._available_anchors = {} # keeps track of currently in-range anchors 
        self._pos = Coordinates()    # Tag position 
        self._orientation = Angles() 

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
    def active_tags(self)->set[int]:
        now = time.perf_counter() 
        return {tag_id for tag_id, last_seen in self._active_tags.items() if (now-last_seen)<DISCOVERY_TIMEOUT}
    
    @property
    def available_anchors(self):
        return self._available_anchors 
    
    @property
    def TWR_seq(self)->int: 
        """ 
        TWR sequence identifier. Keeps track of each TWR transaction between 2 particular devices. 
        Should be kept constant for the entirety of a TWR exchange between the devices. 
        Only access through this property to ensure the counter prevents duplicate identifiers for separate transactions.
        """
        self._twr_seq = (self._twr_seq + 1) & 0xFF  # Go from 0 to 255 and then restart 
        return self._twr_seq 

    def gen_twr_msg_header(self, target_id:int, msg_type:Literal['POLL', 'ANSWER', 'FINAL', 'REPORT'], TWR_seq:int)->list: 
        """ 
        Generates the first 7 sections needed for a message from this tag to a BC anchor.

        These are: [Frame Control (2 bytes), SEQ (1byte), PAN ID (2bytes), Destination Addr (8bytes), Source Addr (8bytes), Msg Type (1 byte), TWR SEQ (1 byte)].

        It's expected the caller of this function will add any remaining info if using REPORT type. 

        Returns None if unsupported args are passed. 
        """
        BC_MESSAGE_HEADER = ANCHOR_TAG_BC_HEADER  
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
    
    def gen_comm_msg_header(self, target_id:int): 
        """
        Generates a header for inter-tag communication messages. 
        TODO add ID value check in init and config file to prevent overflow 
        """
        HEADER = INTER_TAG_BC_HEADER  
        SOURCE_ADDR = [(self.tag_id & 0xFF), (self.tag_id>>8 & 0xFF)]
        DEST_ADDR =   [(target_id   & 0xFF), (self.tag_id>>8 & 0xFF)]
        return HEADER + DEST_ADDR + SOURCE_ADDR

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id: int)->bool:
        # NOTE: for BC as of 2026-05-26, I am defining anchors as devices with IDs <=10 
        # Tags must have any other IDs. 
        return device_id<=10 
    
    def addNeighborTag(self, tag_id:int)->None: 
        self._active_tags[tag_id] = time.perf_counter() 

    def addAnchor(self, anchor_id:int)->None:
        self._available_anchors.add(anchor_id)

    def clearAnchors(self) -> None:
        self._available_anchors.clear() 

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
    
    def get_device_list(self, discovery_type:Literal["all", "anchor", "tag"]) -> set[int]: 
        device_list = set() 
        # Iterating through known (config.py) anchors and checking who responds 
        twr_seq = self.TWR_seq 
        if discovery_type == "all" or discovery_type == "anchor": 
            for anchor in ANCHORS: 
                poll_msg = self.gen_twr_msg_header(target_id=anchor['id'], msg_type='POLL', TWR_seq=twr_seq)
                self._dw.transmit(data=poll_msg, ranging=False) 
                resp = self._dw.listen() 
                if resp:
                    # Truncating to expected size because if the anchor had a position set internally, it'll make the message longer to include it. We discard this as we don't care, our anchor positions are set in config file ONLY.  
                    resp = resp[:23] 
                    # If we got a proper response, it should be the same structure, but with 
                    # source address and destination address swapped & with msg_type answer 
                    expected_msg = poll_msg[:5] + poll_msg[13:21] + poll_msg[5:13] + [TWR_ANSWER] + [twr_seq]
                    if resp == expected_msg: 
                        device_list.add(anchor['id'])
                # Adding a little pause to make sure the DW properly resets. Else, we miss anchors. 
                time.sleep(DW_PAUSE_DELAY)

        if discovery_type == "all" or discovery_type == "tag":
            device_list = device_list.union(self.active_tags)

        return device_list 

    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------
    def sendData(self, destination:int, payload:list) -> bool: 
        # never used for ranging messages, only in task.py, messenger.py and initilization.py 
        # TODO: confirm use of destination. 
        # It's only used as '0' in all calls of this function. 
        # Check how it's used (specifically how messages are received) 
        message = self.gen_comm_msg_header(target_id=destination) + payload   
        return self._dw.transmit(data=message, ranging=False)
    
    def receiveData(self) -> tuple[int, bytes]:
        msg = self._dw.listen() # TODO Eval if replace listen 
        if not msg: 
            return None, b'' 
        # Checking message is part of our network 
        if msg[:5] != INTER_TAG_BC_HEADER: 
            return None, b'' 
        # Extracting sender ID and data  # TODO add check for receive ID? requires checking why they're always using 0 in sendData
        sender_id = msg[7] | (msg[8]<<8)
        data = msg[9:] 
        return sender_id, bytes(data) 

    def listen_for_message(self, timeout:int, exp_src:int, exp_dest:int, exp_msg_type:int, exp_twr_seq:int)->tuple[list,int]|tuple[None,None]: 
        """
        Listens until it hears a specific BC message or timeout. 
        Checks the expected source and destination addresses, the message type and the TWR_SEQ. 

        RETURNS (None, None) if fails:
            - List of received data 
            - Received timestamp (always listens in ranging mode) 
        """
        tstart = time.perf_counter() 
        while (time.perf_counter()-tstart)<timeout: 
            message, timestamp = self._dw.listen(ranging=True)

            if not message: # If dw1000 listen timed-out but we still have time, continue 
                continue 

            exp_src =  [exp_src >>(shift*8) & 0xFF for shift in range(6)] + [0xCF, 0xBC] 
            exp_dest = [exp_dest>>(shift*8) & 0xFF for shift in range(6)] + [0xCF, 0xBC]
            
            dest_addr = message[5:13]
            source_addr = message[13:21]
            msg_type = message[21]
            twr_seq = message[22]

            if exp_dest==dest_addr and exp_src==source_addr and exp_msg_type==msg_type and exp_twr_seq==twr_seq:
                return message, timestamp 
        return None, None 
    
    ### -------------------------------------------- LOCALIZATION --------------------------------------------
    def configureAnchorSelection(self, number_of_anchors:int) -> None:
        # Currently don't have a specific way to determine which anchors to use 
        # Pypozyx already had an algo for this, which is why the function exists 
        # TODO: for now, simply using all available anchors. Implement somthing in future. 
        
        # NOTE: this could even simply be inside positioning method to simplify things 
        pass 

    @property
    def coordinates(self)->Coordinates: 
        return self._pos
    
    @coordinates.setter 
    def coordinates(self, new_coords:Coordinates): 
        self._pos = new_coords 

    @property
    def orientation(self):
        return self._orientation
    
    def compute_range(self, target_id:int)->int|None: 
        """
        Computes the distance in meters between the tag and another device. 
        Returns None if the transaction is unsuccessful.  
        TODO: a more thorough testing of timeouts, maybe even adding timeout param to arg of this function is needed 
        """
        def compute_clock_delta(t2, t1): 
            ### Calculates difference in clock ticks considering DW1000 clock is 40bit
            TICK_DELTA_MASK = (1 << 40) - 1
            return (t2-t1) & TICK_DELTA_MASK

        distance = None 
        twr_seq = self.TWR_seq # for this entire ranging transaction
         
        if self.is_anchor(target_id): 
            # based on https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/twr-protocol/
            # Messages to send by us
            poll = self.gen_twr_msg_header(target_id, 'POLL', twr_seq) 
            final = self.gen_twr_msg_header(target_id, 'FINAL', twr_seq)
            # Transaction with anchor 
            _, T1     = self._dw.transmit(poll, ranging=True) 
            _, R2     = self.listen_for_message(self._dw.DW_LISTEN_TIMEOUT, exp_src=target_id, exp_dest=self.tag_id, exp_msg_type=TWR_ANSWER, exp_twr_seq=twr_seq)            
            _, T3     = self._dw.transmit(final, ranging=True) 
            report, _ = self.listen_for_message(self._dw.DW_LISTEN_TIMEOUT, exp_src=target_id, exp_dest=self.tag_id, exp_msg_type=TWR_REPORT, exp_twr_seq=twr_seq)
            if report and R2: # Ensures transaction happened 
                # NOTE: I also check R2 as it might have returned None if the tag didn't hear it, but it was still sent. Since we send T3 anyways, the transaction might complete even if we didn't get R2. 
                # Calculating TOF 
                timing_info = report[23:38] # the rest is pressure related info, don't care
                R1, T2, R3 = struct.unpack('<5s5s5s', bytes(timing_info))
                T_r1  = compute_clock_delta(R2, T1)
                T_r2  = compute_clock_delta(int.from_bytes(R3, byteorder='little'), int.from_bytes(T2, byteorder='little'))
                T_rp1 = compute_clock_delta(int.from_bytes(T2, byteorder='little'), int.from_bytes(R1, byteorder='little'))
                T_rp2 = compute_clock_delta(T3, R2)
                tof_ticks = ((T_r1 * T_r2) - (T_rp1 * T_rp2)) / (T_r1+T_r2+T_rp1+T_rp2)
                distance = (tof_ticks + ANTENNA_TICK_DELAY_ANCHORS) * self._dw.TIME_UNIT * SPEED_OF_LIGHT
        else: 
            # TODO implement tag 
            pass 
        return distance

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