from ..custom_terminal import print 

from .tag import Tag
from .containers import Coordinates, Angles
from .dw_1000 import DW1000
from .anchors import Anchors

from typing import Literal
import time  
import struct

import numpy as np 
from scipy.optimize import least_squares

ALL_ANCHORS = Anchors().anchors_dict # Dict {id:Coordinates()} of all the known anchors 

SPEED_OF_LIGHT = 299_792_458
ANTENNA_TICK_DELAY_ANCHORS = -16395 # Antenna delay to apply to anchor range measurements in ticks. This value was roughly calibrated 2026-06-24 (CalibratingAntennaDelay.xlsx) in my backyard. TODO better calib in future.  

RANGING_BC_HEADER =    [0x41, 0xDC, 0x00, 0xCF, 0xBC]  # format expected by BC anchors. Used to generate TWR messages to them. Also use this type of header for inter-tag ranging exchanges, but just for convenience. 
INTER_TAG_HEADER =     [0x41, 0x98, 0x00, 0xCF, 0xBC]  # for our internal communication, based on default ranging messages, but uses 16bit addresses, as these are the ones we enforce in config.py

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
    def __init__(self, tag_id:int, dw1000_bus:int, dw1000_cs:int, 
                 channel:int, PRF:int, bitrate:int, 
                 preamble_length:int, preamble_code:int,
                 smart_tx_power:bool, tx_power_settings:list[int]|None=None):

        self._id = tag_id        # NOTE: It's expected that all ID's used are max 2bytes (<0xFFFF). Should be enforced by config.py
        self._twr_seq = 0        # keeps track of TWR Sequence, only access through property. 
        self._dw = DW1000(dw1000_bus, dw1000_cs, channel, PRF, bitrate, preamble_length, preamble_code, smart_tx_power, tx_power_settings)

        self._active_tags = dict()      # keeps track of nearby tags and when they were last seen 
        self._available_anchors = set() # keeps track of currently in-range anchors 
        self._pos = Coordinates()       # Tag position 
        self._orientation = Angles() 

        self.EXPECTED_RANGING_HEADER = RANGING_BC_HEADER + list(self.tag_id.to_bytes(6, 'little')) + [0xCF, 0xBC] # the header message that we expect for ranging requests sent to this tag. Generated here to avoid regenerating it every time.

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
    def available_anchors(self)->set[int]:
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
        DESTINATION_ADDR = list(target_id.to_bytes(6, 'little')   + b'\xcf\xbc') # not using self.EXPECTED_RANGING_HEADER, to allow this function to be used with diff IDs (ex: to check an expected header from another tag's perspective)
        SOURCE_ADDR =      list(self.tag_id.to_bytes(6, 'little') + b'\xcf\xbc') 
        if msg_type == 'POLL': 
            MSG_TYPE = TWR_POLL 
        elif msg_type == 'ANSWER': 
            MSG_TYPE = TWR_ANSWER 
        elif msg_type == 'FINAL':
            MSG_TYPE = TWR_FINAL
        elif msg_type == 'REPORT': 
            MSG_TYPE = TWR_REPORT
        else:
            return None 
        if TWR_seq>0xFF: 
            return None 
        return RANGING_BC_HEADER + DESTINATION_ADDR + SOURCE_ADDR + [MSG_TYPE] + [TWR_seq]
    
    def gen_comm_msg_header(self, target_id:int): 
        """
        Generates a header for inter-tag communication messages. 
        """
        HEADER = INTER_TAG_HEADER  
        DEST_ADDR =   list(target_id.to_bytes(2, 'little'))
        SOURCE_ADDR = list(self.tag_id.to_bytes(2, 'little'))
        return HEADER + DEST_ADDR + SOURCE_ADDR

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id: int)->bool:
        # NOTE: for BC as of 2026-05-26, I am defining anchors as devices with IDs <=10 
        # Tags must have any other IDs. 
        return device_id<=10 
    
    def addNeighborTag(self, tag_id:int)->None: 
        self._active_tags[tag_id] = time.perf_counter() 

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
        # Iterating through known anchors and checking who responds 
        twr_seq = self.TWR_seq 
        if discovery_type == "all" or discovery_type == "anchor": 
            for anc_id in ALL_ANCHORS: 
                poll_msg = self.gen_twr_msg_header(target_id=anc_id, msg_type='POLL', TWR_seq=twr_seq)
                self._dw.transmit(data=poll_msg, ranging=False) 
                resp = self._dw.listen() 
                if resp:
                    # Truncating to expected size because if the anchor had a position set internally, it'll make the message longer to include it. We discard this as we don't care, our anchor positions are set in config file ONLY.  
                    resp = resp[:23] 
                    # If we got a proper response, it should be the same structure, but with 
                    # source address and destination address swapped & with msg_type answer 
                    expected_msg = poll_msg[:5] + poll_msg[13:21] + poll_msg[5:13] + [TWR_ANSWER] + [twr_seq]
                    if resp == expected_msg: 
                        device_list.add(anc_id)
                        self._available_anchors.add(anc_id) # also updating our internal list 
                # Adding a little pause to make sure the DW properly resets. Else, we miss anchors. 
                time.sleep(DW_PAUSE_DELAY)

        if discovery_type == "all" or discovery_type == "tag":
            # For BC tags, the discovery is done every time we receive a message, which updates the internal list.
            # Accessing the set through the property automatically filters old tags out. 
            device_list = device_list.union(self.active_tags)

        return device_list 

    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------
    def broadcast(self, payload:list) -> bool: 
        # Using a proper header for clarity, but the destination ID is 0 because this is intended for all devices in range. 
        message = self.gen_comm_msg_header(target_id=0) + payload   
        return self._dw.transmit(data=message, ranging=False)
    
    def receive_data(self) -> tuple[int, bytes]:
        msg, R1 = self._dw.listen(ranging=True) # R1 timestamp taken by default but only used if ranging exchange
        
        if not msg: 
            return None, b'' 
        
        # Checking if it's a ranging message (time-sensitive) and responding if so 
        if msg[:13] == self.EXPECTED_RANGING_HEADER and msg[21]==TWR_POLL: 
            # Extracting sender_id to update our discovery info in messenger.py 
            sender_id = struct.unpack('<H', bytes(msg[13:15]))[0] # config.py enforces that our tag IDs <0xFFFF 
            # Do rest of ranging exchange
            self.respond_to_ranging_exchange(sender_id, msg, R1)  
            return sender_id, b'' # This function is expected to return broadcast messages, so although we respond to ranging, we return nothing as we didn't get a proper msg 
        
        # If it's not ranging, check if it's communication 
        if msg[:5] == INTER_TAG_HEADER: 
            # Extracting sender ID and data 
            # NOTE: For these types of messages, tags do general broadcasts, so we don't need to check destination ID at the moment
            sender_id = msg[7] | (msg[8]<<8)
            data = msg[9:] 
            return sender_id, bytes(data) 
        return None, b'' 
    
    def respond_to_ranging_exchange(self, requester_id:int, msg:list, R1:int): 
        """
        To be called if we received a ranging (POLL) request from a fellow tag. 
        Tries to perform the ranging request. 
        """
        # Get TWR seq of this transaction 
        twr_seq = msg[22] 
        # Respond with answer
        _, T2 = self._dw.transmit(data=self.gen_twr_msg_header(target_id=requester_id, msg_type='ANSWER', TWR_seq=twr_seq),
                                   ranging=True) 
        _, R3 = self.listen_for_TWR_msg(self._dw.DW_LISTEN_TIMEOUT, exp_src=requester_id, exp_dest=self.tag_id, exp_msg_type=TWR_FINAL, exp_twr_seq=twr_seq)
        if R3: 
            # Report consists of header + 5 bytes each for R1, T2, R3 (40bit clock timings) 
            report = self.gen_twr_msg_header(requester_id, 'REPORT', twr_seq) + list(R1.to_bytes(5, 'little') + T2.to_bytes(5, 'little') + R3.to_bytes(5, 'little'))
            self._dw.transmit(report, ranging=True)

    def listen_for_TWR_msg(self, timeout:int, exp_src:int, exp_dest:int, exp_msg_type:int, exp_twr_seq:int)->tuple[list,int]|tuple[None,None]: 
        """
        Listens until it hears a specific TWR message or timeout. 
        Checks the expected source and destination addresses, the message type and the TWR_SEQ. 

        RETURNS (None, None) if fails, else:
            - List of received data 
            - Received timestamp 
        """
        exp_src_addr = list(exp_src.to_bytes(6, 'little') + b'\xcf\xbc')
        exp_dest_addr = list(exp_dest.to_bytes(6, 'little') + b'\xcf\xbc')

        tstart = time.perf_counter() 
        while (time.perf_counter()-tstart)<timeout: 
            message, timestamp = self._dw.listen(ranging=True)

            if not message: # If dw1000 listen timed-out but we still have time, continue 
                continue 

            dest_addr = message[5:13]
            source_addr = message[13:21]
            try: 
                msg_type = message[21]
                twr_seq = message[22]
            except IndexError: 
                # If these indexes don't exist, we caught a general comm message
                # these messages are not for TWR, so ignore it 
                continue 

            if exp_dest_addr==dest_addr and exp_src_addr==source_addr and exp_msg_type==msg_type and exp_twr_seq==twr_seq:
                return message, timestamp 
        return None, None 
    
    ### -------------------------------------------- LOCALIZATION --------------------------------------------
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
        Computes the distance in mm between the tag and another device. 
        Returns None if the transaction is unsuccessful.  
        TODO: a more thorough testing of timeouts to actually figure out what is a good reliable value  
        """
        def compute_clock_delta(t2, t1): 
            ### Calculates difference in clock ticks considering DW1000 clock is 40bit
            TICK_DELTA_MASK = (1 << 40) - 1
            return (t2-t1) & TICK_DELTA_MASK

        distance = None 
        twr_seq = self.TWR_seq # for this entire ranging transaction
         
        ## Ranging exchanges with both the tag and anchors are based on Bitcraze's TWR transactions
        ## https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/twr-protocol/
        # Messages to send 
        poll = self.gen_twr_msg_header(target_id, 'POLL', twr_seq) 
        final = self.gen_twr_msg_header(target_id, 'FINAL', twr_seq)
        # Transaction  
        _, T1     = self._dw.transmit(poll, ranging=True) 
        _, R2     = self.listen_for_TWR_msg(self._dw.DW_LISTEN_TIMEOUT, exp_src=target_id, exp_dest=self.tag_id, exp_msg_type=TWR_ANSWER, exp_twr_seq=twr_seq)            
        _, T3     = self._dw.transmit(final, ranging=True) 
        report, _ = self.listen_for_TWR_msg(self._dw.DW_LISTEN_TIMEOUT, exp_src=target_id, exp_dest=self.tag_id, exp_msg_type=TWR_REPORT, exp_twr_seq=twr_seq)
        if report and R2: # Ensures transaction happened 
            # NOTE: I also check R2 as it might have returned None even if the message was sent, but never received by us. Since we send T3 anyways, the transaction might complete even if we didn't get R2. 
            # Calculating TOF 
            timing_info = report[23:38] # If talking with an anchor, the rest is pressure related info, don't care
            R1, T2, R3 = struct.unpack('<5s5s5s', bytes(timing_info))
            T_r1  = compute_clock_delta(R2, T1)
            T_r2  = compute_clock_delta(int.from_bytes(R3, byteorder='little'), int.from_bytes(T2, byteorder='little'))
            T_rp1 = compute_clock_delta(int.from_bytes(T2, byteorder='little'), int.from_bytes(R1, byteorder='little'))
            T_rp2 = compute_clock_delta(T3, R2)
            tof_ticks = ((T_r1 * T_r2) - (T_rp1 * T_rp2)) / (T_r1+T_r2+T_rp1+T_rp2)
            distance = int((tof_ticks + ANTENNA_TICK_DELAY_ANCHORS) * self._dw.TIME_UNIT * SPEED_OF_LIGHT * 1000) # in mm 
        return distance

    def trilaterate_position(self) -> Coordinates | None:
        anchors = [] 
        distances = [] 
        for anchor_id in self.available_anchors:  
            dist = self.compute_range(anchor_id) 
            if dist: 
                anchors.append([ALL_ANCHORS[anchor_id].x, ALL_ANCHORS[anchor_id].y, ALL_ANCHORS[anchor_id].z])
                distances.append(dist)
        if len(anchors)<3: 
            return None # Could not get the minimum amount of range measurements needed 
        # Residual function (Error = Calculated Distance - Measured Distance)
        def equations(position):
            calculated_distances = np.linalg.norm(anchors - position, axis=1)
            return calculated_distances - distances
        # Solving with Non-linear Least Squares (Levenberg-Marquardt or Trust Region Reflective)
        initial_guess = np.array(self.coordinates.data)  # using last known position as guess
        result = least_squares(equations, initial_guess, method='lm')  
        
        if not result.success:
            print("[WARNING] trilaterate_position() did not converge perfectly!", 'info', 'loc')
            
        return Coordinates(result.x[0], result.x[1], result.x[2])

    def doRanging(self, target_id:int) -> Coordinates | None: 
        distance = self.compute_range(target_id) 
        return Coordinates(x=distance, y=0, z=0) if distance else None 