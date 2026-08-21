from ..custom_terminal import print 

from .tag import Tag
from .containers import Coordinates, Angles
from .dw_1000 import DW1000
from .anchors import Anchors

from typing import Literal
import time  
import struct

ALL_ANCHORS = Anchors().anchors_dict # Dict {id:Coordinates()} of all the known anchors 

SPEED_OF_LIGHT = 299_792_458
ANTENNA_TICK_DELAY_ANCHORS = -16395 # Antenna delay to apply to anchor range measurements in ticks. This value was roughly calibrated 2026-06-24 (CalibratingAntennaDelay.xlsx) in my backyard. TODO better calib in future.  
ANTENNA_TICK_DELAY_TAGS = -32873    # Antenna delay for tag range measurements. Very very very roughly calibrated 2026-07-08 on my desk. TODO NEEDS PROPER CALIBRATION (didn't have other rpi on hand at the time) 

RANGING_BC_HEADER =    [0x41, 0xDC, 0x00, 0xCF, 0xBC]  # format expected by BC anchors. Used to generate TWR messages to them. Also use this type of header for inter-tag ranging exchanges, but just for convenience. 
INTER_TAG_HEADER =     [0x41, 0x98, 0x00, 0xCF, 0xBC]  # for our internal communication, based on default ranging messages, but uses 16bit addresses, as these are the ones we enforce in config.py

TWR_POLL   = 0x01
TWR_ANSWER = 0x02 
TWR_FINAL  = 0x03 
TWR_REPORT = 0x04

REPORT_HEADER_SIZE = 23
REPORT_TIMESTAMP_SIZE = 15
REPORT_NEIGHBOR_INFO = struct.Struct('<9i')  # x, y, z, xx, yy, zz, xy, xz, yz

DW_PAUSE_DELAY = 0.005    # Delay used to give some time for the DW to reset between loops 

DISCOVERY_TIMEOUT = 15     # If we don't hear from another tag after this time, we consider it inactive 

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
        self._pos = Coordinates()       # Tag position and associated covariance
        self._orientation = Angles() 

        self.EXPECTED_RANGING_HEADER = RANGING_BC_HEADER + list(self.tag_id.to_bytes(6, 'little')) + [0xCF, 0xBC] # the header message that we expect for ranging requests sent to this tag. Generated here to avoid regenerating it every time.

        print(f"SUCCESSFULLY CONNECTED TO BC DEVICE", 'ok', 'gen') 
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
    
    def add_neighbor(self, tag_id:int)->None: 
        self._active_tags[tag_id] = time.perf_counter() 

    def clear_anchors(self) -> None:
        self._available_anchors.clear() 

    def reset(self) -> None:
        # NOTE: 2026-07-10 all testing up until now has shown that soft_reset was sufficient 
        # if a proper hard reset is required, look into implementing full reset with RSTn pin of DW1000 
        self._dw.soft_reset() 
        print("BitcrazeTag was reset", 'info', 'device')
    
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
        Tries to perform the ranging request and embeds the tag's position and covariance in the response.  
        """
        # Get TWR seq of this transaction 
        twr_seq = msg[22] 
        # Respond with answer
        _, T2 = self._dw.transmit(data=self.gen_twr_msg_header(target_id=requester_id, msg_type='ANSWER', TWR_seq=twr_seq),
                                   ranging=True) 
        _, R3 = self.listen_for_TWR_msg(self._dw.DW_LISTEN_TIMEOUT, exp_src=requester_id, exp_dest=self.tag_id, exp_msg_type=TWR_FINAL, exp_twr_seq=twr_seq)
        if R3: 
            # Report consists of header + 5 bytes each for R1, T2, R3 (40bit clock timings) + tag position and covariance
            report = self.gen_twr_msg_header(requester_id, 'REPORT', twr_seq) + list(R1.to_bytes(5, 'little') + T2.to_bytes(5, 'little') + R3.to_bytes(5, 'little'))
            # The tag position and covariance is sent as 9 signed ints [x, y, z, xx, yy, zz, xy, xz, yz]
            coordinates = self.coordinates
            covariance = coordinates.covar
            if covariance is not None: # TODO check where covariance is updated and check if it can happen that it's not done before this is called? 
                try:                   # TODO also ENSURE covar is only ints in estimator use, to prevent failure here 
                    neighbor_info = REPORT_NEIGHBOR_INFO.pack(
                        *coordinates.data,
                        covariance[0][0], covariance[1][1], covariance[2][2],
                        covariance[0][1], covariance[0][2], covariance[1][2],
                    )
                except (IndexError, OverflowError, struct.error, TypeError) as error:
                    print(f"BitcrazeTag.respond_to_ranging_exchange: invalid position metadata: {error}", 'error', 'loc')
                else:
                    report += list(neighbor_info)
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

    @staticmethod
    def extract_report_neighbor_info(report_msg:list) -> Coordinates:
        """
        Takes in a TWR report from a neighboring tag and extracts the embedded neighbor position and covariance info from it. 
        """
        payload_start = REPORT_HEADER_SIZE + REPORT_TIMESTAMP_SIZE
        payload_end = payload_start + REPORT_NEIGHBOR_INFO.size
        x, y, z, xx, yy, zz, xy, xz, yz = REPORT_NEIGHBOR_INFO.unpack(bytes(report_msg[payload_start:payload_end]))
        neighbor_coords = Coordinates(x, y, z)
        neighbor_coords.update_covar((xx, yy, zz, xy, xz, yz))
        return neighbor_coords

    def compute_range(self, target_id:int)->tuple[int|None, Coordinates|None]: 
        """
        Computes the distance in mm between the tag and another device. If the other device is a tag, also returns the tag's Coordinates object (which also holds covar)
        
        TODO: a more thorough testing of timeouts to actually figure out what is a good reliable value 
        RETURNS: 
        - Measured distance in mm if successful, None if not 
        - Coordinates position of the target, if it's a tag    
        """
        if self.is_anchor(target_id): 
            ANTENNA_TICK_DELAY = ANTENNA_TICK_DELAY_ANCHORS
        else: 
            ANTENNA_TICK_DELAY = ANTENNA_TICK_DELAY_TAGS

        def compute_clock_delta(t2, t1): 
            ### Calculates difference in clock ticks considering DW1000 clock is 40bit
            TICK_DELTA_MASK = (1 << 40) - 1
            return (t2-t1) & TICK_DELTA_MASK

        distance, target_coords = None, None
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
        if report and R2: # Ensures transaction happened. Also check R2 because ANSWER may have been sent but never recorded, and we send T3 anyways. Transaction might still complete, leaving us with incomplete info.
            # Calculating TOF 
            timing_info = report[REPORT_HEADER_SIZE:REPORT_HEADER_SIZE + REPORT_TIMESTAMP_SIZE]
            R1, T2, R3 = struct.unpack('<5s5s5s', bytes(timing_info))
            T_r1  = compute_clock_delta(R2, T1)
            T_r2  = compute_clock_delta(int.from_bytes(R3, byteorder='little'), int.from_bytes(T2, byteorder='little'))
            T_rp1 = compute_clock_delta(int.from_bytes(T2, byteorder='little'), int.from_bytes(R1, byteorder='little'))
            T_rp2 = compute_clock_delta(T3, R2)
            tof_ticks = ((T_r1 * T_r2) - (T_rp1 * T_rp2)) / (T_r1+T_r2+T_rp1+T_rp2)
            distance = int((tof_ticks + ANTENNA_TICK_DELAY) * self._dw.TIME_UNIT * SPEED_OF_LIGHT * 1000) # in mm 
            # We got the distance, if it's a tag, also extract it's position and covariance from the response 
            if not self.is_anchor(target_id) and len(report) == REPORT_HEADER_SIZE + REPORT_TIMESTAMP_SIZE + REPORT_NEIGHBOR_INFO.size:
                target_coords = self.extract_report_neighbor_info(report) 
        return distance, target_coords

    def ranging(self, target_id:int) -> tuple[int|None, Coordinates|None]: 
        # TODO add to docstring, returns range, target position Coordinates if it's a tag only
        # NOTE leaving as separate function in case want to avg measurements here 
        distance, target_pos = self.compute_range(target_id) 
        return distance, target_pos
