from ..custom_terminal import print 

from .tag import Tag
from .containers import Pose
from dw1000_spi import DW1000
from .anchors import Anchors

import time 

ANCHORS = Anchors() 

class SyntheticTag(Tag):
    """
    A synthetic tag for testing stationary IMU behavior outside of the lab
    """
    def __init__(self, tag_id:int):

        self._id = tag_id        # NOTE: It's expected that all ID's used are max 2bytes (<0xFFFF). Should be enforced by config.py
        self._active_tags = dict()      # keeps track of nearby tags and when they were last seen 
        self._available_anchors = set() # keeps track of currently in-range anchors 
        self._pose = Pose()             # Tag position, orientation, and covariance

        print(f"SYNTHETIC TAG STARTED!", 'ok', 'device') 
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
        return {100, 101, 102}
    
    @property
    def available_anchors(self)->set[int]:
        return self._available_anchors 

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
        pass
    
    def get_device_list(self, discovery_type:Literal["all", "anchor", "tag"]) -> set[int]: 
        device_list = set() 
        # Iterating through known anchors and checking who responds 
        twr_seq = self.TWR_seq 
        if discovery_type == "all" or discovery_type == "anchor": 
            for anchor in ANCHORS.anch

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
            pose = self.pose
            covariance = pose.covar
            if covariance is None: # If covar is not ready, we'll send 0s everywhere (impossible), if the other tag sees this they'll ignore our message
                covariance = np.array([[0,0,0],[0,0,0],[0,0,0]])
            try:                 
                neighbor_info = REPORT_NEIGHBOR_INFO.pack(
                    *pose.coordinates,
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
    def pose(self) -> Pose:
        return self._pose
    
    @pose.setter
    def pose(self, new_pose: Pose):
        self._pose = new_pose

    @property
    def orientation(self) -> Pose:
        return self._pose.angles

    @staticmethod
    def extract_report_neighbor_info(report_msg:list) -> Pose|None:
        """
        Takes in a TWR report from a neighboring tag and extracts the embedded neighbor position and covariance info from it. 
        If the neighbor didn't share covar (ex: too early), we ignore him and return None. 
        """
        payload_start = REPORT_HEADER_SIZE + REPORT_TIMESTAMP_SIZE
        payload_end = payload_start + REPORT_NEIGHBOR_INFO.size
        x, y, z, xx, yy, zz, xy, xz, yz = REPORT_NEIGHBOR_INFO.unpack(bytes(report_msg[payload_start:payload_end]))
        if xx==0: # This is our sign that the neighbor didn't have covar info as it will never be exactly 0 
            return None 
        return Pose(x, y, z, covar=(xx, yy, zz, xy, xz, yz))

    def compute_range(self, target_id:int) -> tuple[int | None, Pose | None]:
        """
        Computes the distance in cm between the tag and another device. If the other device is a tag, also returns its Pose (which also holds covar).
        
        TODO: a more thorough testing of timeouts to actually figure out what is a good reliable value 
        RETURNS: 
        - Measured distance in cm if successful, None if not
        - Pose of the target, if it's a tag
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
            distance = int((tof_ticks + ANTENNA_TICK_DELAY) * self._dw.TIME_UNIT * SPEED_OF_LIGHT * 100) # in cm
            if distance<=0: 
                distance = None 
            # We got the distance, if it's a tag, also extract it's position and covariance from the response 
            if not self.is_anchor(target_id) and len(report) == REPORT_HEADER_SIZE + REPORT_TIMESTAMP_SIZE + REPORT_NEIGHBOR_INFO.size:
                target_coords = self.extract_report_neighbor_info(report) # if neighbor doesn't give position AND covar, this is None 
        return distance, target_coords

    def ranging(self, target_id:int) -> tuple[int | None, Pose | None]:
        # TODO add to docstring, returns range and target Pose if it is a tag.
        # NOTE leaving as separate function in case want to avg/median measurements here 
        distance, target_pos = self.compute_range(target_id) 
        return distance, target_pos
