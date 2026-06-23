from ..custom_terminal import print 

import spidev 
import time 
from typing import Literal

TX_TIME_UNIT = 1.5650040064102565e-11 

# Channel-specific UWB configuration values
# Maps channel -> {register_name: [values]}
_CHANNEL_RF_CONFIG = {
    1: {
        '0x28_B': [0xD8],
        '0x28_C': [0x40, 0x5C, 0x00, 0x00],
        '0x2A'  : [0xC9],
        '0x2B_7': [0x07, 0x04, 0x00, 0x09],
        '0x2B_B': [0x1E],
    },
    2: {
        '0x28_B': [0xD8],
        '0x28_C': [0xA0, 0x5C, 0x04, 0x00],
        '0x2A'  : [0xC2],
        '0x2B_7': [0x08, 0x05, 0x40, 0x08],
        '0x2B_B': [0x26],
    },
    3: {
        '0x28_B': [0xD8],
        '0x28_C': [0xC0, 0x6C, 0x08, 0x00],
        '0x2A'  : [0xC5],
        '0x2B_7': [0x09, 0x10, 0x40, 0x08],
        '0x2B_B': [0x5E],
    },
    4: {
        '0x28_B': [0xBC],
        '0x28_C': [0x80, 0x5C, 0x04, 0x00],
        '0x2A'  : [0x95],
        '0x2B_7': [0x08, 0x05, 0x40, 0x08],
        '0x2B_B': [0x26],
    },
    5: {
        '0x28_B': [0xD8],
        '0x28_C': [0xE0, 0x3F, 0x1E, 0x00],
        '0x2A'  : [0xC0],
        '0x2B_7': [0x1D, 0x04, 0x00, 0x08],
        '0x2B_B': [0xA6],
    },
    7: {
        '0x28_B': [0xBC],
        '0x28_C': [0xE0, 0x7D, 0x1E, 0x00],
        '0x2A'  : [0x93],
        '0x2B_7': [0x1D, 0x04, 0x00, 0x08],
        '0x2B_B': [0xA6],
    },
}

# TX power configuration: channel -> {PRF: [values]}
# Used for register 0x1E
_TX_POWER_CONFIG = {
    1: {16: [0x75, 0x55, 0x35, 0x15], 64: [0x67, 0x47, 0x27, 0x07]},
    2: {16: [0x75, 0x55, 0x35, 0x15], 64: [0x67, 0x47, 0x27, 0x07]},
    3: {16: [0x6F, 0x4F, 0x2F, 0x0F], 64: [0x8B, 0x6B, 0x4B, 0x2B]},
    4: {16: [0x5F, 0x3F, 0x1F, 0x1F], 64: [0x9A, 0x7A, 0x5A, 0x3A]},
    5: {16: [0x48, 0x28, 0x08, 0x0E], 64: [0x85, 0x65, 0x45, 0x25]},
    7: {16: [0x92, 0x72, 0x52, 0x32], 64: [0xD1, 0xB1, 0x71, 0x51]},
}

class DW1000: 
    """
    Class defining interactions with the Decawave DW1000 UWB chip through SPI.
    Communicates in SPI mode 0. 
    
    IMPORTANT: Use a context manager to ensure that the connection is properly closed. 
    """
    def __init__(self, bus:int, cs:int, channel:Literal[1,2,3,4,5,7], PRF:Literal[16,64], bitrate:Literal[110,850,6,7], preamble_length:Literal[64,128,256,512,1024,1536,2048,4096], preamble_code:int): 
        self.spi = spidev.SpiDev() 
        self.spi.open(bus, cs) 
        
        self.spi.max_speed_hz = 3_000_000 # On init, should not exceed 3MHz  
        self.spi.mode = 0b00              # GPIO 5 and 6 dictate the mode, should be untouched 
        
        self.prep_device_for_use()        # Checks that device initialized properly and sets clock to 20MHz
        self.config_uwb_settings(channel, PRF, bitrate, preamble_length, preamble_code) # Checks validity of settings and applies them 
        print("DW1000 DEVICE READY", 'ok', 'gen')
    
    def prep_device_for_use(self): 
        """
        After a reset:
            - Checks that the device properly turned on & communicates
            - Verifies that the clock PLL locked 
            - Loads LDE microcode
            - Sets comm speed to 20MHz 
        """
        ## Checking device ID is expected for DW1000 
        d_id = self.read_register([0x00], 4, reverse=True)
        if d_id != ['0xde', '0xca', '0x1', '0x30']:
            print("DW1000 did not return the correct ID. Is it plugged in correctly?", 'error', 'gen')
            print(f"Expected ID: ['0xde', '0xca', '0x1', '0x30'], got {d_id}", 'error', 'gen')
            self.close() 
            quit() 
        
        ## Checking CLK PLL locked and device completed INIT mode 
        cfg = self.read_register([0x0F], 5) 
        cplock = self.hex_to_octet(cfg[0])[6] # 2nd bit of 0th octet
        slp2init = self.hex_to_octet(cfg[2])[0] # last bit of 2nd octet
        if not (cplock and slp2init): 
            print("[ERROR] DW1000 did not lock CLK PLL and/or did not reach INIT successfully. Try replugging it?")
            self.close() 
            quit() 

        ## Loading LDE microcode as per steps in p.22 of user manual 
        self.write_register([0xF6, 0x00], [0x01, 0x03])
        self.write_register([0xED, 0x06], [0x00, 0x80])
        time.sleep(150e-6)
        self.write_register([0xF6, 0x00], [0x00, 0x02])
        
        ## Make sure LDE loads after sleep 
        aon = self.read_register([0x6C, 0x00], 2, return_ints=True)
        value = aon[0] | (aon[1] << 8)
        value |= (1 << 11)   # ONW_LLDE
        new_aon = [value & 0xFF, (value >> 8) & 0xFF]
        if aon!=new_aon: 
            self.write_register([0xEC, 0x00], new_aon)

        ## If we get here, DW1000 should be in IDLE state, set rate to maximum 
        self.spi.max_speed_hz = 20_000_000 # In IDLE state, can operate at 20MHz 
    
    def close(self, verbose=True): 
        self.spi.close() 
        if verbose: 
            print("DW1000 connection closed", 'info', 'gen')

    def __enter__(self):
        return self 

    def __exit__(self, exc_type, exc_val, exc_tb): 
        self.close() 
        print("DW1000 exited context manager successfully", 'ok', 'gen')

    def __del__(self):
        try:
            self.close(verbose=False) 
            print("__del__ called on dw1000 obj", 'info', 'gen')
        except: 
            pass 

    @staticmethod
    def hex_to_octet(hex_str:str)->str: 
        """ 
        Converts a hex string into the equivalent octet binary string 
        """
        # TODO: add <0xFF checks 
        return f"{int(hex_str, base=16):08b}"
    
    def read_register(self, header:list, length:int, reverse:bool=False, return_ints:bool=False)->list[str|int]: 
        """
        Reads the value of a register. WARNING: Does not check validity of transaction.

        ARGS: 
            - header: 1 to 3 octet header of the transaction in list format
            - length: Size of register in octets
            - reverse: Reverse output. By default, read is done LSB first as specified in DW1000 user manual.  
            - return_ints: Return int values instead of hex strings
        RETURNS: 
            - List of values read from the register. 
        """
        if type(header) != list or type(length) != int or type(reverse) != bool: 
            print("Unexpected type in read_register. Check your args.", 'error', 'device')
            return None 
        response = self.spi.xfer2(header + [0]*length) # xfer2 is supposed to keep CS pressed for the entire transaction, although some sources differ, I use it to be safe. At worst its equivalent to xfer 
        response = response[len(header):]              # throwing away the header
        if reverse: response.reverse() 
        return response if return_ints else [hex(octet) for octet in response]
    
    def write_register(self, header:list, data:list)->None: 
        """
        Writes values to a register. WARNING: Does not check validity of transaction.

        ARGS: 
            - header: 1 to 3 octet header of the transaction in list format
            - data: List of octets to write to the register (LSB first)
        """
        if type(header) != list or type(data) != list or type(data[0]) != int:
            print("Unexpected type in write_register. Check your args.", 'error', 'device')
            return None 
        self.spi.xfer2(header + data) 

    def soft_reset(self, rx_only:bool=False): 
        """
        Performs a soft reset of the IC with the SOFTRESET register, as specified by the user manual. 
        Will reset the IC TX, RX, Host Interface and the PMSC. 
        
        ARGS:
            - rx_only: Whether or not to only reset the rx components 
        """   
        # Getting the initial configuration to ensure we don't overwrite bits 
        pmsc = self.read_register([0x76, 0x00], 4, return_ints=True) # should be smthing like: [0, 2, 48, 240]
        # Since we are about to reset the device, lower the speed to init values 
        self.spi.max_speed_hz = 3_000_000
    
        # Setting SYSCLKS to 01 
        pmsc[0] = (pmsc[0] & 0b11111100) | 0x01 # turning first 2 bits to 0 and then 01 
        self.write_register([0xF6, 0x00], pmsc) 
        # Clearing SOFTRESET to all zeros (except if RX only reset: affects only bit 28) 
        if rx_only: 
            pmsc[3] = (pmsc[3] & 0xEF) | 0x00
        else: 
            pmsc[3] = (pmsc[3] & 0x0F) | 0x00
        self.write_register([0xF6, 0x00], pmsc)
        # Waiting a bit 
        time.sleep(0.01) 
        # Setting SOFTRESET to all ones (except if RX only reset: affects only bit 28)
        if rx_only: 
            pmsc[3] = (pmsc[3] & 0xEF) | 0x10
        else:
            pmsc[3] = (pmsc[3] & 0x0F) | 0xF0
        self.write_register([0xF6, 0x00], pmsc) 
        # Setting SYSCLKS back to 00 (auto mode for reset)
        pmsc[0] = (pmsc[0] & 0b11111100) | 0x00
        self.write_register([0xF6, 0x00], pmsc) 
        
        # Reset done, run our init checks and set speed back to 20MHz
        self.prep_device_for_use() 
        print("Soft reset of DW1000 completed.", 'info', 'device')

    def transmit(self, data:list, ranging:bool, timeout:float=0.05)->tuple[bool, bytes|None]: 
        """
        Transmits a message with the DW1000. Returns bool on whether was successfully sent. 

        ARGS: 
            - data: List of bytes to send (LSB first) 
            - ranging: Bool indicating if ranging type of message
            - timeout: Max amount of time [s] to wait for confirmed send of message 
        RETURNS: 
            - bool indicating if operation successful 
            - If ranging True, also returns TX timestamp (reg 0x17) IN CLK TICKS (raw 40bit value) 
        """
        tx_timestamp = None
        ### Writing data to TX buffer 
        self.write_register([0x89], data)
        ### Setting frame length in Transmit Frame Control 
        data_length = len(data) + 2 # adding 2 octets of CRC check 
        # NOTE: currently not supporting extended data frames, messages shouldn't exceed 127 bytes 
        if data_length>127: 
            print("[WARNING] in DW1000.transmit(), data frame exceed 127bytes (TFLE!=0). Currently not supporting this, check the message?", 'info', 'device')
            return 
        curr = self.read_register([0x08], 2, return_ints=True)
        tx_fctrl = data_length | ((curr[1] & 0x60)<<8)  # TFLEN == data_length, keeping TXBR and 0 TFLE and R 
        tx_fctrl |= int(ranging)<<15
        self.write_register([0x88], [tx_fctrl & 0xFF, (tx_fctrl>>8) & 0xFF])
        ### Starting transmission in System Control Register, this automatically clears TXFRS 
        self.write_register([0x8D], [0x02])
        ### Checking if TX completed with TXFRS in SYS_STATUS
        t1 = time.perf_counter() 
        while (time.perf_counter()-t1)<timeout: 
            txfrs = self.read_register([0x0F], 1, return_ints=True)[0] >> 7  
            if txfrs: 
                if ranging: ## If TX OK & ranging, return TX_TIME too 
                    tx_time = self.read_register([0x17], 5, return_ints=True)
                    tx_timestamp = tx_time[0] | tx_time[1]<<8 | tx_time[2]<<16 | tx_time[3]<<24 | tx_time[4]<<32
                    return bool(txfrs), tx_timestamp
                break # DW1000 will return to IDLE automatically 
        else: 
            # TODO Can add more in depth inspection of error bits in future 
            print("Transmit timeout in DW1000.transmit(), returning to IDLE", 'error', 'device')
            self.write_register([0x8D], [0x40]) # Force return to IDLE 
        return bool(txfrs) 

    def listen(self, timeout:float, return_ints=True): 
        """
        Sets the DW1000 to RX mode and listens for messages. 
        Returns the first message found, if any, and closes the connection. 
        
        ARGS: 
            - timeout: Max amount of time [s] to stay listening 
            - return_ints: Whether to return in hex string format or pure ints
        """
        data = None 
        self.write_register([0x8D], [0, 1]) # enable RX at reg. SYS_CTRL 0x0D
        start = time.perf_counter() 
        while (time.perf_counter()-start)<timeout: 
            # Listen in reg. SYS_STATUS 0x0F 
            status = self.read_register([0x0F], 5, return_ints=True) 
            bits8_23 = status[1] | (status[2]<<8) 
            # Good frames have: RXFCG, RXDFR, RXPHD, LDEDONE, RXSFDD and RXPRD set  
            if (bits8_23 & 0x006F)==0x006F: # Good frame 
                # Checking RXFINFO for frame length 
                rx_finfo = self.read_register([0x10], 4, return_ints=True)
                rxfle_rxflen = (rx_finfo[0] | (rx_finfo[1]<<8)) & 0x3FF
                # NOTE: currently not supporting non std operation, so RXFLE should be 0 
                if rxfle_rxflen>0x7F: # Currently, messages should be <= 127bytes 
                    print("[WARNING] in DW1000.listen(), received extended data frame (RXFLE!=0). Currently not supporting this, check the message?", 'info', 'device')
                # Now reading buffer with frame length 
                rxfle_rxflen-=2 # throwing away FCS at the end 
                data = self.read_register([0x11], rxfle_rxflen, return_ints=return_ints)  
                break 
            else: 
                # Either received a bad frame or nothing 
                # NOTE, TODO: In the future, would be interesting to add more debugging info 
                # (Checking individually error bits) 
                # When doing that, also clearing latched bits during iter to make sure checks are properly done 
                pass 
        # Disabling RX, this will automatically clear latched bits related to it 
        self.write_register([0x8D], [0x40])
        return data 
    
    @staticmethod
    def check_valid_uwb_config(channel:int, PRF:int, bitrate:float, preamble_length:int, preamble_code:int)->bool: 
        """ 
        Checks if a specific UWB configuration is valid based on the DW1000 user manual. 
        """
        if type(channel)!=int or type(PRF)!=int or type(preamble_code)!=int: 
            return False 
        if channel not in [1,2,3,4,5,7]: 
            return False 
        if not (PRF==64 or PRF==16):
            return False 
        ## Checking preambles (p.202)
        if PRF==64 and channel!=4 and channel!=7: 
            allowed_preambles = [9,10,11,12] 
        elif PRF==64 and (channel==4 or channel==7):
            allowed_preambles = [17,18,19,20] 
        elif PRF==16 and channel==1: 
            allowed_preambles = [1, 2] 
        elif PRF==16 and (channel==2 or channel==5): 
            allowed_preambles = [3, 4] 
        elif PRF==16 and channel==3: 
            allowed_preambles = [5, 6] 
        elif PRF==16 and (channel==4 or channel==7): 
            allowed_preambles = [7, 8] 
        if not preamble_code in allowed_preambles:
            return False 
        ## Checking bitrate
        if bitrate not in [110, 850, 6.8, 6, 7]: 
            return False 
        ## Preamble length 
        if preamble_length not in [64,128,256,512,1024,1536,2048,4096]: 
            return False 
        ## Preamble and bitrate combination (for DRX_TUNE1b, p.133 of user manual)
        if (preamble_length>1024) and (bitrate==110): 
            pass # ok 
        elif (128<=preamble_length<=1024) and (bitrate!=110):
            pass # ok
        elif preamble_length==64 and bitrate!=110 and bitrate!=850: 
            pass # ok 
        else: 
            print("Invalid bitrate-preamble config for DRX_TUNE1b", 'error', 'device')
            return False 
        ## If all passed, everything ok 
        return True 

    def config_uwb_settings(self, channel:Literal[1,2,3,4,5,7], PRF:Literal[16,64], bitrate:Literal[110,850,6,7], preamble_length:Literal[64,128,256,512,1024,1536,2048,4096], preamble_code:int)->None: 
        """
        Configures the DW1000's UWB settings. Validates the settings before applying them. 
        
        ARGS: 
            - channel: Communication channel 
            - PRF: Pulse repetition frequency (16 or 64MHz) 
            - preamble_code: Code associated to PRF, see user manual. 
            - bitrate: Bitrate, 110kps, 850kps or 6.8Mbps (latter can be passed as 6, 7 or 6.8, all correspond to 6.8)
            - preamble_length: in symbols

        Modifies the following registers:
            - 0x1F    - Channel Control
            - 0x08    - Transmit Frame Control 
            - 0x28:0B - RF_RXCTRLH
            - 0x28:0C - RF_TXCTRL 
            - 0x2A:0B - TC_PGDELAY
            - 0x2B:07 - FS_PLLCFG
            - 0x2B:0B - FS_PLLTUNE
            - 0x27:02 - DRX_TUNE0b
            - 0x27:04 - DRX_TUNE1a
            - 0x27:06 - DRX_TUNE1b
            - 0x27:08 - DRX_TUNE2
            - 0x23:04 - AGC_TUNE1
            - 0x23:0C - AGC_TUNE2
            - 0x23:12 - AGC_TUNE3
            - 0x1E    - TX Power
        """
        if not self.check_valid_uwb_config(channel, PRF, bitrate, preamble_length, preamble_code):
            # This ensures that the combination of values is valid 
            # and simplifies following if/elses 
            print("Unsupported UWB config for DW1000, check DW1000.config_uwb_settings()", 'error', 'device')
            return 
        
        ###################### 0x1F - Channel Control ######################
        og_cfg = self.read_register([0x1F], 4, return_ints=True) 
        new_cfg = 0x00_00_00_00
        ## First byte is the channel for RX and TX 
        byte1 = int(f"0x{channel}{channel}", base=16) 
        new_cfg |= byte1
        ## Second byte is entirely reserved, we don't change it  
        new_cfg |= og_cfg[1] << 8 # shifting left by 8 to match register layout 
        ## Now we look at bits 19-16 
        # RXPRF (bit 19-18) will set the PRF 
        # DWSFD (bit 17) left at 0 to use std DW SFD. Also leaving RNSSFD and TNSSFD at 0 for this. 
        # Reserved bit 16 is left unchanged. 
            # To get the value of bit16, we'll have to modify whole octet (23-16)
            # but it's ok, we'll just write over it next step, since TX_PCODE is part of it
        if PRF==16: # 16MHz: RXPRF at 0b01
            prf_cfg = (og_cfg[2] & 0x01) | 0x04 
        elif PRF==64: # 64MHz: RXPRF at 0b10
            prf_cfg = (og_cfg[2] & 0x01) | 0x08
        new_cfg |= prf_cfg << 16  
        ## Now writing TX_PCODE and RX_PCODE (bits 31-22) 
        new_cfg |= preamble_code << 22 
        new_cfg |= preamble_code << 27 
        ## Finally splitting this into bytes and sending 
        chan_cfg = [new_cfg & 0xFF, (new_cfg>>8) & 0xFF, (new_cfg>>16) & 0xFF, (new_cfg>>24) & 0xFF]
        self.write_register([0x9F], chan_cfg) 

        ###################### 0x08 - Transmit Frame Control ######################
        og_cfg = self.read_register([0x08], 5, return_ints = True) 
        new_cfg = 0x00_00_00_00_00 
        # TFLEN, TFLE and R stay unchanged (bits 0-12) 
        new_cfg |= ((og_cfg[1]<<8) | og_cfg[0])  & 0x1FFF # mask only keeps bits 0-12 
        # TXBR sets bitrate (bits 13-14)
        if bitrate == 110:
            new_cfg |= 0b00<<13 
        elif bitrate == 850:
            new_cfg |= 0b01<<13 
        else: # check_valid_uwb_config ensures other values are 6.8, 6 or 7
            new_cfg |= 0b10<<13 
        # TR (bit 15) unchanged 
        new_cfg |= (og_cfg[1]<<8) & 0x8000
        # TXPRF - Sets PRF (bits 16-17)
        if PRF==16: 
            new_cfg |= 0b01<<16
        elif PRF==64: 
            new_cfg |= 0b10<<16 
        # TXPSR and PE - Sets preamble length (bits 18-21)
        if preamble_length==64: 
            bits = 0b0001
        elif preamble_length==128: 
            bits = 0b0101
        elif preamble_length==256: 
            bits = 0b1001
        elif preamble_length==512: 
            bits = 0b1101
        elif preamble_length==1024: 
            bits = 0b0010
        elif preamble_length==1536: 
            bits = 0b0110
        elif preamble_length==2048: 
            bits = 0b1010
        elif preamble_length==4096: 
            bits = 0b0011
        new_cfg |= bits<<18
        # Not touching the rest (bits 22-40)
        new_cfg |= ((og_cfg[2] & 0xC0)<<16) # bits 22 and 23 
        new_cfg |= og_cfg[3]<<24 
        new_cfg |= og_cfg[4]<<32
        ## Splitting into bytes and sending 
        new_cfg = [new_cfg&0xFF, (new_cfg>>8)&0xFF, (new_cfg>>16)&0xFF, (new_cfg>>24)&0xFF, (new_cfg>>32)&0xFF]
        self.write_register([0x88], new_cfg)        

        ###################### 0x28:0B, 0x28:0C, 0x2A:0B, 0x2B:07 and 0x2B:0B ######################
        channel_config = _CHANNEL_RF_CONFIG[channel]
        
        og_0x28_B = self.read_register([0x68, 0x0B], 1, return_ints=True)
        if og_0x28_B != channel_config['0x28_B']:
            self.write_register([0xE8, 0x0B], channel_config['0x28_B'])
        
        og_0x28_C = self.read_register([0x68, 0x0C], 4, return_ints=True)
        if og_0x28_C != channel_config['0x28_C']:
            self.write_register([0xE8, 0x0C], channel_config['0x28_C'])
            # NOTE: for some reason, the last (0x00) byte I send doesn't seem to affect it
            # it always reads as 0xDE after the operation. But all other bytes are good. 
            # i assume those bytes are just not writeable or are overwritten immediately. 
        
        og_0x2A = self.read_register([0x6A, 0x0B], 1, return_ints=True)
        if og_0x2A != channel_config['0x2A']:
            self.write_register([0xEA, 0x0B], channel_config['0x2A'])
        
        og_0x2B_7 = self.read_register([0x6B, 0x07], 4, return_ints=True)
        if og_0x2B_7 != channel_config['0x2B_7']:
            self.write_register([0xEB, 0x07], channel_config['0x2B_7'])
        
        og_0x2B_B = self.read_register([0x6B, 0x0B], 1, return_ints=True)
        if og_0x2B_B != channel_config['0x2B_B']:
            self.write_register([0xEB, 0x0B], channel_config['0x2B_B'])
        
        ###################### 0x27:02 ######################
        # NOTE: Currently only supports standard SFD! 
        tune0b = self.read_register([0x67, 0x02], 2, return_ints=True)
        if bitrate == 110:
            new_cfg = [0x0A, 0x00]
        elif bitrate == 850:
            new_cfg = [0x01, 0x00]
        else: # check_valid_uwb_config ensures other values are 6.8, 6 or 7
            new_cfg = [0x01, 0x00] 
        if tune0b!=new_cfg: 
            self.write_register([0xE7, 0x02], new_cfg) 
        ###################### 0x27:04 ######################
        tune1a = self.read_register([0x67, 0x04], 2, return_ints=True)
        if PRF==16: 
            new_cfg = [0x87, 0x00]
        elif PRF==64:
            new_cfg = [0x8D, 0x00]
        if tune1a!=new_cfg:
            self.write_register([0xE7, 0x04], new_cfg)
        ###################### 0x27:06 ######################
        tune1b = self.read_register([0x67, 0x06], 2, return_ints=True)
        if (preamble_length>1024) and (bitrate==110): 
            new_cfg = [0x64, 0x00]
        elif (128<=preamble_length<=1024) and (bitrate!=110):
            new_cfg = [0x20, 0x00]
        elif preamble_length==64 and bitrate!=110 and bitrate!=850: 
            new_cfg = [0x10, 0x00]
        if tune1b!=new_cfg: 
            self.write_register([0xE7, 0x06], new_cfg)
        ###################### 0x27:08 ######################
        tune2 = self.read_register([0x67, 0x08], 4, return_ints=True)
        # NOTE: this sets PAC size, using recommended # of Table 5 user manual (p.29)
        if preamble_length<=128: 
            PAC_size = 8 
            if PRF==16:
                new_cfg = [0x2D,0x00,0x1A,0x31] 
            elif PRF==64: 
                new_cfg = [0x6B,0x00,0x3B,0x31] 
        elif preamble_length<=512: 
            PAC_size = 16 
            if PRF==16:
                new_cfg = [0x52,0x00,0x1A,0x33]
            elif PRF==64: 
                new_cfg = [0xBE,0x00,0x3B,0x33]
        elif preamble_length==1024: 
            PAC_size = 32 
            if PRF==16:
                new_cfg = [0x9A,0x00,0x1A,0x35]
            elif PRF==64: 
                new_cfg = [0x5E,0x01,0x3B,0x35]
        elif preamble_length>=1536: 
            PAC_size = 64 
            if PRF==16:
                new_cfg = [0x1D,0x01,0x1A,0x37]
            elif PRF==64: 
                new_cfg = [0x96,0x02,0x3B,0x37]
        if tune2!=new_cfg: 
            self.write_register([0xE7, 0x08], new_cfg)
        ###################### 0x23:04 ######################
        agc = self.read_register([0x63, 0x04], 2, return_ints=True)
        if PRF==16:
            new_cfg = [0x70, 0x88]
        elif PRF==64: 
            new_cfg = [0x9B, 0x88]
        if agc!=new_cfg: 
            self.write_register([0xE3, 0x04], new_cfg) 
        ###################### 0x23:0C ######################
        agc = self.read_register([0x63, 0x0C], 4, return_ints=True)
        new_cfg = [0x07,0xA9,0x02,0x25] 
        if agc!=new_cfg: 
            self.write_register([0xE3, 0x0C], new_cfg) 
        ###################### 0x23:12 ######################
        agc = self.read_register([0x63, 0x12], 2, return_ints=True)
        new_cfg = [0x55, 0x00]
        if agc!=new_cfg: 
            self.write_register([0xE3, 0x12], new_cfg) 
        ###################### 0x2E:1806 ####################
        # This is a 3 octet read, 0x86 comes from 0x80|(0x1806&0x7F) 
        # and 0x30 comes from 0x1806>>7 as per header construction rules of user manual. 
        lde = self.read_register([0x6E, 0x86, 0x30], 2, return_ints=True)
        if PRF==16:
            new_cfg = [0x07, 0x16]
        elif PRF==64: 
            new_cfg = [0x07, 0x06] 
        if lde!=new_cfg:
            self.write_register([0xEE, 0x86, 0x30], new_cfg)
        ###################### 0x1E ######################
        # NOTE 0x1E: 2026-06-10, only supporting smart TX power rn 
        # in future, check if useful to add manual power 
        tx_power = self.read_register([0x1E], 4, return_ints=True)
        new_cfg = _TX_POWER_CONFIG[channel][PRF]
        if tx_power != new_cfg: 
            self.write_register([0x1E], new_cfg)
        ###################### DONE! ######################
        to_print = f"""
        --- DW 1000 UWB settings configured. ---
        Channel: {channel}
        Bitrate: {bitrate}
        PRF: {PRF}
        Preamble length: {preamble_length}
        Preamble code: {preamble_code}
        PAC size: {PAC_size}
        **NOTE**: Settings currently only support smart TX power and standard SFD.""" 
        print(to_print, 'ok', 'device')