import spidev 
import time 
from typing import Literal

class DW1000: 
    """
    Class defining interactions with the Decawave DW1000 UWB chip through SPI.
    Communicates in SPI mode 0. 
    
    IMPORTANT: Use a context manager to ensure that the connection is properly closed. 
    """
    def __init__(self, bus:int, cs:int): 
        self.spi = spidev.SpiDev() 
        self.spi.open(bus, cs) 
        
        self.spi.max_speed_hz = 3_000_000 # On init, should not exceed 3MHz  
        self.spi.mode = 0b00              # GPIO 5 and 6 dictate the mode, should be untouched 
        
        self.check_device_ready()         # Checks that device initialized properly and sets clock to 20MHz
        print("[OK] DW1000 DEVICE READY")
    
    def check_device_ready(self): 
        """
        After a reset, checks that the device properly turned on. 
        Also verifies that the clock PLL locked & sets the comm speed accordingly. 
        """
        ## Checking device ID is expected for DW1000 
        d_id = self.read_register([0x00], 4, reverse=True)
        if d_id != ['0xde', '0xca', '0x1', '0x30']:
            print("[ERROR] DW1000 did not return the correct ID. Is it plugged in correctly?")
            print(f"Expected ID: ['0xde', '0xca', '0x1', '0x30'], got {d_id}")
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

        ## If we get here, DW1000 should be in IDLE state, set rate to maximum 
        self.spi.max_speed_hz = 20_000_000 # In IDLE state, can operate at 20MHz 
    
    def close(self, verbose=True): 
        self.spi.close() 
        if verbose: 
            print("[INFO] DW1000 connection closed")

    def __enter__(self):
        return self 

    def __exit__(self, exc_type, exc_val, exc_tb): 
        self.close() 
        print("DW1000 exited context manager successfully")

    def __del__(self):
        try:
            self.close(verbose=False) 
            print("[INFO] __del__ called on dw1000 obj")
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
        Reads the value of a register. 
        Args: 
            header: 1 to 3 octet header of the transaction in list format
            length: Size of register in octets
            reverse: Reverse output. By default, read is done LSB first as specified in DW1000 user manual.  
            return_ints: Return int values instead of hex strings
        Returns: 
            List of values read from the register. 
        """
        if type(header) != list or type(length) != int or type(reverse) != bool: 
            print("[ERROR] Unexpected type in read_register. Check your args.")
            return None 
        response = self.spi.xfer2(header + [0]*length) # xfer2 is supposed to keep CS pressed for the entire transaction, although some sources differ, I use it to be safe. At worst its equivalent to xfer 
        response = response[len(header):]              # throwing away the header
        if reverse: response.reverse() 
        return response if return_ints else [hex(octet) for octet in response]
    
    def write_register(self, header:list, data:list)->None: 
        """
        Writes values to a register. 
        Args: 
            header: 1 to 3 octet header of the transaction in list format
            data: list of octets to write to the register 
        """
        if type(header) != list or type(data) != list:
            print("[ERROR] Unexpected type in write_register. Check your args.")
            return None 
        self.spi.xfer2(header + data) 

    def soft_reset(self, rx_only:bool=False): 
        """
        Performs a soft reset of the IC with the SOFTRESET register, as specified by the user manual. 
        Will reset the IC TX, RX, Host Interface and the PMSC. 
        Args: 
            rx_only: Whether to only reset the rx components 
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
        self.check_device_ready() 
        print("[INFO] Soft reset of DW1000 completed.")

    @staticmethod
    def check_valid_uwb_config(channel:int, PRF:int, preamble_code:int, bitrate:float, preamble_length:int)->bool: 
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
            print("Invalid bitrate-preamble config for DRX_TUNE1b")
            return False 
        ## If all passed, everything ok 
        return True 

    def config_uwb_settings(self, channel:Literal[1,2,3,4,5,7], PRF:Literal[16,64], preamble_code:int, bitrate:Literal[110,850,6,7], preamble_length:Literal[64,128,256,512,1024,1536,2048,4096])->None: 
        """
        Configures the DW1000's UWB settings. Modifies the following registers:
            - 0x1F    - Channel Control
            - 0x08    - Transmit Frame Control 
            - 0x28:0B - RF_RXCTRLH
            - 0x28:0C - RF_TXCTRL 
            - 0x2A:0B - TC_PGDELAY
            - 0x2B:07 - FS_PLLCFG
            - 0x27:02 - DRX_TUNE0b
            - 0x27:04 - DRX_TUNE1a
            - 0x27:06 - DRX_TUNE1b
            - 0x27:08 - DRX_TUNE2
        
        Args: 
            - channel: Communication channel 
            - PRF: Pulse repetition frequency (16 or 64MHz) 
            - preamble_code: Code associated to PRF, see user manual. 
            - bitrate: Bitrate, 110kps, 850kps or 6.8Mbps (latter can be passed as 6, 7 or 6.8, all correspond to 6.8)
            - preamble_length: in symbols 
        """
        if not self.check_valid_uwb_config(channel, PRF, preamble_code, bitrate, preamble_length):
            # This ensures that the combination of values is valid 
            # and simplifies following if/elses 
            print("[ERROR] Unsupported UWB config for DW1000, check DW1000.config_uwb_settings()")
            return 
        
        ###################### 0x1F - Channel Control ######################
        og_cfg = self.read_register([0x1F], 4, return_ints=True) 
        new_cfg = 0x00_00_00_00
        ## First byte is the channel for RX and TX 
        byte1 = int(f"0x{channel}{channel}", base=16) 
        new_cfg |= byte1
        ## Second byte is entirely reserved, we don't change it 
        byte2 = og_cfg[1] 
        new_cfg |= byte2 << 8 # shifting left by 8 to match register layout 
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

        ###################### 0x28:0B, 0x28:0C, 0x2A:0B and ######################
        ###################### 0x2B:07, 0x2B:0B,  ######################
        og_0x28_B = self.read_register([0x68, 0x0B], 1, return_ints=True)
        og_0x28_C = self.read_register([0x68, 0x0C], 4, return_ints=True) 
        og_0x2A   = self.read_register([0x6A, 0x0B], 1, return_ints=True)
        og_0x2B_7 = self.read_register([0x6B, 0x07], 4, return_ints=True)
        og_0x2B_B = self.read_register([0x6B, 0x0B], 1, return_ints=True)

        if channel == 1: 
            new_0x28_B = [0xD8]
            new_0x28_C = [0x40, 0x5C, 0x00, 0x00]
            new_0x2A = [0xC9]
            new_0x2B_7 = [0x07, 0x04, 0x00, 0x09]
            new_0x2B_B = [0x1E]
        elif channel == 2: 
            new_0x28_B = [0xD8]
            new_0x28_C = [0xA0, 0x5C, 0x04, 0x00]
            new_0x2A = [0xC2]
            new_0x2B_7 = [0x08, 0x05, 0x40, 0x08]
            new_0x2B_B = [0x26]
        elif channel == 3: 
            new_0x28_B = [0xD8]
            new_0x28_C = [0xC0, 0x6C, 0x08, 0x00]
            new_0x2A = [0xC5]
            new_0x2B_7 = [0x09, 0x10, 0x40, 0x08]
            new_0x2B_B = [0x5E]
        elif channel == 4: 
            new_0x28_B = [0xBC]
            new_0x28_C = [0x80, 0x5C, 0x04, 0x00]
            new_0x2A = [0x95]
            new_0x2B_7 = [0x08, 0x05, 0x40, 0x08]
            new_0x2B_B = [0x26]
        elif channel == 5: 
            new_0x28_B = [0xD8]
            new_0x28_C = [0xE0, 0x3F, 0x1E, 0x00]
            new_0x2A = [0xC0]
            new_0x2B_7 = [0x1D, 0x04, 0x00, 0x08]
            new_0x2B_B = [0xA6]
        elif channel == 7:
            new_0x28_B = [0xBC]
            new_0x28_C = [0xE0, 0x7D, 0x1E, 0x00]
            new_0x2A = [0x93]
            new_0x2B_7 = [0x1D, 0x04, 0x00, 0x08]
            new_0x2B_B = [0xA6]

        if og_0x28_B != new_0x28_B:
            self.write_register([0xE8, 0x0B], new_0x28_B)
        if og_0x28_C!=new_0x28_C: 
            self.write_register([0xE8, 0x0C], new_0x28_C)
            # NOTE: for some reason, the last (0x00) byte I send doesn't seem to affect it
            # it always reads as 0xDE after the operation. But all other bytes are good. 
            # i assume those bytes are just not writeable or are overwritten immediately. 
        if og_0x2A!=new_0x2A:
            self.write_register([0xEA, 0x0B], new_0x2A)
        if og_0x2B_7!=new_0x2B_7: 
            self.write_register([0xEB, 0x07], new_0x2B_7)
        if og_0x2B_B!=new_0x2B_B:
            self.write_register([0xEB, 0x0B], new_0x2B_B)
        
        ###################### 0x27:02 ######################
        # NOTE: Assuming only using standard SFD! 
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