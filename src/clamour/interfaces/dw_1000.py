import spidev 
import time 

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
        self.spi.mode = 0b00            # GPIO 5 and 6 dictate the mode, should be untouched 
        
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
        print("[OK] DW1000 DEVICE READY")
    
    def close(self): 
        self.spi.close() 
        print("[INFO] DW1000 connection closed")

    def __enter__(self):
        return self 

    def __exit__(self, exc_type, exc_val, exc_tb): 
        self.close() 
        print("DW1000 exited context manager successfully")

    def __del__(self):
        try:
            self.close() 
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
    
    def read_register(self, header:list, length:int, reverse:bool=False)->list[str]: 
        """
        Reads the value of a register. 
        Args: 
            Header: 1 to 3 octet header of the transaction in list format
            Length: Size of register in octets
            Reverse: Reverse output. By default, read is done LSB first as specified in DW1000 user manual.  
        Returns: 
            List of hex values read from the register. 
        """
        if type(header) != list or type(length) != int or type(reverse) != bool: 
            print("[ERROR] Unexpected type. Check your args.")
            return None 
        response = self.spi.xfer2(header + [0]*length) # xfer2 is supposed to keep CS pressed for the entire transaction, although some sources differ, I use it to be safe. At worst its equivalent to xfer 
        response = [hex(octet) for octet in response][len(header):] # throwing away the header
        if reverse: response.reverse() 
        return response
