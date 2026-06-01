import spidev 
import time 

class DW1000: 
    """
    Class defining interactions with the Decawave DW1000 UWB chip through SPI.
    Always instanciate with a context manager to ensure that the connection is properly closed. 
    """
    def __init__(self, bus:int, cs:int): 
        self.spi = spidev.SpiDev() 
        self.spi.open(bus, cs) 
        
        self.spi.max_speed_hz = 3000000 # On init, should not exceed 3MHz  
        self.spi.mode = 0b00            # GPIO 5 and 6 dictate the mode, should be untouched 
        
        ## Checking device ID is expected for DW1000 
        d_id = self.read_register(0x00, 4, reverse=True) 
        if d_id != ['0xde', '0xca', '0x1', '0x30']:
            print("[ERROR] DW1000 did not return the correct ID. Is it plugged in correctly?")
            print(f"Expected ID: ['0xde', '0xca', '0x1', '0x30'], got {d_id}")
            self.spi.close() 
            quit() 
        else: 
            print("Successfully connected DW1000 device. ")

    def __enter__(self):
        return self 

    def __exit__(self, exc_type, exc_val, exc_tb): 
        self.spi.close() 
        print("DW1000 context manager exited. Connection closed successfully.")

    def __del__(self):
        try:
            self.spi.close() 
        except: 
            pass 
    
    def read_register(self, header:bytes, length:int, reverse:bool=False)->list[str]: 
        """
        Reads the value of a register. 
        Args: 
            Header: Header of the transaction. Points to register and any subaddressings.
            Length: Size of register in octets
            Reverse: Reverse output. By default, read is done LSB first as specified in DW1000 user manual.  
        Returns: 
            List of hex values read from the register. 
        """
        if type(header) != int or type(length) != int or type(reverse) != bool: 
            print("[ERROR] Unexpected type. Check your args.")
            return None 
        response = self.spi.xfer2([header] + [0]*length) # xfer2 is supposed to keep CS pressed for the entire transaction, although some sources differ, I use it to be safe. At worst its equivalent to xfer 
        response = [hex(octet) for octet in response][1:] # throwing away the header
        if reverse: response.reverse() 
        return response
