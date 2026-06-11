import sys
from pathlib import Path
from rich import print
import time 

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.dw_1000 import DW1000 

with DW1000(bus=0, cs=0) as dw: 
    dw.config_uwb_settings(2, 64, 9, 6.8, 128)
    dw.write_register([0x8D], [0, 1]) # enable RX 0x0D
    print("--- started RX ---")
    for _ in range(50): 
        status = dw.read_register([0x0F], 5) # reading sys status 0x0F
        byte2 = status[1]
        byte3 = status[2] 
        print("----------")
        print(f"bits 8-15: {byte2} | {int(byte2, base=16):08b}")
        print(f"bits 16-23: {byte3} | {int(byte3, base=16):08b}")
        if byte2=='0x6f': # replace with a more thorough check of bits 
            # good frame
            # check rxfinfo for frame legnth 
            rx_finfo = dw.read_register([0x10], 4, return_ints=True)
            print(f"{rx_finfo=}")
            rxfle_rxflen = (rx_finfo[0] | (rx_finfo[1]<<8)) & 0x3FF
            print(f"{rxfle_rxflen=}")
            rxfle = rxfle_rxflen>>7
            rxflen = rxfle_rxflen & 0x7F
            if rxfle != 0:
                print("RXFLE SHOULD BE 0, NOT SUPPORTING NON STD OPERATIONS")
                break
            print(f"ok message length {rxflen=} this is already in bytes!")
            message = dw.read_register([0x11], rxflen) # reading length found in rxflen 
            print(f"{message=}")
            break 
        time.sleep(0.2) 
    else: 
        print("did not detect a message") 
    dw.write_register([0x8D], [0x40]) # disable RX 0x0D
    print("--- closed RX ---")
