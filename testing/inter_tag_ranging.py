import sys
from pathlib import Path
from rich import print
import time 
import numpy as np 

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.bitcraze_tag import BitcrazeTag
from src.clamour.interfaces.containers import Coordinates

id = sys.argv[1] 
try: 
    if int(id)==11: 
        TAG_ID = 11 
    elif int(id)==15: 
        TAG_ID = 15 
    else: 
        raise ValueError() 
except: 
    print("invalid arg")
    quit() 


if TAG_ID == 11: 
    with BitcrazeTag(tag_id=TAG_ID, dw1000_bus=0, dw1000_cs=0, channel=2, PRF=64, bitrate=6.8,
                preamble_length=128, preamble_code=9,
                smart_tx_power=False, tx_power_settings=[0x10, 0x10, 0x10, 0x10]) as bc: 
        ### Sender tag 
        d, n_coords = bc.compute_range(15) 
        print(f"Range measured: {d}cm")
        if n_coords: 
            print(f"Neighbor coords: {n_coords}")
            print(f"Neighbor covar: {n_coords.covar}")


elif TAG_ID == 15: 
    with BitcrazeTag(tag_id=TAG_ID, dw1000_bus=1, dw1000_cs=0, channel=2, PRF=64, bitrate=6.8,
            preamble_length=128, preamble_code=9,
            smart_tx_power=False, tx_power_settings=[0x10, 0x10, 0x10, 0x10]) as bc: 
        mock_pos = Coordinates(x=-33512, y=0, z=15616513.51)
        bc.coordinates = mock_pos
        bc.coordinates.update_covar((10, -20, 30, 99, -77, 234324))
        ### Receiver tag 
        while True: 
            s_id, data = bc.receive_data() 
            if s_id: 
                print("ATTEMPTING RANGING! -------------------")  
            time.sleep(0.008) # if refresh is high enough, pretty much always get messages, timout on other side don't matter, only if we are listening 
        