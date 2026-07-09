"""
Testing bitcraze tag implementation 
"""
import sys
from pathlib import Path
from rich import print 
import time 

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.bitcraze_tag import BitcrazeTag 
#############################################################################


with BitcrazeTag(tag_id=11, dw1000_bus=0, dw1000_cs=0, channel=2, PRF=64, bitrate=6.8, preamble_length=128, preamble_code=9) as bc: 
    sender_id, data = bc.receiveData()  
    print(f"Message from: {sender_id} with data: {data} ")