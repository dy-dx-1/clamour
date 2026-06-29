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


with BitcrazeTag(tag_id=43981, dw1000_bus=1, dw1000_cs=0, channel=2, PRF=64, bitrate=6.8, preamble_length=128, preamble_code=9) as bc: 
    t = time.perf_counter() 
    while time.perf_counter()-t < 10: 
        bc.sendData(destination=0, payload=[1,2,3,4,5])
        time.sleep(0.1) 