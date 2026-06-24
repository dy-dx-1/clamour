import sys
from pathlib import Path
from rich import print
import time 
import struct

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.bitcraze_tag import BitcrazeTag

def compute_clock_delta(t2, t1):
    TICK_DELTA_MASK = (1 << 40) - 1
    return (t2-t1) & TICK_DELTA_MASK

with BitcrazeTag(tag_id=11, dw1000_bus=0, dw1000_cs=0, channel=2, PRF=64, bitrate=6.8, preamble_length=128, preamble_code=9) as bc: 
    dists = []
    ticks = [] 
    for _ in range(50): 
        distance = bc.compute_range(5) 

        print(f"{distance=}") 
        if distance: 
            dists.append(distance)
        time.sleep(0.5) 
    print(f"avg dist: {sum(dists)/len(dists)}")