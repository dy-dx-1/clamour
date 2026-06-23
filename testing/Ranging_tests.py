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
    for _ in range(20):
        T1, T2, T3, R1, R2, R3, T_r1, T_rp1, T_rp2, T_r2, tof_ticks, distance = bc.compute_range(5) 
        #print("TAG stuff")
        #print("T1 =", T1)
        #print("R2 =", R2)
        #print("T3 =", T3)
        #print("T_r1 = ", T_r1)
        #print("T_rp2 =", T_rp2)
        #print("ANCHOR stuff")
        #print("R1 =", int.from_bytes(R1,'little'))
        #print("T2 =", int.from_bytes(T2,'little'))
        #print("R3 =", int.from_bytes(R3,'little'))
        #print("T_rp1 =", T_rp1)
        #print("T_r2  =", T_r2)

        #print("tof_ticks =", tof_ticks)
        print(f"{distance=}") 
        dists.append(distance)
        time.sleep(0.2) 
    print(f"avg: {sum(dists)/len(dists)}")