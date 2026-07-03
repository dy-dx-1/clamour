import sys
from pathlib import Path
from rich import print
import time 

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.dw_1000 import DW1000 

with DW1000(bus=0, cs=0, channel=2, PRF=64, bitrate=6.8, preamble_length=128, preamble_code=9, smart_tx_power=True, tx_power_settings=None) as dw: 
    print(dw.read_register([0x04], 4, reverse=True)) 
    print(dw.read_register([0x1E], 4, reverse=True)) 
    #dw.write_register([0x84], [0, 0x12, 0x4, 0])
    #print(dw.read_register([0x04], 4, reverse=True)) 
