import sys
from pathlib import Path
from rich import print
import time 

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.dw_1000 import DW1000 

with DW1000(bus=0, cs=0, channel=2, PRF=64, bitrate=6.8, preamble_length=128, preamble_code=9) as dw: 
    msg = dw.listen(timeout=5, return_ints=False) 
    print(msg) 

