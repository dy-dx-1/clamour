import sys
from pathlib import Path

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import RPi.GPIO
import spidev 
import time 

from src.clamour.interfaces.dw_1000 import DW1000 

with DW1000(bus=0, cs=0) as dw:
    print("init") 

print("done") 