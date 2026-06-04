"""
Testing bitcraze tag implementation 
"""
import sys
from pathlib import Path

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.bitcraze_tag import BitcrazeTag 
from src.clamour.interfaces.containers import DeviceCoordinates
#############################################################################
a1 = DeviceCoordinates(0, 1)
a2 = DeviceCoordinates(1, 1) 
t1 = DeviceCoordinates(13, 0) 

with BitcrazeTag(tag_id=11, dw1000_bus=0, dw1000_cs=0) as bc: 
    bc.addDevice(a1) 
    bc.addDevice(a2)
    bc.addDevice(t1) 
    print(f"{bc.device_list=}")
    print(f"{bc.is_anchor(a1.network_id)=}")
    print(f"{bc.is_anchor(a2.network_id)=}")
    print(f"{bc.is_anchor(t1.network_id)=}")
    print(f"{bc.is_anchor(bc.tag_id)=}")
    bc.clearDevices() 
    print(f"{bc.device_list=}")