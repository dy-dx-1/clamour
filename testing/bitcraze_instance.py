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

fake_tag = DeviceCoordinates(12, 0)
fake_anchor_1 = DeviceCoordinates(0, 1) 
fake_anchor_2 = DeviceCoordinates(0, 1) 

bc = BitcrazeTag() 
try: 
    print("testing is_anchor")
    print("fake tag: ", BitcrazeTag.is_anchor(fake_tag.network_id)) 
    print("fake anchor1: ", BitcrazeTag.is_anchor(fake_anchor_1.network_id))
    print("fake anchor2: ", BitcrazeTag.is_anchor(fake_anchor_2.network_id))

    print("Adding devices") 
    for d in [fake_tag, fake_anchor_1, fake_anchor_2]:
        bc.addDevice(d) 
    print(f"{bc.device_list= }")
    print("clearing devices") 
    bc.clearDevices() 
    print(f"{bc.device_list= }")

    print("testing error print") 
    bc.printCurrentError("Tag.doPositioning(), returned None")
finally: 
    print("done") 
    bc.serial_con.close() 