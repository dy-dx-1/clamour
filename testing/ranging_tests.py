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

ANCHOR_ID = 2

SMART_TX_POWER = True 

power_lvls = [ [0x67, 0x67, 0x67, 0x67], [0x60,0x60,0x60,0x60], [0x58,0x58,0x58,0x58], [0x50,0x50,0x50,0x50], [0x48,0x48,0x48,0x48], [0x40,0x40,0x40,0x40], [0x30,0x30,0x30,0x30], [0x20,0x20,0x20,0x20], [0x10,0x10,0x10,0x10], [0x08,0x08,0x08,0x08], [0x00,0x00,0x00,0x00] ] 
relative_power = 100 # just to print %  

results = {} # {power_level: (success_rate, mean, std_dev)}

def get_range_measurements(smart_tx, power_cfg): # IN CM 
    with BitcrazeTag(tag_id=11, dw1000_bus=0, dw1000_cs=0, channel=2, PRF=64, bitrate=6.8,
                preamble_length=128, preamble_code=9,
                smart_tx_power=smart_tx, tx_power_settings=power_cfg) as bc: 
        # Taking 100 measurements 
        ranges = [] 
        for _ in range(100): 
            range_measurement = bc.compute_range(ANCHOR_ID)[0]
            if range_measurement: 
                ranges.append(round(range_measurement/10)) 
            time.sleep(0.05) 
    return np.array(ranges) 

for pwr_lvl in [None]: 
    # Computing stats
    ranges = get_range_measurements(SMART_TX_POWER, pwr_lvl)
    if len(ranges) != 0: 
        mean = round(np.mean(ranges)) 
        std_dev = round(np.std(ranges))
    else: 
        mean = None 
        std_dev = None
    #print(f"--- At power level: {relative_power}% ({pwr_lvl}) ---")
    #print(f"Success rate: {len(ranges)}%")
    #print(f"Mean range: {mean}cm")          # Used to compute error 
    #print(f"STD dev of range: {std_dev}cm") # 68% of measures within this range and 95% within double 
    results[relative_power] = (len(ranges), mean, std_dev)
    relative_power -= 10 
    time.sleep(0.1) 

print(results) 