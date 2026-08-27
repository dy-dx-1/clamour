"""
Defines all configuration parameters for Clamour. 
- Tag type and ID 
- DW1000/UWB settings if applicable 
- Terminal output control 
- Saving output to CSV
- Anchor definition 
"""
### Tag type and ID 
TAG_TYPE = "Bitcraze"      # Manufacturer of the tag. Bitcraze or Pozyx
TAG_ID = 11                # Tag IDs must be >10. This ONLY applies for BC tags. Pozyx breaks ranging when enforcing IDs, dk why, already wasted 4h+ on it. 
### DW1000 and UWB config - only applicable for Bitcraze Tags
DW1000_BUS =  0            # SPI bus where the deck is connected 
DW1000_CS  =  0            # Chip select # where  the deck is connected 
UWB_CHANNEL = 2            # Any in [1,2,3,4,5,7]
UWB_PRF = 64               # 16 or 64 MHz
UWB_BITRATE = 6.8          # 110kps, 850kps or 6.8Mbps
UWB_PREAMBLE_LENGTH = 128  # Any in [64,128,256,512,1024,1536,2048,4096] symbols
UWB_PREAMBLE_CODE = 9     
SMART_TX_POWER = False      # Enable or disable smart TX power - Only works for 6.8Mbps bitrate 
TX_POWER_CONFIG = [0x10, 0x10, 0x10, 0x10]     # Overwrites default TX power setting if different from None. MUST be a list[int] where each element is a byte value of the 0x1E register in LSB order (ex: [0x67, 0x67, 0x67, 0x67]) 

### State estimation control 
ESTIMATOR_TYPE = "FG"     # EKF or FG (Factor Graph) 

### Output control 
## Terminal
GEN_MSGS    = True     # Turn off/on general terminal output 
DEVICE_MSGS = True     # Turn off/on device management-related terminal output  
TDMA_MSGS   = True     # Turn off/on TDMA-related terminal output  
LOC_MSGS    = True     # Turn off/on localization-related terminal output  
## CSV saving 
SAVE_TO_CSV = False     # Save localization data to csv or not 

### Anchor definition 
# Anchors are represented by dicts in a tuple 
# Anchor IDs are expected to be >0 and <=10. Coordinates are in cm. 
# temp NOTE A2 with broken usb is ID 2 
ANCHORS = ({'id': 2, 'level': 0, 'x': 210, 'y': 32, 'z': 85},
           {'id': 3, 'level': 0, 'x': -35, 'y': 130, 'z': 133},
           {'id': 4, 'level': 0, 'x': 225, 'y': 350, 'z': 106},
           {'id': 5, 'level': 0, 'x': 0, 'y': 0, 'z': 90})


### ----------------------- VALIDATION CHECKS ----------------------- ### 
assert TAG_TYPE in ("Bitcraze", "Pozyx") 
assert 10<TAG_ID<0xFFFF

from .tdma.timing import SCHEDULING_SLOT_COUNT 
# Each tag's proposal slot is TAG_ID & 0xFF.  SCHEDULING_SLOT_COUNT must be
# greater than the highest deployed low-byte ID; deployed low-byte IDs must be
# unique. A smaller safe count gives tags proposal opportunities more often.

# Honestly the 0xFF mask could be removed by simply enforcing tag IDs max 255, anyways low byte must be unique to avoid conflict. To consider TODO.
assert (TAG_ID & 0xFF) < SCHEDULING_SLOT_COUNT, (
    f"TAG_ID low byte {TAG_ID & 0xFF} must be smaller than SCHEDULING_SLOT_COUNT {SCHEDULING_SLOT_COUNT}"
)

assert all([1<=anc['id']<=10 for anc in ANCHORS]) # 0 is unsupported because reserved in the code for general broadcasts 

if TX_POWER_CONFIG: # If specified, TX POWER CONFIG must be list in LSB order of bytes for 0x1E register
    assert type(TX_POWER_CONFIG) == list 
    assert len(TX_POWER_CONFIG) == 4 
    assert type(TX_POWER_CONFIG[0])== int

# Estimator, only supporting EKF or Factor Graphs 
assert ESTIMATOR_TYPE in ("EKF", "FG") # these match the types accepted and expected by the class StateEstimator 
