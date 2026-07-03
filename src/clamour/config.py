"""
Defines all configuration parameters for Clamour. 
- Tag type and ID 
- DW1000/UWB settings if applicable 
- Terminal output control 
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
SMART_TX_POWER = True      # Enable or disable smart TX power - Only works for 6.8Mbps bitrate 
TX_POWER_CONFIG = None     # Overwrites default TX power setting if different from None. MUST be a list[int] where each element is a byte value of the 0x1E register in LSB order (ex: [0x67, 0x67, 0x67, 0x67]) 

### Output control 
## Terminal
GEN_MSGS    = True     # Turn off/on general terminal output 
DEVICE_MSGS = True     # Turn off/on device management-related terminal output  
TDMA_MSGS   = True     # Turn off/on TDMA-related terminal output  
LOC_MSGS    = True     # Turn off/on localization-related terminal output  
## CSV saving 
SAVE_TO_CSV = False     # Save EKF data to csv or not 

### Anchor definition 
# Anchors are represented by dicts in a tuple 
# Anchor IDs are expected to be <=10. Coordinates are in mm. 
# temp NOTE A2 with broken usb is ID 2 
ANCHORS = ({'id': 2, 'level': 0, 'x': 0, 'y': 0, 'z': 850}, 
           {'id': 3, 'level': 0, 'x': 2130, 'y': 2280, 'z': 1060},
           {'id': 4, 'level': 0, 'x': 1950, 'y': 0, 'z': 840})