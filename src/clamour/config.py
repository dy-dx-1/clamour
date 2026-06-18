"""
Defines all configuration parameters for Clamour. 
- Tag type and ID 
- DW1000/UWB settings if applicable 
- Terminal output control 
- Anchor definition 
"""
### Tag type and ID 
TAG_TYPE = "Bitcraze" # Manufacturer of the tag. Bitcraze or Pozyx
TAG_ID = 11           # Tag IDs must be >10. This will overwrite any hardware-defined ID if it exists.  

### DW1000 and UWB config - only applicable for Bitcraze Tags
DW1000_BUS =  0            # SPI bus where the deck is connected 
DW1000_CS  =  0            # Chip select # where  the deck is connected 
UWB_CHANNEL = 2            # Any in [1,2,3,4,5,7]
UWB_PRF = 64               # 16 or 64 MHz
UWB_BITRATE = 6.8          # 110kps, 850kps or 6.8Mbps
UWB_PREAMBLE_LENGTH = 128  # Any in [64,128,256,512,1024,1536,2048,4096] symbols
UWB_PREAMBLE_CODE = 9      

### Terminal output control 
GEN_MSGS    = True     # Turn off/on general terminal output 
DEVICE_MSGS = True     # Turn off/on device management-related terminal output  
TDMA_MSGS   = True     # Turn off/on TDMA-related terminal output  
LOC_MSGS    = True     # Turn off/on localization-related terminal output  

### Anchor definition 
# Anchors are represented by dicts in a tuple 
# Anchor IDs are expected to be <=10. Coordinates are in mm. 
ANCHORS = ({'id': 4, 'level': 0, 'x': 2950, 'y': 3240, 'z': 530}, 
           {'id': 5, 'level': 0, 'x': 150, 'y': 2880, 'z': 1080},
           {'id': 27182, 'level': 0, 'x': 0, 'y': 0, 'z': 900},
           {'id': 27199, 'level': 0, 'x': 3640, 'y': 1260, 'z': 920})