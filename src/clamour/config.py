"""
Defines config parameters for clamour. 
"""
TAG_TYPE = "Bitcraze" # Manufacturer of the tag. Bitcraze or Pozyx
TAG_ID = 11           # Tag IDs must be >10. Pozyx does not currently support ID assignement through config file. 

DW1000_BUS =  0      # If using a Bitcraze Tag, this specifies the SPI bus where it's connected 
DW1000_CS  =  0      # If using a Bitcraze Tag, this specifies the chip select # where it's connected 

GEN_MSGS    = True     # Turn off/on general terminal output 
DEVICE_MSGS = True     # Turn off/on device management-related terminal output  
TDMA_MSGS   = True     # Turn off/on TDMA-related terminal output  
LOC_MSGS    = True     # Turn off/on localization-related terminal output  

# Anchors are represented by dicts in a tuple 
# Anchor IDs are expected to be <=10. Coordinates are in mm. 
ANCHORS = ({'id': 4, 'level': 0, 'x': 2950, 'y': 3240, 'z': 530}, 
           {'id': 5, 'level': 0, 'x': 150, 'y': 2880, 'z': 1080},
           {'id': 27182, 'level': 0, 'x': 0, 'y': 0, 'z': 900},
           {'id': 27199, 'level': 0, 'x': 3640, 'y': 1260, 'z': 920})