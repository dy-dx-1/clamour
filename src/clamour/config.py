"""
Defines config parameters for clamour. 
"""
TAG_TYPE = "Bitcraze" # Manufacturer of the tag. Bitcraze or Pozyx
TAG_ID = 11           # Tag IDs must be >10. Pozyx does not currently support ID assignement through config file. 

DW1000_BUS =  0      # If using a Bitcraze Tag, this specifies the SPI bus where it's connected 
DW1000_CS  =  0      # If using a Bitcraze Tag, this specifies the chip select # where it's connected 

SUPPRESS_ALL_MSGS = False    # Easily turn off all prints. Overrides all other types of printing. 
DEVICE_MSGS = True  # turn off/on device management-related terminal output  
TDMA_MSGS = True    # turn off/on TDMA-related terminal output  
LOC_MSGS = True    # turn off/on localization-related terminal output  