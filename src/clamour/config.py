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