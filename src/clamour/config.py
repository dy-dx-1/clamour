import numpy as np 
"""
Defines all configuration parameters for Clamour. 
- Tag type and ID 
- DW1000/UWB settings if applicable 
- IMU selection
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
SMART_TX_POWER = True      # Enable or disable smart TX power - Only works for 6.8Mbps bitrate 
TX_POWER_CONFIG = None     # Overwrites default TX power setting if different from None. MUST be a list[int] where each element is a byte value of the 0x1E register in LSB order (ex: [0x67, 0x67, 0x67, 0x67]) 

### State estimation control 
ESTIMATOR_TYPE = "EKF"     # EKF or FG (Factor Graph) 

### IMU control - Set IMU_TYPE to None for constant velocity model 
IMU_TYPE = "LSM6DSV320X" 
# The following parameters are specific to each individual hardware unit 
# Scale factor 
# NOTE TODO fix all units in this section 
IMU_ACCEL_SCALE_FACTOR = np.array([[1.0017558373609916,      0.0,                  0.0],
                                   [0.006256910346063383,    1.0018358510192535,   0.0],
                                   [-0.0050975506470306515, -0.000828014063373834, 1.0032780653565945]])
IMU_GYRO_SCALE_FACTOR  =  np.array([[1,0,0],
                                    [0,1,0],
                                    [0,0,1]])
# Initial bias from calibration, the FG estimates its drift automatically
IMU_ACCEL_INITIAL_BIAS = (-1.9653053938720833, -14.595203153973426, -2.6589320093328133) 
IMU_GYRO_INITIAL_BIAS  = (-376.4132487893914, -21.328286579486857,-206.75801625034532) 
# Covariance on the bias. Variance of the initial bias value when calibrating. 
IMU_ACCEL_INITIAL_BIAS_COV = None 
IMU_GYRO_INITIAL_BIAS_COV  = None 
# Accelerometer and Gyro bias random walk covariance (requires Allan variance analysis) 
# As of 21sept 2026, placeholders as it'll be enough to validate IMU integration. We don't dead-reckon for long without range factors to correct. 
# GTSAM expects these values to be /s as it will *s during pre-integration 
IMU_ACCEL_WALK_COV = 0.032**2 # units: (mg**2)/s
IMU_GYRO_WALK_COV  = 5.73**2  # units: (mdps**2)/s

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
# Anchor IDs are expected to be >0 and <=10. Spatial coordinates are in cm.
ANCHORS = ({'id': 1, 'level': 0, 'x': 22.3, 'y': 45.8, 'z': 219}, 
           {'id': 2, 'level': 0, 'x': 137.3, 'y': 552.0, 'z': 219},
           {'id': 5, 'level': 0, 'x': 79.3, 'y': 297.1, 'z': 219}) 
ANCHOR_POS_UNCERTAINTY = 5 # +- precision on the anchor's coordinates (cm) 


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
