import numpy as np 
"""
Defines all configuration parameters for Clamour and validates they can be used by the application. 
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
ESTIMATOR_TYPE = "FG"     # EKF or FG (Factor Graph) 

### IMU control - Set IMU_TYPE to None for constant velocity model 
IMU_TYPE = "LSM6DSV320X" 
# The following parameters are specific to each individual hardware unit 
# Scale factors are dimensionless corrections applied to measurements in mg and mdps.
IMU_ACCEL_SCALE_FACTOR_MG = np.array([[1.0017558373609916,      0.0,                  0.0],
                                     [0.006256910346063383,    1.0018358510192535,   0.0],
                                     [-0.0050975506470306515, -0.000828014063373834, 1.0032780653565945]])
IMU_GYRO_SCALE_FACTOR_MDPS  =  np.array([[1.0,0.0,0.0],
                                    [0.0,1.0,0.0],
                                    [0.0,0.0,1.0]])
# Initial bias from calibration; the FG estimates its drift automatically.
# Acceleration bias is in mg, angular-rate bias is in mdps.
IMU_ACCEL_INITIAL_BIAS_MG   = np.array([-1.9653053938720833, -14.595203153973426, -2.6589320093328133])
IMU_GYRO_INITIAL_BIAS_MDPS  = np.array([-376.4132487893914, -21.328286579486857,-206.75801625034532])
# Per-axis standard deviations of the initial bias in mg and mdps, respectively.
# NOTE TEMP VALUES AWAITING CALIB TODO 
IMU_ACCEL_INITIAL_BIAS_STD_MG = np.array([0.5, 0.5, 0.5])
IMU_GYRO_INITIAL_BIAS_STD_MDPS = np.array([1.0, 1.0, 1.0])
# Accelerometer and gyro bias random-walk covariance densities. These are
# variances per second and require Allan-variance analysis for calibration.
IMU_ACCEL_WALK_COV_MG2_PER_S = 0.032**2  # (mg^2)/s
IMU_GYRO_WALK_COV_MDPS2_PER_S = 5.73**2  # (mdps^2)/s

### Output control 
## Terminal
GEN_MSGS    = True     # Turn off/on general terminal output 
DEVICE_MSGS = True     # Turn off/on device management-related terminal output  
TDMA_MSGS   = True     # Turn off/on TDMA-related terminal output  
LOC_MSGS    = True     # Turn off/on localization-related terminal output  
## CSV saving 
SAVE_TO_CSV = True     # Save localization data to csv or not 

### Anchor definition 
# Anchors are represented by dicts in a tuple 
# Anchor IDs are expected to be >0 and <=10. Spatial coordinates are in cm.
ANCHORS = ({'id': 1, 'level': 0, 'x': 473.6, 'y': 219.4, 'z': 121.3}, 
           {'id': 2, 'level': 0, 'x': 137.3, 'y': 552.0, 'z': 219}, 
           {'id': 3, 'level': 0, 'x': 137.2, 'y':  15.0, 'z': 12.8}, 
           {'id': 4, 'level': 0, 'x': 473.6, 'y': 219.4, 'z': 30.2}, 
           {'id': 5, 'level': 0, 'x':  79.3, 'y': 297.1, 'z': 219}, 
           {'id': 6, 'level': 0, 'x': 458.2, 'y': 604.0, 'z': 146}, 
           {'id': 7, 'level': 0, 'x':  22.3, 'y':  45.8, 'z': 219}, 
           {'id': 8, 'level': 0, 'x': 504.5, 'y': 419.4, 'z': 107})
ANCHOR_POS_UNCERTAINTY_CM = 5.0  # +/- precision on each anchor coordinate, in cm

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

# IMU parameter types and shapes
# all dtypes must be floats to work well with GTSAM 
assert IMU_TYPE is None or (IMU_TYPE in ["LSM6DSV320X"])
assert isinstance(IMU_ACCEL_SCALE_FACTOR_MG, np.ndarray)
assert np.issubdtype(IMU_ACCEL_SCALE_FACTOR_MG.dtype, float)
assert IMU_ACCEL_SCALE_FACTOR_MG.shape == (3, 3)
assert isinstance(IMU_GYRO_SCALE_FACTOR_MDPS, np.ndarray)
assert np.issubdtype(IMU_GYRO_SCALE_FACTOR_MDPS.dtype, float)
assert IMU_GYRO_SCALE_FACTOR_MDPS.shape == (3, 3)
assert isinstance(IMU_ACCEL_INITIAL_BIAS_MG, np.ndarray)
assert np.issubdtype(IMU_ACCEL_INITIAL_BIAS_MG.dtype, float)
assert IMU_ACCEL_INITIAL_BIAS_MG.shape == (3,)
assert isinstance(IMU_GYRO_INITIAL_BIAS_MDPS, np.ndarray)
assert np.issubdtype(IMU_GYRO_INITIAL_BIAS_MDPS.dtype, float)
assert IMU_GYRO_INITIAL_BIAS_MDPS.shape == (3,)
if ESTIMATOR_TYPE == "FG":
    assert IMU_ACCEL_INITIAL_BIAS_STD_MG is not None
    assert IMU_GYRO_INITIAL_BIAS_STD_MDPS is not None
    assert isinstance(IMU_ACCEL_INITIAL_BIAS_STD_MG, np.ndarray)
    assert np.issubdtype(IMU_ACCEL_INITIAL_BIAS_STD_MG.dtype, float)
    assert IMU_ACCEL_INITIAL_BIAS_STD_MG.shape == (3,)
    assert np.all(np.isfinite(IMU_ACCEL_INITIAL_BIAS_STD_MG))
    assert np.all(IMU_ACCEL_INITIAL_BIAS_STD_MG > 0)
    assert isinstance(IMU_GYRO_INITIAL_BIAS_STD_MDPS, np.ndarray)
    assert np.issubdtype(IMU_GYRO_INITIAL_BIAS_STD_MDPS.dtype, float)
    assert IMU_GYRO_INITIAL_BIAS_STD_MDPS.shape == (3,)
    assert np.all(np.isfinite(IMU_GYRO_INITIAL_BIAS_STD_MDPS))
    assert np.all(IMU_GYRO_INITIAL_BIAS_STD_MDPS > 0)
assert isinstance(IMU_ACCEL_WALK_COV_MG2_PER_S, float)
assert isinstance(IMU_GYRO_WALK_COV_MDPS2_PER_S, float)
