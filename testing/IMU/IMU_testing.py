import smbus2
from typing import Literal 
import numpy as np
import csv 
import time

######################## DATASHEET CONFIGURATION BITS FOR IMU ########################
# ODR bit value to set for a desired rate in Hz 
# Applies for the CTRL1 and CTRL2 registers (accel and gyro) 
# NOTE Not all rates are compatible with all modes 
# Although high-performance mode is compatible with all rates
# Currently only supporting high-perf so it's ok, but careful in the future 
ODR_FROM_HZ = {1.875: 0b0001,
                7.5:   0b0010, 
                15:    0b0011,
                30:    0b0100,
                60:    0b0101,
                120:   0b0110,
                240:   0b0111,
                480:   0b1000,
                960:   0b1001,
                1920:  0b1010,
                3840:  0b1011,
                7680:  0b1100}

# CTRL6 bits to set depending on desired DPS range for the Gyro 
# Bit in position 4 is needed for correct operation of device (see p.69)
GYRO_DPS_SCALE_BITS = {250:  0b1001, 
                        500:  0b1010,
                        1000: 0b1011, 
                        2000: 0b1100, 
                        4000: 0b1101}

# CTRL8 bits to set depending on accelerometer scale in gs 
ACCEL_SCALE_BITS = {2:  0b00, 
                    4:  0b01, 
                    8:  0b10, 
                    16: 0b11}

# CONVERSION FACTORS mg/LSB and mdps/LSB (p.12) DEPENDING ON SCALES 
GYRO_SCALE_CONVERSION = {250:  8.75, 
                            500:  17.50,
                            1000: 35, 
                            2000: 70, 
                            4000: 140}
ACCEL_SCALE_CONVERSION = {2:  0.061, 
                            4:  0.122, 
                            8:  0.244, 
                            16: 0.488}

def uint16_to_int16(value:int)->int: 
    """
    Converts an unsigned 2-byte int to a signed int, following two's complement 
    """ 
    return value - 0x10000 if value & 0x8000 else value

class LSM6DSV320X: 
    ### ACCEL/GYRO COVARIANCE  
    # Measurement values (from datasheet) (GTSAM expects them in setAccelerometerCovariance/setGyroscopeCovariance) 
    # GTSAM expects a density as it will multiply per 1/delta_t during pre-integration 
    accel_covar_density_mg = 0.060**2    # VALUE SPECIFIC TO LOW-G, HIGH-PERF mode! Units: (mg**2)*s 
    gyro_covar_density_mdps  = 3.8**2 # Units: (mdps**2)*s
    # Bias random walk covariance - PLACEHOLDER VALUES - TO BE ESTIMATED WITH ALLAN VARIANCE ANALYSIS 
    # As of 21sept 2026, placeholders as it'll be enough to validate IMU integration. We don't dead-reckon for long without range factors to correct. 
    # GTSAM expects these values to be /s as it will *s during pre-integration 
    # To be used in GTSAM setBiasAccCovariance/setBiasOmegaCovariance
    accel_bias_random_walk_covar_mg =  0.032**2   # Guessed placeholders, units: (mg**2)/s
    gyro_bias_random_walk_covar_mdps =   5.73**2  # Guessed placeholders, units: (mdps**2)/s

    ### ACCEL/GYRO CALIBRATION VALUES
    ## NOTE THESE ARE PRELIMINARY & SPECIFIC TO THE UNIQUE PHYSICAL UNIT THEY WERE CALCULATED FOR! (2026-09-10)
    ## Units are in mgs and mdps. Format is x,y,z. Match the equation: calibrated_measure = scale_factor @ (raw_measure-bias)
    # Scale Factor (always used internally) 
    accel_scale_factor = np.array([[1.0017558373609916,      0.0,                  0.0],
                                   [0.006256910346063383,    1.0018358510192535,   0.0],
                                   [-0.0050975506470306515, -0.000828014063373834, 1.0032780653565945]])
    gyro_scale_factor  =  np.array([[1,0,0],
                                    [0,1,0],
                                    [0,0,1]]) # didn't have rate table, so this cannot be calibrated. Not as important as the others. 
    # Bias (Fixed initial value, does not consider bias drift) 
    # If using GTSAM, use these values to initialize bias tracking & let the FG handle drift-estimation afterwards 
    # convert_accel_bytes_to_mgs() and convert_gyro_bytes_to_mdps() have params to return or not bias compensated values for this. 
    # as GTSAM will use non-compensated values so it can apply it's own estimated bias 
    accel_bias = np.array([-1.9653053938720833, -14.595203153973426, -2.6589320093328133])
    gyro_bias  = np.array([-376.4132487893914, -21.328286579486857,-206.75801625034532])
    
    def __init__(self, ODR_rate:Literal['7.5', 15, 30, 60, 120, 240, 480, 960, 1920, 3840, 7680], accelerometer_scale:Literal[2,4,8,16], gyro_dps_scale: Literal[250,500,1000,2000,4000],
                 SDO_state:bool,  i2c_bus:int=1): 
        """
        I2C interface with the LSM6DSV320X IMU. 

        IMPORTANT CONSIDERATIONS: 
        - ALWAYS USE WITH A CONTEXT MANAGER 
        - Only supporting high performance mode accel/gyro
        - On instanciation, ensure args are defined according to manual. No internal safeguards. 
        - Accel/gyro values are always returned in **mg** and **mdps** units 
            - Values are always scale-corrected
            - Bias correction is optional so they can be applied by an external estimator if desired (ex: GTSAM). See class attributes.
        - Remember that accel/gyro scale factor, bias and bias random walk covar all are HARDWARE UNIT SPECIFIC. 
            - NOTE TODO: Should shift these from class attributes to facilitate per-unit definition? 

        ARGS: 
        - ODR_rate: Output Data Rate in Hz (must match High Perf mode see p.65 manual)
        - accelerometer_scale: Full scale of the accelerometer measurements in g's 
        - gyro_dps_scale: Full scale of the gyro measurements in dps 
        - SDO_state: True for 'HIGH' and False for 'LOW', defines the Target Address 
        - i2c_bus: I2C bus, 1 by default 
        """
        self.TAD = 0x6A if not SDO_state else 0x6B 
        self.bus = smbus2.SMBus(i2c_bus)
        if not self.validate_connection(): 
            raise RuntimeError("Could not validate connection to the LSM6DSV320X IMU.")
        else:
            self.configure(accelerometer_scale, gyro_dps_scale, ODR_rate) 
            print("SUCCESSFULLY CONNECTED TO IMU")

    def __enter__(self):
        return self 

    def __exit__(self, exc_type, exc_val, exc_tb): 
        self.bus.close() 

    def __del__(self): 
        try: 
            self.bus.close() 
        except: 
            pass

    @staticmethod
    def _apply_scalar_calibration(x:float, y:float, z:float, bias:np.ndarray,
                                  scale_factor:np.ndarray, apply_bias:bool)->np.ndarray:
        """
        Apply bias correction and a 3x3 scale-factor matrix without NumPy matrix multiplication.

        FIFO conversion runs once per sensor-axis sample. Scalar arithmetic avoids creating an intermediate 
        input vector and dispatching a small matrix multiplication for every sample. 
        While remaining equivalent to the equation calibrated_measure = scale_factor @ (raw_measure-bias)
        """
        if apply_bias:
            x -= bias[0]
            y -= bias[1]
            z -= bias[2]

        calibrated_x = (scale_factor[0, 0] * x +
                        scale_factor[0, 1] * y +
                        scale_factor[0, 2] * z)
        calibrated_y = (scale_factor[1, 0] * x +
                        scale_factor[1, 1] * y +
                        scale_factor[1, 2] * z)
        calibrated_z = (scale_factor[2, 0] * x +
                        scale_factor[2, 1] * y +
                        scale_factor[2, 2] * z)
        return np.array([calibrated_x, calibrated_y, calibrated_z])

    def convert_accel_bytes_to_mgs(self, x:int, y:int, z:int, apply_bias:bool)->np.ndarray: 
        """
        Convert packed unsigned 16-bit XYZ words to signed, calibrated, mgs in vector format.
        Set **apply_bias** to False to NOT correct for bias (ex: if using GTSAM, the FG will apply the ConstantBias on it's end)
        """
        return self._apply_scalar_calibration(
            self.LSB_TO_MG * uint16_to_int16(x),
            self.LSB_TO_MG * uint16_to_int16(y),
            self.LSB_TO_MG * uint16_to_int16(z),
            self.accel_bias,
            self.accel_scale_factor,
            apply_bias,
        )

    def convert_gyro_bytes_to_mdps(self, pitch:int, roll:int, yaw:int, apply_bias:bool)->np.ndarray: 
        """
        Convert packed unsigned 16-bit XYZ words to signed, calibrated, mdps in vector format.
        Set **apply_bias** to False to NOT correct for bias (ex: if using GTSAM, the FG will apply the bias on it's end)
        """
        return self._apply_scalar_calibration(
            self.LSB_TO_MDPS * uint16_to_int16(pitch),
            self.LSB_TO_MDPS * uint16_to_int16(roll),
            self.LSB_TO_MDPS * uint16_to_int16(yaw),
            self.gyro_bias,
            self.gyro_scale_factor,
            apply_bias,
        )

    def validate_connection(self): 
        """
        Reads the WHO_AM_I register (0x0F) and checks it's value to confirm if a connection is properly established. 
        """
        try: 
            check = (self.bus.read_byte_data(self.TAD, 0x0F) == 0x73)
        except: 
            check = False 
        return check 

    def configure(self, accel_scale:int, gyro_scale:int, ODR_rate:int): 
        """
        Configures the settings of the IMU and the internal dependent conversion factors. Args are taken from __init__
        
        **Currently only supporting high performance mode accel/gyro**
        
        Affects the following registers: 
        - CTRL3 (0x12) 
        - CTRL1 (0x10) 
        - CTRL2 (0x11) 
        - CTRL6 (0x15)
        - CTRL8 (0x17) 
        - FUNCTIONS_ENABLE (0x50)
        """
        # TODO check if replace all writes by write only if read differs
        ### Ensure BDU and IF_INC are turned on in the CTRL3 reg as we depend on these in this class. 
        if self.bus.read_byte_data(self.TAD, 0x12)!=0x44: self.bus.write_byte_data(self.TAD, 0x12, 0x44)

        ### Some configs need to be done with the accelerometer and gyro in power down mode, so we first turn them off 
        self.bus.write_byte_data(self.TAD, 0x10, 0x00)
        self.bus.write_byte_data(self.TAD, 0x11, 0x00) 

        ### CTRL6 (0x15) Gyro bandwidth 
        lpf1_bw = 0b0000 # NOTE low passfilter tuning NOT DONE, this is default 
        fs_g    = GYRO_DPS_SCALE_BITS[gyro_scale] 
        self.LSB_TO_MDPS = GYRO_SCALE_CONVERSION[gyro_scale]
        self.bus.write_byte_data(self.TAD, 0x15, lpf1_bw<<4|fs_g)

        ### CTRL8 (0x17) Accelerometer scale 
        # NOTE currently not touching HP_LPF2_XL_BW_2 
        self.LSB_TO_MG = ACCEL_SCALE_CONVERSION[accel_scale]
        fs_xl = ACCEL_SCALE_BITS[accel_scale] 
        self.bus.write_byte_data(self.TAD, 0x17, fs_xl) 

        ### Accelerometer control reg 1 - CTRL1 - 0x10 
        ### AND 
        ### Gyroscope control reg 2 - CTRL2 - 0x11 
        # The 4 MSBs will all be 0 as long as we only support high-perf mode
        mode = 0b0000 # NOTE ONLY SUPPORTING HIGH-PERF MODE CURRENTLY 
        ODR_bits = ODR_FROM_HZ[ODR_rate]
        self.bus.write_byte_data(self.TAD, 0x10, mode<<4 | ODR_bits)
        self.bus.write_byte_data(self.TAD, 0x11, mode<<4 | ODR_bits)

        ### Enable timestamp - FUNCTIONS_ENABLE - 0x50
        # NOTE for now not touching other functions 
        current = self.bus.read_byte_data(self.TAD, 0x50) 
        TIMESTAMP_EN = 1<<6 
        self.bus.write_byte_data(self.TAD, 0x50, current|TIMESTAMP_EN)

        ### Configuring FIFO 
        self.fifo_config(data_freq = ODR_rate) 
        print(f"CONFIG COMPLETE, TEMP READING: {self.get_temp()}") 

    def fifo_config(self, data_freq:int): 
        ### We'll reset the FIFO on init to ensure it starts back up, else it gets disabled after 1 read
        self.bus.write_byte_data(self.TAD, 0x0A, 0) # register CTRL4
        ### FIFO_CTRL1 - 0x07 
        ## 1 LSB = 7 bytes in the FIFO. Max capacity without compression is 1.5KB 
        ## NOTE currently setting it ~50% just as placeholder to give time to empty it before full. Can be tuned in future. 
        self.bus.write_byte_data(self.TAD, 0x07, 0x6B)
        ### FIFO_CTRL2 - 0x08 
        STOP_ON_WTM =      0b0<<7 # Limits the depth to the watermark, leaving this off as our WTM serves as warning 
        FIFO_COMPR_RT_EN = 0b0<<6 # Disable compression 
        ODR_CHG_EN =       0b0<<4 # Batch ODR CHANGE sensor in FIFO 
        UNCOMPR_RATE =    0b00<<1 # Configure compression algorithm 
        self.bus.write_byte_data(self.TAD, 0x08, STOP_ON_WTM|FIFO_COMPR_RT_EN|ODR_CHG_EN|UNCOMPR_RATE) # other bits must be 0 
        ### FIFO_CTRL3 - 0x09 
        ## Controls write frequency in FIFO for gyro and accel 
        ## keeping the same freq as the selected ODR 
        value = ODR_FROM_HZ[data_freq]<<4 | ODR_FROM_HZ[data_freq]
        self.bus.write_byte_data(self.TAD, 0x09, value)
        ### FIFO_CTRL4 - 0x0A 
        ## Controls timestamp, temperature, EIS batching and FIFO mode 
        DEC_TS_BATCH = 0b01<<6 # Batching timestamps, decimation 1 
        ODR_T_BATCH =  0b00<<4 # Not batching temp 
        G_EIS_FIFO_EN = 0b0<<3 # Not batching EIS 
        FIFO_MODE = 0b110      # Continuous mode (overwrites oldest data when full) 
        self.bus.write_byte_data(self.TAD, 0x0A, DEC_TS_BATCH|ODR_T_BATCH|G_EIS_FIFO_EN|FIFO_MODE)
        ### INT1_CTRL and INT2_CTRL - 0x0D and 0x0E 
        ## Can be used to enable interrupts on INT1 when FIFO full
        ## NOTE currently unused 
    
    def FIFO_past_WTM(self)->bool: 
        """Checks if the FIFO filling is equal to or greater than the set watermark"""
        return bool(self.bus.read_byte_data(self.TAD, 0x1C) & 0x80) # FIFO_STATUS2 register 

    def get_FIFO_count(self)->int: 
        """Returns the number of words in the FIFO"""
        ### Checking DIFF_FIFO which is split between FIFO_STATUS1 and FIFO_STATUS2 registers 
        ## It gives the number of words (1 word = 7 bytes) that are in FIFO 
        lo, st2 = self.bus.read_i2c_block_data(self.TAD, 0x1B, 2)  # STATUS1+STATUS2 together
        return min(((st2 & 0x01) << 8) | lo, 256)

    def read_FIFO(self, apply_bias:bool)->list[tuple]: 
        """
        Reads all of the data present in the FIFO. 

        The FIFO can hold up to 256 words of uncompressed 6 byte data (1536bytes). A FIFO word is 7 bytes, but the 1 byte of TAG info is stored separately.
        
        Set **apply_bias** to False to NOT correct for bias (ex: if using GTSAM, the FG will apply the ConstantBias on it's end)
        
        RETURNS:
        - A list of tuples in the form [(timestamp, accel_data, gyro_data), ...]
            - Missing data for a sample is returned as None.
            - accel_data is in mg's, gyro_data is in mdps and timestamp is in BYTES
                (so that deltas can be calculated before converting twice).
        """
        samples = []
        previous_tag_cnt = None # Used to group samples that belong together temporally 
        current_sample_idx = 0  # The idx groups samples temporally. TODO track at a class level to ensure coherence between read_FIFO calls? Or would become too big? Check if needed when pre-integration is setup. 
        ### Checking how many words are in the FIFO 
        diff_FIFO = self.get_FIFO_count() 
        ### Reading FIFO_DATA_OUT_TAG and DATA registers (automatically wraps around with block read)
        residual_words = diff_FIFO 
        while residual_words>0: 
            n_words = min(4, residual_words) # read in chunks of 4 until we can't. read_i2c_block_data can read max of 32 bytes at a time
            FIFO_data = self.bus.read_i2c_block_data(self.TAD, 0x78, 7*n_words) 
            for i in range(n_words): 
                ## For each word in the data, determine what type it is and deconstruct it accordingly
                word = FIFO_data[i*7:(i+1)*7]
                tag_type =  word[0] >> 3       # 5 MSBs 
                tag_cnt  = (word[0]>>1) & 0b11 # Bits 1 and 2 
                # Detect transition to new FIFO timeslot 
                # this assumes that *any* change in TAG_CNT corresponds to a new timeslot (verified through testing) 
                # in other words, TAG_CNT can only increase by units of 1, sequentially, until it wraps after 3 
                if previous_tag_cnt is not None and tag_cnt != previous_tag_cnt: 
                    current_sample_idx += 1 
                # Extract and save data 
                X_data = (word[2]<<8) | word[1]
                Y_data = (word[4]<<8) | word[3]
                Z_data = (word[6]<<8) | word[5]
                if tag_type==0x00: # FIFO empty (can happen due to read timing differences), stop reading 
                    residual_words = 0 
                    break 
                # Ensuring that a sample exists for this current_sample_idx 
                # While loop behaves as an if, but is more robust in case of an unexpected >1 idx jump 
                while current_sample_idx >= len(samples):
                    samples.append([None, None, None])
                sample = samples[current_sample_idx]
                if tag_type==0x01: # According to p.114 datasheet 
                    sample[2] = self.convert_gyro_bytes_to_mdps(X_data, Y_data, Z_data, apply_bias)
                elif tag_type==0x02: 
                    sample[1] = self.convert_accel_bytes_to_mgs(X_data, Y_data, Z_data, apply_bias)
                elif tag_type==0x04: 
                    sample[0] = X_data | (Y_data<<16) # Timestamp is just 4 bytes so we take the first 4
                else: 
                    print(f"unknown type in FIFO!: {hex(tag_type)}")
                previous_tag_cnt = tag_cnt
            residual_words -= n_words 
        return [tuple(sample) for sample in samples]

    def get_temp(self): 
        """
        Reads the current temperature according to the 0x20 and 0x21 registers. 
        Result in degrees C.  
        """
        # The info is 2 bytes. Each stored in 0x20 and 0x21 respectively, with 0x20 being the lower one. 
        # read_word_data reads 2 bytes and treats the first one as the lower one, so no further rearranging needed. 
        raw = self.bus.read_word_data(self.TAD, 0x20) 
        return uint16_to_int16(raw) / 256 + 25 # Units based on p.16 of user manual 

    def get_pitch_roll_yaw_speeds(self, apply_bias=True)->np.ndarray: 
        """
        Gets the raw angular rate (in mdps) for the:
        - X (pitch) axis from the 0x22 and 0x23 registers. 
        - Y (roll)  axis from the 0x24 and 0x25 registers. 
        - Z (yaw)   axis from the 0x26 and 0x27 registers. 

        **The conversion units used depend on the selected gyro dps bandwidth.** 

        Set **apply_bias** to False to NOT correct for bias (ex: if using GTSAM, the FG will apply the ConstantBias on it's end)
        """
        data = self.bus.read_i2c_block_data(self.TAD, 0x22, 6)
        return self.convert_gyro_bytes_to_mdps(pitch = data[0] | (data[1] << 8),
                                               roll  = data[2] | (data[3] << 8),
                                               yaw   = data[4] | (data[5] << 8), apply_bias=apply_bias)

    def get_x_y_z_accel(self, apply_bias=True)->np.ndarray:
        """
        Gets the raw linear acceleration **(in mg)** for the:
        - X axis from the 0x28 and 0x29 registers. 
        - Y axis from the 0x2A and 0x2B registers. 
        - Z axis from the 0x2C and 0x2D registers. 

        **The conversion units used depend on the configured accelerometer scale.**   

        Set **apply_bias** to False to NOT correct for bias (ex: if using GTSAM, the FG will apply the ConstantBias on it's end)
        """
        data = self.bus.read_i2c_block_data(self.TAD, 0x28, 6)
        return self.convert_accel_bytes_to_mgs(x = data[0] | (data[1] << 8),
                                               y = data[2] | (data[3] << 8),
                                               z = data[4] | (data[5] << 8), apply_bias=apply_bias) 

    def get_timestamp(self)->int: 
        """
        Gets the timestamp data from the 0x40, 0x41, 0x42 and 0x43 registers. 
        Returns timestamp in microseconds 
        """
        # From p.85, the conversion is 1LSB=21.7microseconds 
        raw_bytes = bytes(self.bus.read_i2c_block_data(self.TAD, 0x40, 4))
        return int.from_bytes(raw_bytes, 'little')*21.7

if __name__=="__main__":
    with LSM6DSV320X(ODR_rate=120, accelerometer_scale=2, gyro_dps_scale=500, SDO_state=False) as imu: 
        data = [("timestamp", "accel", "gyro")]
        # Recording data to for straight line test
        try:
            print("Ready to record, press ctrl-c to start")
            while True:
                pass
        except KeyboardInterrupt: 
            pass 
        print("RECORDING! Press ctrl-c to stop and save data")
        t1 = time.perf_counter() 
        # Clearing previous stored data in FIFO to begin
        imu.read_FIFO(apply_bias=False) 
        try: 
            while True: 
                # At 120Hz, the 1.5KB FIFO will fill in ~0.7s 
                # 1536bytes /3 elements per reading (timestamp, accel, gyro) / 6bytes per reading = ~85 readings total 
                # 85readings/120 readings per sec = 0.7s to fill 
                if imu.get_FIFO_count()>=170:
                    fifo = imu.read_FIFO(apply_bias=False) 
                    data.extend(fifo)
                time.sleep(0.05) # Giving CPU time to breathe  
        except KeyboardInterrupt: 
            print("\nCTRL-C detected, stopping loop") 
            # Need to do a final read to get the data that we may have missed during the keyboard interrupt
            fifo = imu.read_FIFO(apply_bias=False) 
            data.extend(fifo)
        # Now have a list of all the data available in this form [("timestamp", "accel", "gyro"), ...] 
        # Save it to csv 
        with open("straight_line.csv", 'w', newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(data) 
            print(f"Data saved to CSV. Experiment lasted: {(time.perf_counter()-t1):.2f}")
