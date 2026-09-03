import smbus2
from typing import Literal 

def unsigned_to_signed(value:int)->int: 
    """
    Converts an unsigned 2-byte int to a signed int, following two's complement 
    """ 
    return value - 0x10000 if value & 0x8000 else value

class LSM6DSV320X: 
    # ODR bit value to set for a desired rate in Hz 
    # These values apply for the CTRL1 and CTRL2 registers (accel and gyro) 
    # NOTE Not all rates are compatible with all modes 
    # Although high-performance mode is compatible with all rates
    # Currently only supporting high-perf so it's ok, but careful in the future 
    ODR_from_HZ = {1.875: 0b0001,
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

    def __init__(self, ODR_rate:Literal['7.5', 15, 30, 60, 120, 240, 480, 960, 1920, 3840, 7680], accelerometer_scale:Literal[2,4,8,16], gyro_dps_scale: Literal[250,500,1000,2000,4000],
                 SDO_state:bool,  i2c_bus:int=1): 
        """
        I2C interface with the LSM6DSV320X IMU. 

        IMPORTANT CONSIDERATIONS: 
        - ALWAYS USE WITH A CONTEXT MANAGER 
        - Currently only supporting high performance mode accel/gyro
        - Ensure arg values are properly selected according to manual. No internal safeguards. 

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
        fs_g    = self.GYRO_DPS_SCALE_BITS[gyro_scale] 
        self.LSB_TO_MDPS = self.GYRO_SCALE_CONVERSION[gyro_scale]
        self.bus.write_byte_data(self.TAD, 0x15, lpf1_bw<<4|fs_g)

        ### CTRL8 (0x17) Accelerometer scale 
        # NOTE currently not touching HP_LPF2_XL_BW_2 
        self.LSB_TO_MG = self.ACCEL_SCALE_CONVERSION[accel_scale]
        fs_xl = self.ACCEL_SCALE_BITS[accel_scale] 
        self.bus.write_byte_data(self.TAD, 0x17, fs_xl) 

        ### Accelerometer control reg 1 - CTRL1 - 0x10 
        ### AND 
        ### Gyroscope control reg 2 - CTRL2 - 0x11 
        # The 4 MSBs will all be 0 as long as we only support high-perf mode
        mode = 0b0000 # NOTE ONLY SUPPORTING HIGH-PERF MODE CURRENTLY 
        ODR_bits = self.ODR_from_HZ[ODR_rate]
        self.bus.write_byte_data(self.TAD, 0x10, mode<<4 | ODR_bits)
        self.bus.write_byte_data(self.TAD, 0x11, mode<<4 | ODR_bits)

        ### Enable timestamp - FUNCTIONS_ENABLE - 0x50
        # NOTE for now not touching other functions 
        current = self.bus.read_byte_data(self.TAD, 0x50) 
        TIMESTAMP_EN = 1<<6 
        self.bus.write_byte_data(self.TAD, 0x50, current|TIMESTAMP_EN)
        print(f"CONFIG COMPLETE, TEMP READING: {self.get_temp()}") 

    def fifo_config(self, data_freq:int): 
        ### FIFO_CTRL1 - 0x07 
        ## 1 LSB = 7 bytes in the FIFO. Max capacity without compression is 1.5KB 
        ## NOTE currently setting it ~75% just as placeholder to give time to empty it before full. Can be tuned in future. 
        self.bus.write_byte_data(self.TAD, 0x07, 0xA0)
        ### FIFO_CTRL2 - 0x08 
        STOP_ON_WTM =      0b0<<7 # Limits the depth to the watermark, leaving this off as our WM serves as warning 
        FIFO_COMPR_RT_EN = 0b0<<6 # Disable compression 
        ODR_CHG_EN =       0b0<<4 # Batch ODR CHANGE sensor in FIFO 
        UNCOMPR_RATE =    0b00<<1 # Configure compression algorithm 
        self.bus.write_byte_data(self.TAD, 0x08, STOP_ON_WTM&FIFO_COMPR_RT_EN&ODR_CHG_EN&UNCOMPR_RATE) # other bits must be 0 
        ### FIFO_CTRL3 - 0x09 
        ## Controls write frequency in FIFO for gyro and accel 
        ## keeping the same freq as the selected ODR 
        value = self.ODR_from_HZ[data_freq]<<4 | self.ODR_from_HZ[data_freq]
        self.bus.write_byte_data(self.TAD, 0x09, value)
        ### FIFO_CTRL4 - 0x0A 
        ## Controls timestamp, temperature, EIS batching and FIFO mode 
        DEC_TS_BATCH = 0b01<<6 # Batching timestamps, decimation 1 # NOTE confirm
        ODR_T_BATCH = 0b00<<4  # Not batching temp 
        G_EIS_FIFO_EN = 0b0<<3 # Not batching EIS 
        FIFO_MODE = 0b001      # FIFO mode (stops when full) 
        self.bus.write_byte_data(self.TAD, 0x0A, DEC_TS_BATCH&ODR_T_BATCH&G_EIS_FIFO_EN&FIFO_MODE)
        ### INT1_CTRL and INT2_CTRL - 0x0D and 0x0E 
        ## Can be used to enable interrupts on INT1 when FIFO full
        ## NOTE currently unused 
    
    def FIFO_past_WTM(self)->bool: 
        """Checks if the FIFO filling is equal to or greater than the set watermark"""
        return bool(self.bus.read_byte_data(self.TAD, 0x1C) & 0x80) # FIFO_STATUS2 register 

    def get_temp(self): 
        """
        Reads the current temperature according to the 0x20 and 0x21 registers. 
        Result in degrees C.  
        """
        # The info is 2 bytes. Each stored in 0x20 and 0x21 respectively, with 0x20 being the lower one. 
        # read_word_data reads 2 bytes and treats the first one as the lower one, so no further rearranging needed. 
        raw = self.bus.read_word_data(self.TAD, 0x20) 
        return unsigned_to_signed(raw) / 256 + 25 # Units based on p.16 of user manual 

    def get_pitch_roll_yaw_speeds(self)->tuple[float, float, float]: 
        """
        Gets the raw angular rate (in mdps) for the:
        - X (pitch) axis from the 0x22 and 0x23 registers. 
        - Y (roll)  axis from the 0x24 and 0x25 registers. 
        - Z (yaw)   axis from the 0x26 and 0x27 registers. 

        **The conversion units used depend on the selected gyro dps bandwidth.** 
        """
        data = self.bus.read_i2c_block_data(self.TAD, 0x22, 6)
        pitch = unsigned_to_signed(data[0] | (data[1] << 8)) 
        roll  = unsigned_to_signed(data[2] | (data[3] << 8)) 
        yaw   = unsigned_to_signed(data[4] | (data[5] << 8)) 
        return pitch*self.LSB_TO_MDPS, roll*self.LSB_TO_MDPS, yaw*self.LSB_TO_MDPS

    def get_x_y_z_accel(self)->tuple[float, float, float]:
        """
        Gets the raw linear acceleration (in mg) for the:
        - X axis from the 0x28 and 0x29 registers. 
        - Y axis from the 0x2A and 0x2B registers. 
        - Z axis from the 0x2C and 0x2D registers. 

        **The conversion units used depend on the configured accelerometer scale.**         
        """
        data = self.bus.read_i2c_block_data(self.TAD, 0x28, 6)
        x = unsigned_to_signed(data[0] | (data[1] << 8))
        y = unsigned_to_signed(data[2] | (data[3] << 8))
        z = unsigned_to_signed(data[4] | (data[5] << 8))
        return x * self.LSB_TO_MG, y * self.LSB_TO_MG, z * self.LSB_TO_MG

    def get_timestamp(self)->int: 
        """
        Gets the timestamp data from the 0x40, 0x41, 0x42 and 0x43 registers. 
        Returns timestamp in microseconds 
        """
        # From p.85, the conversion is 1LSB=21.7microseconds 
        raw_bytes = bytes(self.bus.read_i2c_block_data(self.TAD, 0x40, 4))
        return int.from_bytes(raw_bytes, 'little')*21.7

with LSM6DSV320X(ODR_rate=120, accelerometer_scale=2, gyro_dps_scale=500, SDO_state=False) as imu: 
    
    import time 
    import math
    a = time.perf_counter() 
    last_ts = imu.get_timestamp()*1e-6 # just for testing, this timestamp is not guaranteed to align with any readings because of interface
    speeds = (0,0,0) 
    pos = (0,0,0) 
    angles = (0,0,0)
    while time.perf_counter()-a < 5: 
        raw_accels, raw_ws, ts = imu.get_x_y_z_accel(), imu.get_pitch_roll_yaw_speeds(), imu.get_timestamp()*1e-6
        accels = tuple(accel*0.00980665 for accel in raw_accels) # in m/s2 
        w_rates = tuple(mdps/1000 for mdps in raw_ws) # in deg/s 
        dt = ts - last_ts
        speeds = tuple( (accels[i]*dt)+speeds[i] for i in range(3))
        pos = tuple( (((accels[i]*dt)+speeds[i])*dt) + pos[i] for i in range(3))
        angles = tuple(w_rates[i]*dt + angles[i] for i in range(3))

        print(f"{pos=}  |  {speeds=}  | {accels=}  | {angles=}  | {w_rates=}")
        time.sleep(0.15) 
