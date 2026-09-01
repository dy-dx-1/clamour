import smbus2

def unsigned_to_signed(value:int)->int: 
    """
    Converts an unsigned 2-byte int to a signed int, following two's complement 
    """ 
    return value - 0x10000 if value & 0x8000 else value


class LSM6DSV320X: 
    def __init__(self, SDO_state:bool, i2c_bus:int=1): 
        """
        I2C interface with the LSM6DSV320X IMU. 
        **ONLY USE WITH A CONTEXT MANAGER**. 

        Currently only supporting: 
        - High performance mode 

        ARGS: 
        - SDO_state: True for 'HIGH' and False for 'LOW', defines the Target Address (TAD)
        - i2c_bus: I2C bus, 1 by default 
        """
        self.TAD = 0x6A if not SDO_state else 0x6B 
        self.bus = smbus2.SMBus(i2c_bus)
        if not self.validate_connection(): 
            raise RuntimeError("Could not connect to the LSM6DSV320X IMU. Check wiring.")
        else:
            self.configure() 
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

    # TODO not sure I need these tbh 
    def read_reg_byte(self, reg_address:int)->int: 
        """Helper to read a single byte from a register address without having to rewrite the target address every time"""
        return self.bus.read_byte_data(self.TAD, reg_address)

    def write_reg_byte(self, reg_address:int, byte:int): 
        """Helper to write a single byte to a register address without having to rewrite the target address every time"""
        self.bus.write_byte_data(self.TAD, reg_address, byte)


    def validate_connection(self): 
        """
        Reads the WHO_AM_I register (0x0F) and checks it's value to confirm if a connection is properly established. 
        """
        try: 
            check = (self.bus.read_byte_data(self.TAD, 0x0F) == 0x73)
        except: 
            check = False 
        return check 

    def configure(self): 
        """
        Configures the settings of the IMU and the internal dependent conversion factors.  
        
        **Currently only supporting**:
        - High performance mode accel/gyro
        - 120Hz ODR
        - 500dps gyro bandwidth, filtered for 120Hz 
        - Accelerometer +-2g 
        
        Affects the following registers: 
        - CTRL1 (0x10) 
        - CTRL2 (0x11) 
        - CTRL6 (0x15)
        - CTRL8 (0x17) 
        """
        ### Some configs need to be done with the accelerometer and gyro in power down mode, so we first turn them off 
        self.write_reg_byte(0x10, 0x00)
        self.write_reg_byte(0x11, 0x00) 
        ### CTRL6 (0x15) Gyro bandwidth 
        lpf1_bw = 0b0000 # NOTE bandwidth selection currently hardcoded at 120Hz 
        fs_g = 0b1010    # NOTE Currently hardcoded at 500 dps, see p.69 of manual for others 
        self.LSB_TO_MDPS = 17.5 # CONVERSION FACTOR FOR 500DPS p.12
        self.write_reg_byte(0x15, lpf1_bw<<4|fs_g)
        ### CTRL8 (0x17) Accelerometer scale 
        # not touching HP_LPF2_XL_BW_2 
        self.LSB_TO_MG = 0.061 # CONVERSION FACTOR FOR +-2g p.12
        fs_xl = 0b00 # NOTE currently hardcoded +-2g (see p.71 to change) 
        self.write_reg_byte(0x17, fs_xl) 
        ### Accelerometer control reg 1 - CTRL1 - 0x10 
        # AND 
        ### Gyroscope control reg 2 - CTRL2 - 0x11 
        # The 4 MSBs will all be 0 as long as we only support high-perf mode
        # NOTE For now hardcoding 120Hz, see p.65 of manual if want other ones 
        self.write_reg_byte(0x10, 0b00000110)
        self.write_reg_byte(0x11, 0b00000110)

        # TODO ADD SECTION THAT DEFINES UNIT CONVERSION CONSTANTS BASED ON CONFIG 
        # SEE p.12 

        print(f"CONFIG COMPLETE, TEMP READING: {self.get_temp()}") 

    def get_temp(self): 
        """
        Reads the current temperature according to the 0x20 and 0x21 registers. 
        Result in degrees C.  
        """
        # The info is 2 bytes. Each stored in 0x20 and 0x21 respectively, with 0x20 being the lower one. 
        # read_word_data reads 2 bytes and treats the first one as the lower one, so no further rearranging needed. 
        raw = self.bus.read_word_data(self.TAD, 0x20) 
        return unsigned_to_signed(raw) / 256 + 25 # Units based on p.16 of user manual 

    def get_pitch_roll_yaw_speeds(self)->tuple[int,int,int]: 
        """
        Gets the raw angular rate (in mdps) for the:
        - X (pitch) axis from the 0x22 and 0x23 registers. 
        - Y (roll)  axis from the 0x24 and 0x25 registers. 
        - Z (yaw)   axis from the 0x26 and 0x27 registers. 

        **The conversion units used depend on the selected gyro dps bandwidth.** 
        """
        pitch = unsigned_to_signed(self.bus.read_word_data(self.TAD, 0x22)) 
        roll  = unsigned_to_signed(self.bus.read_word_data(self.TAD, 0x24)) 
        yaw   = unsigned_to_signed(self.bus.read_word_data(self.TAD, 0x26)) 
        return pitch*self.LSB_TO_MDPS, roll*self.LSB_TO_MDPS, yaw*self.LSB_TO_MDPS

    def get_x_y_z_accel(self)->tuple[int,int,int]:
        """
        Gets the raw linear acceleration (in mg) for the:
        - X axis from the 0x28 and 0x29 registers. 
        - Y axis from the 0x2A and 0x2B registers. 
        - Z axis from the 0x2C and 0x2D registers. 

        **The conversion units used depend on the configured accelerometer scale.**         
        """
        x = unsigned_to_signed(self.bus.read_word_data(self.TAD, 0x28))
        y = unsigned_to_signed(self.bus.read_word_data(self.TAD, 0x2A))
        z = unsigned_to_signed(self.bus.read_word_data(self.TAD, 0x2C))
        return x*self.LSB_TO_MG, y*self.LSB_TO_MG, z*self.LSB_TO_MG

with LSM6DSV320X(False) as imu: 
    ## ANGULAR SPEEDS 
    print(imu.get_pitch_roll_yaw_speeds())
    ## Accel 
    print(imu.get_x_y_z_accel())