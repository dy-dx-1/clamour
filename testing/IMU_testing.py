import smbus2

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

    def read_reg(self, reg_address:int)->int: 
        """Helper to read a single byte from a register address without having to rewrite the target address every time"""
        return self.bus.read_byte_data(self.TAD, reg_address)

    def write_reg(self, reg_address:int, byte:int): 
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
        Configures the settings of the IMU. 
        
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
        self.write_reg(0x10, 0x00)
        self.write_reg(0x11, 0x00) 
        ### CTRL6 (0x15) Gyro bandwidth 
        lpf1_bw = 0b0000 # NOTE bandwidth selection currently hardcoded at 120Hz 
        fs_g = 0b1010    # NOTE Currently hardcoded at 500 dps, see p.69 of manual for others 
        self.write_reg(0x15, lpf1_bw<<4|fs_g)
        ### CTRL8 (0x17) Accelerometer scale 
        # not touching HP_LPF2_XL_BW_2 
        fs_xl = 0b00 # NOTE currently hardcoded +-2g (see p.71 to change) 
        self.write_reg(0x17, fs_xl) 
        ### Accelerometer control reg 1 - CTRL1 - 0x10 
        # AND 
        ### Gyroscope control reg 2 - CTRL2 - 0x11 
        # The 4 MSBs will all be 0 as long as we only support high-perf mode
        # NOTE For now hardcoding 120Hz, see p.65 of manual if want other ones 
        self.write_reg(0x10, 0b00000110)
        self.write_reg(0x11, 0b00000110)

        # TODO ADD SECTION THAT DEFINES UNIT CONVERSION CONSTANTS BASED ON CONFIG 
        # SEE p.12 

        print("CONFIG COMPLETE") 
        

        

with LSM6DSV320X(False) as imu: 
    ## Reading temperature OUT_TEMP_L OUT_TEMP_H 
    raw = imu.read_reg(0x21)<<8|imu.read_reg(0x20) 
    # Converting unsigned int to signed 
    raw = raw - 0x10000 if raw & 0x8000 else raw
    # Temp unit is LSB/C on a scale of 256. 0 LSB at 25C 
    temperature_c = raw / 256 + 25
    print(f"{temperature_c=}")

    ## PITCH angular speed OUTX_L_G OUTX_H_G 
    raw = imu.read_reg(0x23)<<8|imu.read_reg(0x22) 
    raw = raw - 0x10000 if raw & 0x8000 else raw
    # NOTE TODO at specifically 500dps, we have 17.5mdps/LSB. ADD config layer that changes internal conversion based on p.12 
    pitch_v = raw*17.5 
    print(f"pitch speed: {pitch_v}mdps")
    # TODO when picking back up, simplify into class functions 

