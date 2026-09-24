"""
Uses averages of data in one position to get rough bias values.
Just using this 24sept to check some problems in straight line test
"""

from IMU_testing import LSM6DSV320X
import numpy as np

with LSM6DSV320X(120, 2, 500, False) as imu: 
    accels = []
    gyros = [] 
    try: 
        while True: 
            accel, gyro = imu.get_x_y_z_accel(apply_bias=False), imu.get_pitch_roll_yaw_speeds(False) 
            accels.append(accel)
            gyros.append(gyro) 
    except KeyboardInterrupt: 
        pass 
    avg_a = np.mean(accels, axis=0)
    avg_g = np.mean(gyros, axis=0) 
    print(avg_a, "  |  ", avg_g)
