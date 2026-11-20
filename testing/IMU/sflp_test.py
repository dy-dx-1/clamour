import sys 
from pathlib import Path 
from time import sleep 


# Add the project's src directory to sys.path when run as a standalone script.
src_dir = str(Path(__file__).resolve().parents[2] / "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from clamour.interfaces.LSM6DSV320X_imu import LSM6DSV320X


import numpy as np
with LSM6DSV320X(120, 2, 500, False) as imu:
    sleep(2)
    for _ in range(5):
        input("Hold the tag still at a new tilt, then press Enter")
        _, pitch_x, roll_y = np.deg2rad(imu.get_attitude())
        accel = imu.get_x_y_z_accel()
        expected = np.array([-np.sin(roll_y), np.sin(pitch_x)*np.cos(roll_y), np.cos(pitch_x)*np.cos(roll_y)])
        print("accel dir:", np.round(accel/np.linalg.norm(accel), 3), "| expected:", np.round(expected, 3))

        