import numpy as np
from time import sleep
import sys 
from pathlib import Path 
from time import sleep 


# Add the project's src directory to sys.path when run as a standalone script.
src_dir = str(Path(__file__).resolve().parents[2] / "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from clamour.interfaces.LSM6DSV320X_imu import LSM6DSV320X
with LSM6DSV320X(120, 2, 500, False) as imu:
    imu.discard_FIFO()  # right after configure, like _initialize_imu
    gyro = []
    for _ in range(8):  # 8 x 0.25s = 2s
        sleep(0.25)
        gyro += [g for _, _, g in imu.read_FIFO()]
    gyro = np.array(gyro)
    print("first 10 samples (mdps):\n", np.round(gyro[:10]))
    print("std all:", np.round(gyro.std(axis=0, ddof=1), 1))
    print("std skipping first 0.25s:", np.round(gyro[30:].std(axis=0, ddof=1), 1))
    print("sample index of largest deviation per axis:", np.abs(gyro - np.median(gyro, axis=0)).argmax(axis=0))