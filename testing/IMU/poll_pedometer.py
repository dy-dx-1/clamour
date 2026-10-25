import sys 
from pathlib import Path 
from time import sleep 


# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.LSM6DSV320X_imu import LSM6DSV320X

with LSM6DSV320X(120, 2, 500, False) as imu: 
    while True: 
        step = imu.step_event() 
        if step: 
            print(f"Step detected!") 
        