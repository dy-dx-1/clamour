import sys 
from pathlib import Path 
from time import sleep 


# Add the project's src directory to sys.path when run as a standalone script.
src_dir = str(Path(__file__).resolve().parents[2] / "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from clamour.interfaces.LSM6DSV320X_imu import LSM6DSV320X

"""
with LSM6DSV320X(120, 2, 500, False) as imu: 
    while True: 
        step = imu.step_event() 
        if step: 
            print(f"Step detected!") 
"""

with LSM6DSV320X(120, 2, 500, False) as imu: 
    while True: 
        steps = imu.step_count() 
        print(f"step count: {steps}") 
        sleep(1)  

        