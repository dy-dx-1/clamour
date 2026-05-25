"""
2026-05-25
Using this script to figure out how to interface through serial with bitcraze nodes. 
"""
import serial 
from pathlib import Path

def find_first_lps_port():
    """
    Looks through ports and checks if one is for a Bitcraze Loco Positioning Node. 
    Uses by-id to give robust paths independent of ttyACMX indexing 
    Returns the path to the port or None 
    """
    for path in Path("/dev/serial/by-id").iterdir():
        if "Bitcraze" in path.name:
            return str(path) 
    return None 

def get_board_info(port):   
    print(f"Connecting to {port}")
    ser = serial.Serial(port=port, baudrate=9600, timeout=1)  
    print("\nReading startup output...\n")
    try:
        for _ in range(50):
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            print(line)

            if "Press 'h' for help" in line:
                break

        ser.write(b"h") 
        for _ in range(50): 
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            print(line)

    finally:
        ser.close()
