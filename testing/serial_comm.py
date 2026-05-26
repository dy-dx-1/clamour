"""
2026-05-25
Using this script to figure out how to interface through serial with bitcraze nodes. 
"""
import serial 
from pathlib import Path
import time 

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

def get_tag_info(ser)->tuple[str, str, str]:   
    """
    Closes and opens the connection to get the startup info of the tag 
    """
    max_attempts = 10
    for _ in range(max_attempts):
        if ser.is_open: 
            ser.close()
            time.sleep(0.1) 

        ser.open() 

        # NOTE: test this, doesn't seem to be printing properly in tag mode because of the 'interrogating anchor X' 
        cpu_id = None 
        uwb_id = None 
        mode_info = None 
        
        try:
            for _ in range(50):
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                if "CPU-ID" in line: 
                    cpu_id = line.split(" ")[2]
                elif "Address" in line: 
                    uwb_id = line.split(" ")[3]
                elif "Mode is" in line: 
                    mode_info = "_".join(line.split(" ")[3:]).lower().strip() 
                elif "Press 'h' for help" in line:
                    break
        except Exception as e:
            ser.close()
            raise(Exception(f"CONNECTION CLOSED. Error occurred during get_tag_info: ", e))
        
        if cpu_id and uwb_id and mode_info: 
            return cpu_id, int(uwb_id, base=16), mode_info 
        print("loop failed") 
    raise RuntimeError(f"Failed to retrieve tag info after {max_attempts} attempts.")

if __name__ == "__main__": 
    ser = serial.Serial(port=find_first_lps_port(), baudrate=9600, timeout=1)  
    print("opened conn, waiting") 
    print(get_tag_info(ser))
    ser.close() 
    