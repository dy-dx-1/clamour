import time
from pathlib import Path
import subprocess
import os

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


def get_usb_hub_and_port(device_path="/dev/ttyACM0"):
    """
    Finds the USB hub and port for a given device symlink.
    """
    try:
        # Resolve symlink (e.g., /dev/ttyACM0 -> /sys/class/tty/ttyACM0)
        device_sys_path = Path(f"/sys/class/tty/{os.path.basename(device_path)}").resolve()
        
        # The sysfs path looks like: /sys/devices/platform/scb/fd500000.pcie/pci0000:00/.../usb1/1-1/1-1.2/...
        # We need to find the part of the path that contains the USB bus/port identifier (e.g., "1-1.2")
        path_parts = device_sys_path.parts
        
        usb_bound_dir = None
        for part in path_parts:
            # USB device directories usually start with the bus number followed by a dash (e.g., '1-1' or '1-1.2')
            if '-' in part and not part.startswith('pci') and not part.startswith('usb'):
                usb_bound_dir = part
        
        if usb_bound_dir:
            # Example: "1-1.2" -> Hub is '1', Port string is '1.2'
            # If it's just "1-2" -> Hub is '1', Port is '2'
            hub = usb_bound_dir.split('-')[0]
            port = usb_bound_dir.split('-')[1]
            
            print(f"[+] Found device mapping: {device_path} -> Hub {hub}, Port {port} (ID: {usb_bound_dir})")
            return usb_bound_dir
        else:
            print(f"[-] Could not parse USB topology from path: {device_sys_path}")
            return None, None
            
    except FileNotFoundError:
        print(f"[-] Device {device_path} not found. Is it plugged in?")
        return None, None

def unprivileged_usb_reset(usb_device_id):
    """
    Resets a USB device by unbinding and rebinding its driver.
    Requires the udev rule to be set up beforehand.
    
    :param usb_device_id: The USB ID string (e.g., '1-1.2')
    """
    unbind_path = Path("/sys/bus/usb/drivers/usb/unbind")
    bind_path = Path("/sys/bus/usb/drivers/usb/bind")
    
    # Quick sanity check to see if the device folder exists
    device_folder = Path(f"/sys/bus/usb/devices/{usb_device_id}")
    if not device_folder.exists():
        print(f"[-] USB Device {usb_device_id} is not currently connected.")
        return False

    try:
        # 1. Unbind the driver (Simulate Unplug)
        print(f"[*] Unbinding device {usb_device_id}...")
        unbind_path.write_text(usb_device_id)
        
        # Wait for the STM32 on the Loco node to settle
        time.sleep(2.0) 
        
        # 2. Bind the driver (Simulate Replug)
        print(f"[*] Re-binding device {usb_device_id}...")
        bind_path.write_text(usb_device_id)
        
        print("[+] Device reset successfully from unprivileged script!")
        return True
        
    except PermissionError:
        print("[-] Permission Denied! Ensure you added the udev rule and your user is in the 'dialout' group.")
        return False
    except Exception as e:
        print(f"[-] Unexpected error during USB reset: {e}")
        return False

if __name__ == "__main__":
    # Replace with your node's specific USB port ID string" 
    add = get_usb_hub_and_port("/dev/ttyACM0")

    unprivileged_usb_reset(add.split(':')[0]) 