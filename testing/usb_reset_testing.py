import os
import fcntl
import time
from pathlib import Path

USBDEVFS_RESET = ord('U') << 8 | 20

def tty_to_usb_device_path(tty_device):
    """
    Convert a tty device or symlink into the underlying USB device path.

    Examples:
        /dev/ttyACM0
        /dev/serial/by-id/usb-Bitcraze_...
    ->
        /dev/bus/usb/001/004
    """

    print(f"Input device: {tty_device}")
    # Resolve the actual tty device if this is a symlink
    tty_realpath = Path(tty_device).resolve()

    print(f"Resolved tty device: {tty_realpath}")
    tty_name = tty_realpath.name

    # Build sysfs path from resolved tty name
    sysfs_tty_path = Path(f"/sys/class/tty/{tty_name}")
    # Follow sysfs symlink to actual device
    device_path = sysfs_tty_path.resolve()

    # Walk upward until we find USB metadata
    current = device_path
    while current != current.parent:
        busnum = current / "busnum"
        devnum = current / "devnum"
        if busnum.exists() and devnum.exists():
            bus = int(busnum.read_text().strip())
            dev = int(devnum.read_text().strip())
            usb_path = f"/dev/bus/usb/{bus:03d}/{dev:03d}"
            print(f"USB device path: {usb_path}")
            return usb_path
        current = current.parent
    raise RuntimeError(f"USB device not found for {tty_device}")

def reset_usb(device_path):

    print(f"Resetting {device_path}")

    fd = os.open(device_path, os.O_WRONLY)

    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)

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

dev = tty_to_usb_device_path(tty_device=find_first_lps_port())

print("Found device")
print(dev) 

reset_usb(dev)

print("Reset sent")
#
time.sleep(3)
#
print("Done")