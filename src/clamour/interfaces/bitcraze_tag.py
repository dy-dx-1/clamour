from .tag import Tag
from .containers import Coordinates, Angles

from typing import Literal
from pathlib import Path
import os 
import fcntl
import time 
import serial

def find_first_bc_port() -> str | None:
    """
    Looks through ports and checks if one is for a Bitcraze device 
    Uses by-id to give robust paths independent of ttyACMX indexing 
    Returns the path to the port or None 
    """
    for path in Path("/dev/serial/by-id").iterdir():
        if "Bitcraze" in path.name:
            return str(path) 
    return None 

def tty_to_usb_device_path(tty_or_symlink_path:str) -> str:
    """
    Convert a tty device or symlink into the underlying USB device path. 
    This is used to reset the device through usb. 

    Examples:
        /dev/ttyACM0
        /dev/serial/by-id/usb-Bitcraze_...
    ->
        /dev/bus/usb/001/004
    """

    print(f"Input device: {tty_or_symlink_path}")
    # Resolve the actual tty device if this is a symlink
    tty_realpath = Path(tty_or_symlink_path).resolve()

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
    return None 

class BitcrazeTag(Tag):
    """
    Defines the UWB tag interface for a Bitcraze Loco Positioning Node/Tag. 
    Methods are adapted from abstractclass Tag. 
    Refer to Tag class for typehints and docstrings, except when overwritten for clarity. 
    """
    def __init__(self):
        self.serial_port = find_first_bc_port() 
        self.usb_path = tty_to_usb_device_path(self.serial_port) 
        if self.serial_port is None:
            raise Exception("No Bitcraze connected. Check your USB cable or your driver.")
        if self.usb_path is None: 
            raise Exception("Could not resolve Bitcraze serial port (tty or symlink to it) into a USB path.")

        self.serial_con = serial.Serial(port=self.serial_port, baudrate=9600, timeout=1) 

        self.cpu_id, self.tag_id, mode_desc = self.get_tag_info() 
        if mode_desc != "twr_tag": # NOTE: In the future can add automatic mode change if useful enough 
            self.serial_con.close() 
            raise Exception(f"Device is not in TWR Tag mode, but instead in : {mode_desc}. Change this.")
        if BitcrazeTag.is_anchor(self.tag_id): 
            raise Exception(f"Device ID {self.tag_id} is unexpected for a tag. Change this to be coherent with BitcrazeTag.is_anchor()")
        
        print(f"[OK] SUCCESSFULLY CONNECTED TO BC DEVICE IN TWR TAG MODE. PORT: {self.serial_port}, ID {self.tag_id}") 

    def get_tag_info(self)->tuple[str, int, str]:   
        """
        Closes and opens the connection to get the startup info of the tag. 
        """
        max_attempts = 10 # Multiple attempts in case info can't be properly read at first 
        for _ in range(max_attempts):
            if self.serial_con.is_open: 
                self.serial_con.close()
                time.sleep(0.1) 

            self.serial_con.open() 
 
            cpu_id = None 
            uwb_id = None 
            mode_info = None 
            
            try:
                for _ in range(50):
                    line = self.serial_con.readline().decode(errors="ignore").strip()
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
                self.serial_con.close()
                raise(Exception(f"CONNECTION CLOSED. Error occurred during get_tag_info: {e}"))
            
            if cpu_id and uwb_id and mode_info: 
                return cpu_id, int(uwb_id, base=16), mode_info 
        
        raise RuntimeError(f"Failed to retrieve tag info after {max_attempts} attempts.")

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------
    @staticmethod
    def is_anchor(device_id: int) -> bool:
        # NOTE: for BC as of 2026-05-26, I am defining anchors as devices with IDs <=10 
        # Tags must have any other IDs. BC supports 0-255 in hexadecimal 
        return device_id<=10 

    def addDevice(self, device:object) -> None:
        """
        Adds an anchor or tag to the tag's internal device list.

        Args:
            device: Any appropriate object that can be handled by the tag interface in it's internal device list.
        """

    def clearDevices(self) -> None:
        """
        Clears the tag's internal device list.
        """

    def resetSystem(self) -> None:
        """
        Resets the tag. 
        """
        if self.serial_con.is_open: 
            self.serial_con.close() 

        USBDEVFS_RESET = ord('U') << 8 | 20
        print(f"[INFO] RESETTING TAG #{self.tag_id} at port: {self.serial_port}")
        fd = os.open(self.usb_path, os.O_WRONLY)
        try:
            fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        finally:
            os.close(fd)
        time.sleep(3) 

        # Since we are using a symlink in find_first_lps_port, the port is still valid
        # We only need to reopen the serial connection 
        self.serial_con = serial.Serial(port=self.serial_port, baudrate=9600, timeout=1)
        print(f"[INFO] RESET COMPLETED") 

    def printCurrentError(self, function_name:str) -> bool:
        """
        Checks if the tag experienced an error and retrieves it. 
        If there was an error, prints it to the terminal & the function name where it happened.
        
        Args:
            function_name: String indicating the function name where this was checked
        Returns: 
            Bool on whether there was really an error or not 
        """
    
    def get_device_list(self, discovery_type:Literal["all", "anchor", "tag"]) -> list[int]:
        """
        Gets the list of IDs of devices seen by the tag. 

        Args: 
            discovery_type: String specifying what type of device to return
        
        Returns:
            List of corresponding device IDs (ints) 
        """

    ### -------------------------------------------- INTER-TAG COMMUNICATION --------------------------------------------
    def sendData(self, destination:int, payload:bytes) -> bool: 
        """
        Transmits data from the tag to a destination. 

        Args:
            destination: destination id (int)
            payload: data to send (bytes)

        Returns:
            Bool on whether it succeeded
        """
    
    def receiveData(self) -> tuple[int, bytes]:
        """
        Reads data received by the tag. 

        Returns: 
            tuple[sender_id (int), data (bytes)]
        """

    ### -------------------------------------------- LOCALIZATION --------------------------------------------
    def configureAnchorSelection(self, number_of_anchors:int) -> None:
        """
        Configures how many and which anchors are used for positioning the tag.
        Should be called with the total number of anchors that are currently available. We can then use them all or a more efficient subset. 
        Only used in task.py: it's updated if there's more than 3 available anchors

        Args:
            number_of_anchors: int specifying how many anchors are available for positioning
        """
    
    def setCoordinates(self, coord_list:list[int]) -> None:
        """
        Manually sets the position of the tag. 

        Args:
            coord_list: List of ints [x, y, z] representing the position of the tag
        """

    def getCoordinates(self) -> Coordinates | None: 
        """
        Gets the coordinates of the device. 
        Does not trigger positioning, only retrieves last known coordinates. 

        Returns:
            Coordinates object of the last known position 
        """

    def getOrientation(self) -> Angles | None: 
        """ 
        Gets the current orientation of the tag in degrees. 
        
        Returns:
            Angles object of the current orientation (heading, roll, pitch)
        """

    def doPositioning(self) -> Coordinates | None:
        """
        Positions the tag in space with UWB ranging. 
        This function computes and stores the position in the tag's memory. 

        Returns:
            Coordinates object with the position or None
        """

    def doRanging(self, target_id:int) -> Coordinates | None: 
        """
        Calculates a UWB range measurement between the tag and another device. 
        
        Args:
            target_id: ID of the target device (int) 
        
        Returns:
            Coordinates object with the position or None
        """ 