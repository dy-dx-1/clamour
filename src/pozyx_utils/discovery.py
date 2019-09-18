from multiprocessing import Lock
from pypozyx import PozyxSerial, SingleRegister, DeviceList
from pypozyx.definitions.constants import POZYX_SUCCESS


class PozyxDiscoverer:

    @staticmethod
    def discover(pozyx: PozyxSerial, pozyx_lock: Lock, discovery_type: int) -> None:
        with pozyx_lock:
            status = pozyx.doDiscovery(discovery_type=discovery_type)

        if status != POZYX_SUCCESS:
            PozyxDiscoverer.handle_error(pozyx, pozyx_lock, "discover")

    @staticmethod
    def get_nb_devices(pozyx: PozyxSerial, pozyx_lock: Lock) -> tuple:
        size = SingleRegister()
        with pozyx_lock:
            status = pozyx.getDeviceListSize(size)
        
        if status != POZYX_SUCCESS:
            PozyxDiscoverer.handle_error(pozyx, pozyx_lock, "get_nb_devices")

        return status, size[0]

    @staticmethod
    def get_device_list(pozyx: PozyxSerial, pozyx_lock: Lock, discovery_type: int) -> DeviceList:
        PozyxDiscoverer.discover(pozyx, pozyx_lock, discovery_type)
        status, size = PozyxDiscoverer.get_nb_devices(pozyx, pozyx_lock)
        devices = DeviceList(list_size=size)

        if status == POZYX_SUCCESS and size > 0:
            with pozyx_lock:
                pozyx.getDeviceIds(devices)
        elif status != POZYX_SUCCESS:
            PozyxDiscoverer.handle_error(pozyx, pozyx_lock, "get_device_list")

        return devices

    @staticmethod
    def handle_error(pozyx: PozyxSerial, pozyx_lock: Lock, function_name: str) -> None:
        error_code = SingleRegister()

        with pozyx_lock:
            pozyx.getErrorCode(error_code)
            message = pozyx.getErrorMessage(error_code)

        if error_code != 0x0:
            print("Error in", function_name, ":", message)
