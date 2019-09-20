from multiprocessing import Lock
from pypozyx import PozyxSerial, SingleRegister, DeviceList, POZYX_DISCOVERY_ALL_DEVICES, POZYX_DISCOVERY_TAGS_ONLY, POZYX_DISCOVERY_ANCHORS_ONLY
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
    def is_anchor(device_id: int) -> bool:
        return device_id < 0x500

    @staticmethod
    def get_device_list(pozyx: PozyxSerial, pozyx_lock: Lock, discovery_type: int) -> DeviceList:
        PozyxDiscoverer.discover(pozyx, pozyx_lock, POZYX_DISCOVERY_ALL_DEVICES)
        status, size = PozyxDiscoverer.get_nb_devices(pozyx, pozyx_lock)
        devices = DeviceList(list_size=size)

        if status == POZYX_SUCCESS and size > 0:
            with pozyx_lock:
                pozyx.getDeviceIds(devices)
        elif status != POZYX_SUCCESS:
            PozyxDiscoverer.handle_error(pozyx, pozyx_lock, "get_device_list")

        if discovery_type == POZYX_DISCOVERY_TAGS_ONLY:
            devices = [device for device in devices if not self.is_anchor(device)]
        else if discovery_type == POZYX_DISCOVERY_ANCHORS_ONLY:
            devices = [device for device in devices if self.is_anchor(device)]

        return devices

    @staticmethod
    def handle_error(pozyx: PozyxSerial, pozyx_lock: Lock, function_name: str) -> None:
        error_code = SingleRegister()

        with pozyx_lock:
            pozyx.getErrorCode(error_code)
            message = pozyx.getErrorMessage(error_code)

        if error_code != 0x0:
            print("Error in", function_name, ":", message)
