from multiprocessing import Lock
from pypozyx import PozyxSerial, SingleRegister, DeviceList, POZYX_DISCOVERY_ALL_DEVICES, POZYX_DISCOVERY_TAGS_ONLY, POZYX_DISCOVERY_ANCHORS_ONLY
from pypozyx.definitions.constants import POZYX_SUCCESS
from struct import error as StructError


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
        try:
            with pozyx_lock:
                status = pozyx.getDeviceListSize(size)
        except StructError as s:
            print(str(s))
        
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
            try:
                with pozyx_lock:
                    pozyx.getDeviceIds(devices)
            except StructError as s:
                print(str(s))
        elif status != POZYX_SUCCESS:
            PozyxDiscoverer.handle_error(pozyx, pozyx_lock, "get_device_list")

        if discovery_type == POZYX_DISCOVERY_TAGS_ONLY:
            devices = [device_id for device_id in devices if not PozyxDiscoverer.is_anchor(device_id)]
        elif discovery_type == POZYX_DISCOVERY_ANCHORS_ONLY:
            devices = [device_id for device_id in devices if PozyxDiscoverer.is_anchor(device_id)]

        return devices

    @staticmethod
    def handle_error(pozyx: PozyxSerial, pozyx_lock: Lock, function_name: str) -> None:
        error_code = SingleRegister()

        try:
            with pozyx_lock:
                pozyx.getErrorCode(error_code)
                message = pozyx.getErrorMessage(error_code)
        except StructError as s:
            print(str(s))

        if error_code != 0x0:
            print("Error in", function_name, ":", message)
