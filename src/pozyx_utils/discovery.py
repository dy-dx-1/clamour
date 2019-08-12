from multiprocessing import Lock
from pypozyx import PozyxSerial, SingleRegister, DeviceList
from pypozyx.definitions.constants import POZYX_SUCCESS


class PozyxDiscoverer:

    @staticmethod
    def discover(pozyx: PozyxSerial, pozyx_lock: Lock, discovery_type: int) -> None:
        with pozyx_lock:
            pozyx.doDiscovery(discovery_type=discovery_type)

    @staticmethod
    def get_nb_devices(pozyx: PozyxSerial, pozyx_lock: Lock) -> tuple:
        size = SingleRegister()
        with pozyx_lock:
            status = pozyx.getDeviceListSize(size)

        return status, size[0]

    @staticmethod
    def get_device_list(pozyx: PozyxSerial, pozyx_lock: Lock) -> DeviceList:
        status, size = PozyxDiscoverer.get_nb_devices(pozyx, pozyx_lock)
        devices = DeviceList(list_size=size)

        if status == POZYX_SUCCESS and size > 0:
            with pozyx_lock:
                pozyx.getDeviceIds(devices)

        return devices
