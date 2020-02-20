#!/usr/bin/python3

import pypozyx as pyp
from pypozyx.definitions.registers import POZYX_NETWORK_ID

serial_port = pyp.get_first_pozyx_serial_port()
pozyx = pyp.PozyxSerial(serial_port)

pozyx.clearDevices()
size = pyp.SingleRegister()
pozyx.doDiscovery(discovery_type=pyp.POZYX_DISCOVERY_ANCHORS_ONLY)
pozyx.getDeviceListSize(size)

devices = pyp.DeviceList(list_size=size[0])

if len(devices) > 0:
    pozyx.getDeviceIds(devices)
    pozyx.printDeviceList()
else:
    print("No devices found.")
