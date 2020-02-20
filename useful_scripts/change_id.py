#!/usr/bin/python3

import pypozyx as pyp
from pypozyx.definitions.registers import POZYX_NETWORK_ID

serial_port = pyp.get_first_pozyx_serial_port()
pozyx = pyp.PozyxSerial(serial_port)

remote_id = 0x6a71
new_id = 0x2030

pozyx.setNetworkId(new_id, remote_id)
pozyx.saveConfiguration(pyp.POZYX_FLASH_REGS, [POZYX_NETWORK_ID], remote_id)
pozyx.resetSystem(remote_id)

data = pyp.Data([0] * 2)
pozyx.getRead(POZYX_NETWORK_ID, data)
hex(data[1] * 256 + data[0])
