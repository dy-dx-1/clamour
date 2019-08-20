import pypozyx as pyp
from pypozyx.definitions.registers import POZYX_NETWORK_ID

serial_port = pyp.get_first_pozyx_serial_port()
pozyx = pyp.PozyxSerial(serial_port)

new_id=0x1104
pozyx.setNetworkId(new_id, None)
pozyx.saveConfiguration(pyp.POZYX_FLASH_REGS, [POZYX_NETWORK_ID], None)
pozyx.resetSystem(None)

data = pyp.Data([0] * 2)
pozyx.getRead(POZYX_NETWORK_ID, data)
hex(data[1] * 256 + data[0])
