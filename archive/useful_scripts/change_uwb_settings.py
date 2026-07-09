from pypozyx import *


def set_device_gain(pozyx: PozyxSerial, gain_db: float, device_id: int):
    original_gain = Data([0], "f")
    status = pozyx.getUWBGain(original_gain, device_id)
    print("Original gain:", original_gain)
    
    if status == POZYX_SUCCESS:
        pozyx.setUWBGain(gain_db, device_id)
        pozyx.saveUWBSettings(device_id)
    else:
        print("Error in setting gain for device", device_id)

def main():
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No pozyx.")
        quit()

    devices = [0x1002]
    pozyx = PozyxSerial(serial_port)

    for device_id in devices:
        set_device_gain(pozyx, 15, device_id)
        

if __name__ == "__main__":
    main()

