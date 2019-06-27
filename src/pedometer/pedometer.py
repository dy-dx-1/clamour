from pypozyx import PozyxSerial, get_first_pozyx_serial_port, EulerAngles, LinearAcceleration
from time import sleep


class Pedometer:
    def __init__(self):
        self.pozyx = self.connect_pozyx()

    def run(self):
        while True:
            linear_acceleration = LinearAcceleration()
            euler_angles = EulerAngles()

            self.pozyx.getAcceleration_mg(linear_acceleration)
            self.pozyx.getEulerAngles_deg(euler_angles)

            print(f"Linear acceleration: {linear_acceleration.data}; Euler angles: {euler_angles.data}")
            sleep(0.5)

    @staticmethod
    def connect_pozyx() -> PozyxSerial:
        serial_port = get_first_pozyx_serial_port()

        if serial_port is None:
            raise Exception("No Pozyx connected. Check your USB cable or your driver.")

        return PozyxSerial(serial_port)
