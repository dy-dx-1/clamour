from pypozyx import PozyxSerial


class MockPozyx(PozyxSerial):
    def __init__(self):
        print("This is a fake PozyxSerial for testing purposes.")
