class PedometerMeasurement:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __gt__(self, other):
        return self.y > other.y

    def __ge__(self, other):
        return self.y >= other.y

    def __le__(self, other):
        return self.y <= other.y

    def __lt__(self, other):
        return self.y < other.y

    def __eq__(self, other):
        return self.y == other.y

    def __repr__(self):
        return "x: " + str(round(self.x, 3)) + " y: " + str(round(self.y, 3)) + " z: " + str(round(self.z, 3))
