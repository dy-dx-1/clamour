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
        return f"x: {round(self.x, 3)} y: {round(self.y, 3)} z: {round(self.z, 3)}"
