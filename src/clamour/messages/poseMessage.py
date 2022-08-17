class PoseMessage:
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    @staticmethod
    def save(message):
        """"Pickles the message"""
        return message.__class__, message.__dict__

    @staticmethod
    def load(cls, attributes) -> "UpdateMessage":
        """Unpickles the message"""
        obj = cls.__new__(cls)
        obj.__dict__.update(attributes)
        return obj
