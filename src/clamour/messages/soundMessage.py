""""This type of message conveys information about a new state measurement.
It is intended to be passed to a ContextManagedQueue as a pickled class + dictionary tuple.
The state information passed within the message will be used to choose which sound to play next."""

from pypozyx import Coordinates


class SoundMessage:
    def __init__(self, coordinates: Coordinates):
        self.coordinates = coordinates

    @staticmethod
    def save(message):
        """"Pickles the message"""
        return message.__class__, message.__dict__

    @staticmethod
    def load(cls, attributes) -> "SoundMessage":
        """Unpickles the message"""
        obj = cls.__new__(cls)
        obj.__dict__.update(attributes)
        return obj
