""" 
This file defines many objects that serve to transport info from the tag to Clamour. 
"""

class Coordinates: 
    """
    Container for x, y, z coordinates (in mm)
    """ 
    def __init__(self, x:int=0, y:int=0, z:int=0): 
        self.data = [x,y,z] 
    
    def __str__(self):
        return f"X: {self.x}, Y: {self.y}, Z:{self.z}"

    def load(self, data:list):
        """ Updates the XYZ object with new data in format [x,y,z], all ints"""
        self.data = data 

    @property
    def x(self):
        return self.data[0] 

    @x.setter
    def x(self, value):
        self.data[0] = value 

    @property
    def y(self):
        return self.data[1] 

    @y.setter
    def y(self, value):
        self.data[1] = value 

    @property
    def z(self):
        return self.data[2] 

    @z.setter
    def z(self, value):
        self.data[2] = value 

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
        }

class Angles:
    """
    Container for angles as heading(yaw), roll, and pitch (in degrees).
    """

    def __init__(self, heading=0, roll=0, pitch=0):
        self.data = [heading, roll, pitch]

    def load(self, data):
        self.data = data

    def __str__(self):
        return f'Heading: {self.heading}, Roll: {self.roll}, Pitch: {self.pitch}'

    @property
    def heading(self):
        return self.data[0] 

    @heading.setter
    def heading(self, value):
        self.data[0] = value 

    @property
    def roll(self):
        return self.data[1] 

    @roll.setter
    def roll(self, value):
        self.data[1] = value 

    @property
    def pitch(self):
        return self.data[2] 

    @pitch.setter
    def pitch(self, value):
        self.data[2] = value 