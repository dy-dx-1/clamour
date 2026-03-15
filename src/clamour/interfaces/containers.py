""" 
This file defines many objects that serve to transport info from the tag to Clamour. 
Originally, all of these directly came from the pypozyx lib. 
In an effort to decouple the code from it, these are general versions that can be used in a platform agnostic manner. 
"""

class XYZ: 
    """
    Generic XYZ data structure consisting of 3 integers x, y and z.
    Is used to derive other data containers
    """
    ### NOTE: These variables are from the original class, idk if they will be useful for us yet
    byte_size = 12
    data_format = 'iii'
    physical_convert = 1 # This is used originally to derive other classes. Not sure if we need it. 

    def __init__(self, x:int=0, y:int=0, z:int=0): 
        """Initializes the XYZ object as a list"""
        self.data = [x,y,z] 
    
    def __str__(self):
        return f"X: {self.x}, Y: {self.y}, Z:{self.z}"

    def load(self, data:list):
        """ Updates the XYZ object with new data in format [x,y,z], all ints"""
        self.data = data 

    @property
    def x(self):
        return self.data[0] / self.physical_convert

    @x.setter
    def x(self, value):
        self.data[0] = value * self.physical_convert

    @property
    def y(self):
        return self.data[1] / self.physical_convert

    @y.setter
    def y(self, value):
        self.data[1] = value * self.physical_convert

    @property
    def z(self):
        return self.data[2] / self.physical_convert

    @z.setter
    def z(self, value):
        self.data[2] = value * self.physical_convert

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
        }

class Coordinates(XYZ): 
    """
    Container for x, y, z coordinates (in mm)
    This is effectively the same thing as XYZ, just with a different name that indicates they're coords
    """ 
    byte_size = 12
    data_format = 'iii'
