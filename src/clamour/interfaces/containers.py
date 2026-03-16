""" 
This file defines many objects that serve to transport info from the tag to Clamour. 
Originally, all of these directly came from the pypozyx lib. 
In an effort to decouple the code from it, these are general versions that can be used in a platform agnostic manner. 
"""

class XYZ: 
    """
    Generic XYZ data structure consisting of 3 integers x, y and z.
    Is used to derive other data containers
    NOTE: The original one inherits from ByteStructure
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

class DeviceCoordinates: 
    """
    NOTE: The original one inherits from ByteStructure
    Container for both reading and writing device coordinates from and to the tag.

    The keyword arguments are at once its properties.

    Kwargs:
        network_id: Network ID of the device
        flag: Type of the device. Tag or anchor.
        pos: Coordinates of the device. Coordinates().
    """
    byte_size = 15
    data_format = 'HBiii'

    def __init__(self, network_id=0, flag=0, pos=Coordinates()):
        """
        Initializes the DeviceCoordinates object.

        Kwargs:
            network_id: Network ID of the device
            flag: Type of the device. Tag or anchor.
            pos: Coordinates of the device. Coordinates().
        """
        self.data = [network_id, flag, int(pos.x), int(pos.y), int(pos.z)]

    def load(self, data):
        self.data = data

    def __str__(self):
        return "ID: 0x{:04X}, flag: {}, ".format(self.network_id, self.flag) + str(self.pos)

    @property
    def network_id(self):
        return self.data[0]

    @network_id.setter
    def network_id(self, value):
        self.data[0] = value

    @property
    def flag(self):
        return self.data[1]

    @flag.setter
    def flag(self, value):
        self.data[1] = value

    @property
    def pos(self):
        return Coordinates(self.data[2], self.data[3], self.data[4])

    @pos.setter
    def pos(self, value):
        self.data[2] = value.x
        self.data[3] = value.y
        self.data[4] = value.z