""" 
This file defines many objects that serve to transport info from the tag to Clamour. 
Originally, all of these directly came from the pypozyx lib. 
In an effort to decouple the code from it, these are general versions that can be used in a platform agnostic manner. 
"""

class Coordinates: 
    """
    Container for x, y, z coordinates (in mm)
    This is based on the Coordinates object from pypozyx that inherits from XYZ who inherits from ByteStructure
    It used to have inherited attributes as well as byte_size and data_format 
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
    Container for euler angles as heading(yaw), roll, and pitch (in degrees).
    This is based on the EulerAngles object from pypozyx, and used inherit from ByteStructure and have the attributes physical_convet, byte_size and data_format
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
        # NOTE: careful when modifying the object, this property (and im sure others) are used in the code, make sure object types coherent
        return Coordinates(self.data[2], self.data[3], self.data[4])

    @pos.setter
    def pos(self, value):
        self.data[2] = value.x
        self.data[3] = value.y
        self.data[4] = value.z