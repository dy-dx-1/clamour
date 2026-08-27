""" 
This file defines many objects that serve to transport info from the tag to Clamour. 
"""
import numpy as np 

class Coordinates: 
    """
    Container for x, y, z coordinates (in cm) and associated covariance (cm^2), if available.

    ALL ELEMENTS MUST BE INTEGERS. 

    Covariance is initialized by default as None. Must be updated with update_covar() method if present.  
    """ 
    def __init__(self, x:int=0, y:int=0, z:int=0): 
        self._data = [int(x),int(y),int(z)] 
        self._covar = None 
    
    def __str__(self):
        return f"Coords X: {self.x}, Y: {self.y}, Z:{self.z}"

    @property
    def data(self)->list[int,int,int]:
        """
        Position in [x,y,z] format
        """
        return self._data

    @property
    def covar(self)->np.ndarray|None: 
        """
        3x3 covariance matrix on the position, if available. ALL ELEMENTS MUST BE INTS. 
        """
        return self._covar

    @covar.setter
    def covar(self, new_matrix:np.ndarray): 
        """
        3x3 covariance matrix on the position, if available 
        [xx, xy, xz]
        [xy, yy, yz] 
        [xz, yz, zz]
        """
        self._covar = new_matrix
    
    def update_covar(self, covariances:tuple[int,int,int,int,int,int]): 
        """
        Update the 3x3 covar matrix by passing the covariance of each distinct element in a tuple containing:
        - xx
        - yy
        - zz
        - xy
        - xz
        - yz
        """
        xx, yy, zz, xy, xz, yz = covariances
        self._covar = np.array([[int(xx), int(xy), int(xz)], 
                                [int(xy), int(yy), int(yz)], 
                                [int(xz), int(yz), int(zz)]])

    def load(self, data:list):
        """ Updates the XYZ object with new data in format [x,y,z], all ints"""
        self._data = data 

    @property
    def x(self):
        return self._data[0] 

    @x.setter
    def x(self, value):
        self._data[0] = value 

    @property
    def y(self):
        return self._data[1] 

    @y.setter
    def y(self, value):
        self._data[1] = value 

    @property
    def z(self):
        return self._data[2] 

    @z.setter
    def z(self, value):
        self._data[2] = value 

class Angles:
    """
    Container for angles as heading(yaw), roll, and pitch (in degrees).
    """

    def __init__(self, heading=0, roll=0, pitch=0):
        self._data = [heading, roll, pitch]

    def load(self, data):
        self._data = data

    def __str__(self):
        return f'Heading: {self.heading}, Roll: {self.roll}, Pitch: {self.pitch}'

    @property
    def heading(self):
        return self._data[0] 

    @heading.setter
    def heading(self, value):
        self._data[0] = value 

    @property
    def roll(self):
        return self._data[1] 

    @roll.setter
    def roll(self, value):
        self._data[1] = value 

    @property
    def pitch(self):
        return self._data[2] 

    @pitch.setter
    def pitch(self, value):
        self._data[2] = value 