from ...custom_terminal import print 
from ...interfaces import Coordinates 

import numpy as np 

class PoseGraph: 
    def __init__(self, prior_pos:Coordinates, prior_yaw:float): 
        self.x = np.array([prior_pos.x, 0, prior_pos.y, 0, prior_pos.z, 0, prior_yaw, 0]) # State vector is coords and speed 
        self.last_measurement_time = 0 

    ### EXTERNAL METHODS USED BY estimator.py 
    def get_position():  
        pass 
    def get_yaw():
        pass 

    def trilateration_update(): 
        # NOTE to confirm if keeping this or just making general 
        pass 
    def ranging_update():
        pass 
    def zero_movement_update():
        pass 
    def pedometer_update():
        pass 
    def custom_odometry_update():
        pass
    
    ### INTERNAL METHODS USED FOR CALCULATION 
