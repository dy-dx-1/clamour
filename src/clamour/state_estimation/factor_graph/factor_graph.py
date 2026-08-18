import gtsam as gt 

from ...custom_terminal import print 
from ...interfaces import Coordinates 

import numpy as np 

### Defining noise models 
## Units in mm, like the rest of the graph 
ANCHOR_POS_NOISE = gt.noiseModel.Diagonal.Sigmas([50, 50, 50]) # uncertainty in anchor placement
RANGING_NOISE = gt.noiseModel.Isotropic.Sigma(1, 100) # precise 1D measurement ~ 10cm 

class PoseGraph: 
    def __init__(self, prior_pos:Coordinates, prior_yaw:float): 
        self.x = np.array([prior_pos.x, 0, prior_pos.y, 0, prior_pos.z, 0, prior_yaw, 0]) # State vector is coords and speed 
        self.last_measurement_time = 0 # NOTE check how EKF does it in init? init with timestamp to estimate speeds? 

        # Graph related stuff 
        self._state_counter = 0  # Keeps track of how many state nodes have been added to the graph 
        self.isam = gt.ISAM2() 

        x = gt.symbol('x', self.state_counter) 
        graph = gt.NonlinearFactorGraph() # initial graph for prior 
        initial_values = gt.Values() 
        # TODO fill inside of args. Define if using centroid or if should trilaterate 
        graph.add(gt.PriorFactorPose3(x, gt.Pose3(), gt.noiseModel.Diagonal.Sigmas([])))
        initial_values.insert(x, gt.Pose3(prior_pos.x, prior_pos.y, prior_pos.z)) # initial guess on the position 

    ### Properties
    @property
    def state_counter(self)->int:
        self._state_counter += 1 
        return self._state_counter 

    ### EXTERNAL METHODS USED BY estimator.py 
    def get_position(self):  
        return Coordinates(self.x[0], self.x[2], self.x[4])
    def get_yaw(self):
        return self.x[6]

    def incorporate_ranging_data(self, timestamp: float, anchors_ranging_data:list[tuple[Coordinates, int]], tags_ranging_data:list[tuple[Coordinates, int]], raw_yaw:float):
        """
        Called whenever we get new ranges from anchors or tags to add to the factor graph. 
        """
        graph = gt.NonlinearFactorGraph() 
        initial_values = gt.Values() 

        x = gt.symbol('x', self.state_counter) 
        # Anchor data 
        for pos, z in anchors_ranging_data: 
            # TODO add way of mapping to anchor's ID to be able to associate to proper landmark ID 
            # need to do same for tags? or not since their position changes? 
            l = gt.symbol('l', anchor_id) 
            graph.add()
            pass 
        # Tag data 
        for pos, z in tags_ranging_data: 
            pass 

        # Updating graph and internal data 
        self.isam.update(graph, initial_values) 
        results = self.isam.calculateEstimate() 
        current_state_estimate = results.atPose3(x) 

    def zero_movement_update():
        pass 
    def pedometer_update():
        pass 
    def custom_odometry_update():
        pass
