import gtsam as gt 

from ...custom_terminal import print 
from ...interfaces import Coordinates 
from ...interfaces import Anchors 

import numpy as np 

anchors = Anchors()
### Defining noise models 
## Units in mm, like the rest of the graph 
ANCHOR_POS_NOISE = gt.noiseModel.Diagonal.Sigmas([50, 50, 50]) # uncertainty in anchor placement
RANGING_NOISE = gt.noiseModel.Isotropic.Sigma(1, 100) # precise 1D measurement ~ 10cm 

"""
Collecting questions to check 
- What about neighboring tags? These have a unique ID, but their position will change. Should they be added every time with the same symbol? Will GTSAM be confused if their position changes? 
- When initialising the graph, are we forced to define a PriorFactorPose in addition to the 3+ anchors?  Can't the graph define it just with anchors? 
"""

class PoseGraph: 
    def __init__(self, anchors_range_data:list[tuple[int, int]], prior_yaw:float): 
        """
        Factor Graph based 3D pose estimator. 
        - anchors_range_data: [(anchor_id, range), ...] At least 3 are required to initialize the prior position 
        """
        # Internal data about the current state. Kept up to date and fed back to estimator.py 
        prior_pos = anchors.get_centroid_for(data[0] for data in anchors_range_data) 
        self.x = np.array([prior_pos.x, 0, prior_pos.y, 0, prior_pos.z, 0, prior_yaw, 0]) # State vector is coords and speed 
        self.last_measurement_time = 0 # NOTE check how EKF does it in init? init with timestamp to estimate speeds? 

        # Graph trackers 
        self._state_counter = 0   # Keeps track of how many state nodes have been added to the graph
        self.seen_anchors = set() # Keeps track of anchors we have previously seen, to avoid re-adding prior factors
        self.isam = gt.ISAM2() 

        # Initialising first pose 
        x = gt.symbol('x', self.state_counter) 
        graph = gt.NonlinearFactorGraph() 
        initial_values = gt.Values() 
        # Our initial pose takes in the prior yaw and throwaway values for pitch and roll as not tracking them right now
        # the prior pose is loosely defined a simply the centroid of the anchors. This is inaccurate and only 
        # serves to give 'ok' initial conditions for the >3 anchors to fully define the first position 
        # henceforth, the rotational noise is small compared to a very large, loose, positional noise for the prior 
        # estimator.py will initialise and subsequently call incorporate_ranging_data with >3 anchors, which will determine the position. 
        graph.add(gt.PriorFactorPose3(x, 
                                      gt.Pose3(0, 0, prior_yaw, prior_pos.x, prior_pos.y, prior_pos.z), 
                                      gt.noiseModel.Diagonal.Sigmas([1, 1, 1, 1e5, 1e5, 1e5])))
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
        # TODO i think for now to mimick EKF behavior could add inside of this function a betweenfactor that uses calculated speed to estimate a prior for this pose 
        # before adding anchor data 
        
        # Anchor data 
        for id, z in anchors_ranging_data: 
            l = gt.symbol('l', id) 
            anchor_pos = anchors.anchors_dict[id].data # List of the x, y, z coordinates in mm 
            if id not in self.seen_anchors:
                # If we have never seen this anchor, need to add a prior on it's position 
                # If we have, then no need to re-add a prior. Just directly reference it during range add
                graph.add(gt.PriorFactorPoint3(l, gt.Point3(*anchor_pos), ANCHOR_POS_NOISE))
                initial_values.insert(l, gt.Point3(*anchor_pos))
                self.seen_anchors.add(id) 
            graph.add(gt.RangeFactor3D(x, l, z, RANGING_NOISE))
            
        # Tag data 
        for pos, z in tags_ranging_data: # TODO eval format compared to anchors, same ID or not? 
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
