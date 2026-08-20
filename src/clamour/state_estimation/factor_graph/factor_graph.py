import gtsam as gt 
import numpy as np 

from ...custom_terminal import print 
from ...interfaces import Coordinates 
from ...interfaces import Anchors 

anchors = Anchors()
### Defining noise models 
## Units in mm, like the rest of the graph 
ANCHOR_POS_NOISE = gt.noiseModel.Diagonal.Sigmas([50, 50, 50]) # uncertainty in anchor placement
RANGING_NOISE = gt.noiseModel.Isotropic.Sigma(1, 100) # precise 1D measurement ~ 10cm 
ODOMETRY_NOISE = gt.noiseModel.Diagonal.Sigmas([0.05, 0.05, 0.05, 500, 500, 500]) # currently using constant velocity so very loose 

class PoseGraph: 
    def __init__(self, anchors_range_data:list[tuple[int, int]], prior_yaw:float, timestamp:float): 
        """
        Factor Graph based 3D pose estimator. 
        - anchors_range_data: [(anchor_id, range), ...] At least 3 are required to initialize the prior position 
        """
        # Internal data about the current state. Kept up to date and fed back to estimator.py  
        # State vector is [x,xdot,y,ydot,z,zdot,theta,thetadot]. Only here for clarity, as the prior will overwrite these 0s. 
        self.x = np.array([0, 0, 0, 0, 0, 0, 0, 0]) 
        self.last_measurement_time = timestamp 
        self.dt = None 
        
        # Graph trackers 
        self._state_counter = 0   # Keeps track of how many state nodes have been added to the graph
        self.seen_anchors = set() # Keeps track of anchors we have previously seen, to avoid re-adding prior factors
        self.isam = gt.ISAM2() 

        # Creating prior factor to lock in rotation  
        self.insert_prior_rotation_lock() 

    ### Properties
    @property
    def state_counter(self)->int:
        self._state_counter += 1 
        return self._state_counter 

    ### INTERNAL COMPUTATION METHODS 
    def insert_prior_rotation_lock(self, yaw_prior:float, anchors_range_data:list[tuple[int, int]]): 
        """
        Only to be used in __init__! Updates the graph and internal state with a simple prior to constrain rotation. 
        As of 2026-08-20, this function is only here while we don't have an IMU. 
        No way to measure rotation, so we don't track/update it. However, to avoid an 
        underterminate system in the graph, we need to initialize it to a known value.
        Therefore, we insert a prior with very loose covariance on a rough position, but tight on the rotation. 
        The position component is simply the centroid of the anchors. This is inaccurate and only serves to put a rough guess that will be corrected by the anchors. 
        estimator.py will then call a incorporate_ranging_data with >=3 anchors, which will correct and lock the position appropriately. 
        """
        prior_pos = anchors.get_centroid_for(data[0] for data in anchors_range_data)
        # NOTE check if way to avoid locking in position even loosely before anchors? 
        x0 = gt.symbol('x', self.state_counter) 
        graph = gt.NonlinearFactorGraph() 
        initial_values = gt.Values() 
        graph.add(gt.PriorFactorPose3(x0, 
                                      gt.Pose3(gt.Rot3.Ypr(yaw_prior, 0, 0), gt.Point3(prior_pos.x, prior_pos.y, prior_pos.z)), 
                                      gt.noiseModel.Diagonal.Sigmas([1, 1, 1, 1e5, 1e5, 1e5])))
        initial_values.insert(x0, gt.Pose3(gt.Rot3.Ypr(yaw_prior, 0, 0), gt.Point3(prior_pos.x, prior_pos.y, prior_pos.z)))
        self.isam.update(graph, initial_values)
        # Also updating internal state tracker 
        self.x = np.array([prior_pos.x, 0, prior_pos.y, 0, prior_pos.z, 0, yaw_prior, 0])

    def validate_update(self, timestamp:float)->bool: 
        """
        To be called before an update. Adjusts the internal timestamp and speed for the constant velocity model. 
        Returns bool depending on if the update can be applied. 
        NOTE: eventually, this will be replaced by an IMU factor 
        """
        if timestamp > self.last_measurement_time: 
            self.dt = timestamp - self.last_measurement_time
            self.last_measurement_time = timestamp
            return True 
        else: 
            print("FG.validate_update(): Update not applied, bad timestamp.", 'error', 'loc')
            return False 

    def constant_velocity_model(self, previous_pose: gt.Pose3)->tuple: 
        """
        Supposes a constant velocity to estimate the motion of the next step. 
        NOTE: When using the velocity in self.x to compute the expected delta, the result is in the global reference frame. 
        However, BetweenFactor expects a relative movement between poses, in the local frame of the body. 
        If yaw is fixed at 0, this doesn't matter, because this simple model doesn't introduce rotation and the frames stay aligned. 
        However, at any other yaw, the global expected delta needs to be mapped into the local frame to be properly applied. 
        Ex: If the yaw is fixed at 90deg and we compute a movement of (5,0,0) in the global frame, if we don't map it to the relative frame
        then BetweenFactor will apply the 5 value to the local 'x' axis, which would result in global mvt along 'y'. 
        """
        # Expected delta in the global reference frame 
        delta_world = gt.Point3(self.x[1]*self.dt, self.x[3]*self.dt, self.x[5]*self.dt)
        # Mapping it to the local frame so BetweenFactor can be applied 
        # this takes the orientation of the previous pose and uses it to convert the global displacement into a local one 
        delta_body = previous_pose.rotation().unrotate(delta_world)
        return delta_body

    ### EXTERNAL METHODS USED BY estimator.py 
    def get_position(self):  
        return Coordinates(self.x[0], self.x[2], self.x[4])
    def get_yaw(self):
        return self.x[6]

    def incorporate_ranging_data(self, timestamp: float, anchors_ranging_data:list[tuple], tags_ranging_data:list[tuple], raw_yaw:float):
        """
        Called whenever we get new ranges from anchors or tags to add to the factor graph. 
        NOTE TODO currently not using raw_yaw to update, because without an IMU no info can be deduced on it. Yaw stays fixed with simple constant velocity model. 
        """
        if not self.validate_update(timestamp): 
            # If the timestamp is not valid, don't use this data for an update
            return 
        
        graph = gt.NonlinearFactorGraph() 
        initial_values = gt.Values() 
        current_state_id = self.state_counter
        x = gt.symbol('x', current_state_id)                             

        # Connecting to previous state through a motion model 
        # NOTE: currently using simple constant velocity and not taking yaw into account 
        previous_pose = gt.Pose3(gt.Rot3.Ypr(self.get_yaw(), 0, 0), gt.Point3(self.x[0], self.x[2], self.x[4])) # NOTE when doing a more complex model, should fetch the pose with results.atPose3()
        x_prev = gt.symbol('x', current_state_id-1) # fetching past state
        delta_local_frame = self.constant_velocity_model(previous_pose) # relative movement that got us here 
        mvt = gt.Pose3( gt.Rot3.Ypr(0,0,0), delta_local_frame ) 
        graph.add(gt.BetweenFactorPose3(x_prev, x, mvt, ODOMETRY_NOISE))
        
        initial_values.insert(x, previous_pose.compose(mvt))

        # Anchor data 
        for id, z in anchors_ranging_data: 
            anchor = gt.symbol('a', id) 
            anchor_pos = anchors.anchors_dict[id].data # List of the x, y, z coordinates in mm 
            if id not in self.seen_anchors:
                # If we have never seen this anchor, need to add a prior on it's position 
                # If we have, then no need to re-add a prior. Just directly reference it during range add
                graph.add(gt.PriorFactorPoint3(anchor, gt.Point3(*anchor_pos), ANCHOR_POS_NOISE))
                initial_values.insert(anchor, gt.Point3(*anchor_pos))
                self.seen_anchors.add(id) 
            graph.add(gt.RangeFactor3D(x, anchor, z, RANGING_NOISE))
            
        # Tag data 
        for n_id, n_pos, n_cov, z in tags_ranging_data:  # iterating over neighbor id's, positions, cov, and range data to them
            neighbor = gt.symbol('n', int(f'{n_id}000{current_state_id}'))
            graph.add(gt.PriorFactorPoint3(neighbor, gt.Point3(*n_pos), gt.noiseModel.Gaussian.Covariance(neighbor_covar)))
            initial_values.insert(neighbor, gt.Point3(*n_pos))
            graph.add(gt.RangeFactor3D(x, neighbor, z, RANGING_NOISE))
            

        # Updating graph and internal data 
        self.isam.update(graph, initial_values) 
        results = self.isam.calculateEstimate() 
        current_state_estimate = results.atPose3(x) 
        # Use the last stored state (self.x) and the new current estimate through the incorporation of the data
        # to estimate the speed and update the stored state to reflect the new one 
        new_x = np.array([current_state_estimate.x(), 
                          (current_state_estimate.x()-self.x[0])/self.dt, 
                           current_state_estimate.y(), 
                           (current_state_estimate.y()-self.x[2])/self.dt, 
                           current_state_estimate.z(), 
                           (current_state_estimate.z()-self.x[4])/self.dt, 
                           current_state_estimate.rotation().rpy()[2], 
                           0])
        self.x = new_x 

    def zero_movement_update():
        pass 
    def pedometer_update():
        pass 
    def custom_odometry_update():
        pass
