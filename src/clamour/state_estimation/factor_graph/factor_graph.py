import gtsam as gt 
import numpy as np 

from ...custom_terminal import print 
from ...interfaces import Coordinates 
from ...interfaces import Anchors 

anchors = Anchors()
### Defining noise models 
## Units in cm, like the rest of the graph
ANCHOR_POS_NOISE = gt.noiseModel.Diagonal.Sigmas([5, 5, 5]) # uncertainty in anchor placement
RANGING_NOISE = gt.noiseModel.Isotropic.Sigma(1, 10) # precise 1D measurement ~ 10cm
ODOMETRY_NOISE = gt.noiseModel.Diagonal.Sigmas([0.05, 0.05, 0.05, 50, 50, 20]) # currently using constant velocity so very loose (except on z cause expect less mvt that way)
ZERO_MOVEMENT_NOISE = gt.noiseModel.Diagonal.Sigmas([1, 1, 1, 1, 1, 1])

class PoseGraph: 
    def __init__(self, anchors_range_data:list[tuple[int, int]], prior_yaw:float, timestamp:float): 
        """
        Factor Graph based 3D pose estimator. 
        - anchors_range_data: [(anchor_id, range), ...] At least 3 are required to initialize the prior position 
        NOTE finish docstring 
        """
        # Internal data about the current state. Kept up to date and fed back to estimator.py  
        # State vector is [x,xdot,y,ydot,z,zdot,theta,thetadot]. Only here for clarity, as the prior will overwrite these 0s. 
        self.x = np.array([0, 0, 0, 0, 0, 0, 0, 0]) 
        self.covars = (0, 0, 0, 0, 0, 0)         # Covar on position xx, yy, zz, xy, xz, yz 
        self.last_measurement_time = timestamp   # Will be updated during subsequent call of validate_update by incorporate_ranging_data 
        self.dt = None 
        
        # Graph trackers 
        self._state_counter = 0   # Keeps track of how many state nodes have been added to the graph
        self.seen_anchors = set() # Keeps track of anchors we have previously seen, to avoid re-adding prior factors
        self.isam = gt.ISAM2() 

        # Creating prior factor that locks rotation and position at initial estimate
        self.insert_prior(prior_yaw, anchors_range_data) 
        print(f"Initial state: {self.x}", 'ok', 'loc')

    ### Properties
    @property
    def state_counter(self)->int:
        self._state_counter += 1 
        return self._state_counter 

    ### INTERNAL COMPUTATION METHODS 
    def insert_prior(self, yaw_prior:float, anchors_range_data:list[tuple[int, int]]): 
        """
        Only to be used in __init__! Updates the graph and internal state with a prior that locks rotation and range factors that lock position. 
        As of 2026-08-20, this function is only here while we don't have an IMU. 
        No way to measure rotation, so we don't track/update it. However, to avoid an inderterminate system in the graph, we need to initialize it to a known value.
        
        Therefore, we insert a prior with very loose covariance on a rough position, but tight on the rotation. 
        The position component of the prior is simply the centroid of the anchors. This is inaccurate and only serves to put a rough guess that will be corrected by the anchors. 
        We then call add_ranging_data which will lock the position appropriately with range factors. 
        
        NOTE: To avoid imprecise/unstable measures, one should ensure anchors are geometrically separated enough to avoid ambiguity in the solution.
        """
        ### ROTATION LOCK 
        throwaway_pos = anchors.get_centroid_for(*(data[0] for data in anchors_range_data))
        # NOTE check if way to avoid locking in position even loosely before anchors? 
        state_key = self.state_counter
        x0 = gt.symbol('x', state_key) 
        graph = gt.NonlinearFactorGraph() 
        initial_values = gt.Values() 
        graph.add(gt.PriorFactorPose3(x0, 
                                      gt.Pose3(gt.Rot3.Ypr(yaw_prior, 0, 0), gt.Point3(throwaway_pos.x, throwaway_pos.y, throwaway_pos.z)), 
                                      gt.noiseModel.Diagonal.Sigmas([1, 1, 1, 1e5, 1e5, 1e5])))
        initial_values.insert(x0, gt.Pose3(gt.Rot3.Ypr(yaw_prior, 0, 0), gt.Point3(throwaway_pos.x, throwaway_pos.y, throwaway_pos.z)))
        ### POSITION LOCK 
        self.add_ranging_data(x0, state_key, graph, initial_values, anchors_range_data, [])
        ### GETTING ESTIMATE AND UPDATING INTERNAL TRACKER 
        self.isam.update(graph, initial_values)
        current_state_estimate = self.isam.calculateEstimate().atPose3(x0) 
        self.x = np.array([current_state_estimate.x(), 0, current_state_estimate.y(), 0, current_state_estimate.z(), 0, yaw_prior, 0])

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
        print(f"Speed: {(self.x[1], self.x[3], self.x[5])}", 'info', 'loc') 
        print(f"Delta: {(self.x[1]*self.dt, self.x[3]*self.dt, self.x[5]*self.dt)}", 'info', 'loc') 
        delta_world = gt.Point3(self.x[1]*self.dt, self.x[3]*self.dt, self.x[5]*self.dt)
        # Mapping it to the local frame so BetweenFactor can be applied 
        # this takes the orientation of the previous pose and uses it to convert the global displacement into a local one 
        delta_body = previous_pose.rotation().unrotate(delta_world)
        return delta_body

    def add_ranging_data(self, state_symbol, state_id, graph, initial_values, anchors_ranging_data:list[tuple], tags_ranging_data:list[tuple]): 
        """
        Add anchor and tag ranging data of a new state to the graph 
        - state_symbol: gt.Symbol of the new state to add to the graph 
        - state_id: Key corresponding to the new state 
        - graph: Factor Graph object 
        - initial_values: Values object related to the graph 
        - anchors_ranging_data: list of measurements [(anchor_id, range), ...] 
        - tags_ranging_data: list of measurements [(neighbor_id, neighbor_Coordinates, range), ...] 
        """
        print(f"Adding anchors {anchors_ranging_data} ||| Tags {tags_ranging_data}", 'info', 'loc')
        # ANCHORS
        for id, z in anchors_ranging_data: 
            anchor = gt.symbol('a', id) 
            anchor_pos = anchors.anchors_dict[id].data # List of the x, y, z coordinates in cm
            if id not in self.seen_anchors:
                # If we have never seen this anchor, need to add a prior on it's position 
                # If we have, then no need to re-add a prior. Just directly reference it during range add
                graph.add(gt.PriorFactorPoint3(anchor, gt.Point3(*anchor_pos), ANCHOR_POS_NOISE))
                initial_values.insert(anchor, gt.Point3(*anchor_pos))
                self.seen_anchors.add(id) 
            graph.add(gt.RangeFactor3D(state_symbol, anchor, z, RANGING_NOISE))
        # TAGS 
        for n_id, n_coords, z in tags_ranging_data:  
            # Iterating over neighbor id's, Coordinates, and range between us
            # tags_ranging_data only contains tags that have known positions/covar (filtered at TASK level)
            # NOTE: in future, could be nice to add here or in TASK a filter for stale data based on timestamps 
            n_pos, n_cov = n_coords.data, n_coords.covar  
            # Adding the other tag's position to the graph with a special ID that tracks his position and the time
            neighbor = gt.symbol('t', int(f'{n_id}000{state_id}')) 
            graph.add(gt.PriorFactorPoint3(neighbor, gt.Point3(*n_pos), gt.noiseModel.Gaussian.Covariance(n_cov)))
            initial_values.insert(neighbor, gt.Point3(*n_pos))
            # Adding range data 
            graph.add(gt.RangeFactor3D(state_symbol, neighbor, z, RANGING_NOISE))

    ### EXTERNAL METHODS USED BY estimator.py 
    def get_position(self)->Coordinates:  
        """
        Current posterior position in Coordinates format. 
        """
        return Coordinates(self.x[0], self.x[2], self.x[4])
    def get_covars(self)->tuple: 
        """
        Current covariance on the position in a tuple (xx, yy, zz, xy, xz, yz) to match Coordinates.update_covar method
        """
        return self.covars 
    def get_yaw(self)->float:
        """
        Current posterior yaw
        """
        return self.x[6]

    def incorporate_ranging_data(self, timestamp: float, anchors_ranging_data:list[tuple], tags_ranging_data:list[tuple[int, Coordinates, int]], raw_yaw:float):
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
        x = gt.symbol('x', current_state_id)        # new state to add    

        ## Using BetweenFactor to loosely link states. This is the structural analogue of a Kalman Filter's prediction step. 
        # If we didn't have any model here, then the graph would simply have individual state estimates based on ranges, 
        # which would directly be ~equivalent to blindly trilaterating and discarding each step. 
        # Having even a loose motion model allows us to link states and smooth the global trajectory, for example, correcting sudden bad trilaterations due to NLOS. 
        # Any information is useful information to include, as long as it's even slightly relevant, as it will affect the joint posterior. 
        x_prev = gt.symbol('x', current_state_id-1) # fetching past state (corresponding to the current value of self.x)
        previous_pose = gt.Pose3(gt.Rot3.Ypr(self.get_yaw(), 0, 0), gt.Point3(*self.get_position().data)) # NOTE when doing a more complex model, should fetch the pose with results.atPose3()
        # Connecting to previous state through a motion model 
        delta_local_frame = self.constant_velocity_model(previous_pose) # Supposing constant velocity, what would be the local vector to the new state?
        mvt = gt.Pose3( gt.Rot3.Ypr(0,0,0), delta_local_frame ) # NOTE simple constant velocity model keeps yaw constant
        graph.add(gt.BetweenFactorPose3(x_prev, x, mvt, ODOMETRY_NOISE))
        initial_values.insert(x, previous_pose.compose(mvt))

        ## Adding ranges 
        self.add_ranging_data(x, current_state_id, graph, initial_values, anchors_ranging_data, tags_ranging_data) 
            
        # Updating graph and internal data 
        self.isam.update(graph, initial_values) 
        current_state_estimate = self.isam.calculateEstimate().atPose3(x) 
        # Use the last stored state (self.x) and the new current estimate through the incorporation of the data
        # to estimate the speed and update the stored state to reflect the new one 
        posterior = np.array([current_state_estimate.x(), 
                          (current_state_estimate.x()-self.x[0])/self.dt, 
                           current_state_estimate.y(), 
                           (current_state_estimate.y()-self.x[2])/self.dt, 
                           current_state_estimate.z(), 
                           (current_state_estimate.z()-self.x[4])/self.dt, 
                           current_state_estimate.rotation().rpy()[2], 
                           0])
        self.x = posterior 
        # Updating covariance. No need to cast to int here as update_covar called in estimator.py will do it 
        covariance = self.isam.marginalCovariance(x)
        self.covars = (
            covariance[3, 3],
            covariance[4, 4],
            covariance[5, 5],
            covariance[3, 4],
            covariance[3, 5],
            covariance[4, 5],
        )
        
    def zero_movement_update(self, timestamp: float) -> None:
        """Temporarily constrain the next pose to the current one after an input gap.

        This mirrors the EKF's synthetic zero-movement update to limit drift from
        the constant-velocity model. It is valid only when an idle input queue
        really means the tag is stationary; it will be replaced by IMU-based
        propagation and stationary detection.
        """
        if not self.validate_update(timestamp):
            return

        graph = gt.NonlinearFactorGraph()
        initial_values = gt.Values()

        current_state_id = self.state_counter
        x = gt.symbol('x', current_state_id)
        x_prev = gt.symbol('x', current_state_id - 1)
        previous_pose = gt.Pose3(
            gt.Rot3.Ypr(self.get_yaw(), 0, 0),
            gt.Point3(*self.get_position().data),
        )
        zero_motion = gt.Pose3(gt.Rot3.Ypr(0, 0, 0), gt.Point3(0, 0, 0))

        graph.add(gt.BetweenFactorPose3(x_prev, x, zero_motion, ZERO_MOVEMENT_NOISE))
        initial_values.insert(x, previous_pose)

        self.isam.update(graph, initial_values)
        current_state_estimate = self.isam.calculateEstimate().atPose3(x)
        self.x = np.array([
            current_state_estimate.x(),
            (current_state_estimate.x() - self.x[0]) / self.dt,
            current_state_estimate.y(),
            (current_state_estimate.y() - self.x[2]) / self.dt,
            current_state_estimate.z(),
            (current_state_estimate.z() - self.x[4]) / self.dt,
            current_state_estimate.rotation().rpy()[2],
            0])
        
    def pedometer_update():
        pass 
    def custom_odometry_update():
        pass
