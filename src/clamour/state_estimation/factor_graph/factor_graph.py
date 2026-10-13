import gtsam as gt 
import numpy as np 

from ...custom_terminal import print 
from ..estimator import TimedIMUSample
from ...messages import RangeObservation
from ...interfaces import Anchors, Pose
from ...estimator_clock import NANOSECONDS_PER_SECOND
from ...config import (ANCHOR_POS_UNCERTAINTY,
                        IMU_ACCEL_INITIAL_BIAS, IMU_ACCEL_INITIAL_BIAS_COV, IMU_ACCEL_WALK_COV,
                        IMU_GYRO_INITIAL_BIAS,  IMU_GYRO_INITIAL_BIAS_COV, IMU_GYRO_WALK_COV)

anchors = Anchors()
### Defining noise models 
## Units in cm, like the rest of the graph
ANCHOR_POS_NOISE = gt.noiseModel.Diagonal.Sigmas([ANCHOR_POS_UNCERTAINTY, ANCHOR_POS_UNCERTAINTY, ANCHOR_POS_UNCERTAINTY]) # uncertainty in anchor placement
RANGING_NOISE = gt.noiseModel.Isotropic.Sigma(1, 15) # precise 1D measurement ~ 15cm
ZERO_MOVEMENT_NOISE = gt.noiseModel.Diagonal.Sigmas([1, 1, 1, 1, 1, 1])
IMU_INTEGRATION_COVAR = (1e-7)**2 * np.eye(3) # Represents uncertainty due to the discrete numerical integration method. Low importance & hardware independent. Value set to common GTSAM example's. 
IMU_INITIAL_BIAS_COV = np.concatenate(IMU_ACCEL_INITIAL_BIAS_COV, IMU_GYRO_INITIAL_BIAS_COV)

class FactorGraph: 
    def __init__(self, range_observations:list[RangeObservation], prior_yaw:float, timestamp_ns:int, imu_accel_covar_density:float, imu_gyro_covar_density:float):
        """
        Factor Graph based 3D pose estimator. 
        - range_observations: Range observations used to initialize the graph; at least 3 anchors are required to fully initialize the prior position.
        - prior_yaw: prior yaw value on initialization **[rad]** 
        - timestamp_ns: initial event time on EstimatorClock, used to calculate dt
        NOTE TODO ensure unit coherence between feeding IMU and internal treatment. Where do we convert? 
        NOTE on units: 
        - All angle units are **radians** 
        - All spatial units are **cm-based**
            - The graph doesn't care about this unit as long as it's fully internally consistent, but enforcing cm until we have proper code-wide documentation and unit standardisation to be safer. (For example, gravity vector in graph should be updated if change of units)
        """
        self.last_measurement_time_ns = timestamp_ns
        self.dt = None 
        
        # Graph trackers 
        self._state_counter = -1   # How many state nodes have been added to the graph
        self.seen_anchors = set()  # Anchors we have previously seen; to avoid re-adding prior factors
        self.isam = gt.ISAM2() 

        ## Initializing graph 
        initial_state_key = self.state_counter 
        graph = gt.NonlinearFactorGraph() 
        initial_values = gt.Values()
        # Defining IMU objects if present 
        if imu_accel_covar_density and imu_gyro_covar_density: 
            self.imu_accel_covar_density = imu_accel_covar_density 
            self.imu_gyro_covar_density  = imu_gyro_covar_density
            self.pim  = self._create_imu_pim_obj(initial_state_key, graph, initial_values) # PreintegratedCombinedMeasurements object, used for IMU pre-integration. 
        # Creating prior factor that locks rotation and position at initial estimate
        # Internal state tracker is a NavState for Pose and Velocity 
        self._state = self._insert_init_prior(initial_state_key, graph, initial_values, prior_yaw, range_observations) 

    ### Properties
    @property
    def state_counter(self)->int:
        self._state_counter += 1 
        return self._state_counter 

    @property
    def pose(self) -> Pose:
        """The posterior pose after running the estimator, or the last estimate if we didn't run it yet"""
        position = self._state.t()  # 1D array [x, y, z]
        rot = self._state.R().ypr() # 1D array [yaw, pitch, roll]
        return Pose(position[0], position[1], position[2], heading=rot[0], pitch=rot[1], roll=rot[2], covar=self.covars)

    ### -------------------------------------------------- EXTERNAL METHODS USED BY estimator.py --------------------------------------------------
    
    def advance_to(self, timestamp:float, imu_data:list[TimedIMUSample]|None, range_observations:list[RangeObservation]):
        """
        Creates a new state in the FG, linked to the previous by a motion factor and supported by external measurements if available. 
        Optimizes the graph and internal data with the posterior. 
        This function should be called every STATE_INTERVAL seconds by estimator.py 
        """
        if not self.validate_update(timestamp):
            # If the timestamp is not valid, don't use this data for an update
            return
        
        ## Initialize the new section of the graph and the new state
        graph = gt.NonlinearFactorGraph()
        initial_values = gt.Values()

        current_state_id = self.state_counter
        x = gt.symbol('x', current_state_id)
        v = gt.symbol('v', current_state_id)

        ## Motion factor 
        if imu_data is None: 
            self.add_constant_velocity_link(graph, initial_values, x, current_state_id)
        else:
            self._add_imu_link()

        ## External measurements 
        self.add_ranging_data(current_state_id, graph, initial_values, range_observations)

        ## Updating graph and internal data with the posterior
        self.isam.update(graph, initial_values)
        # NOTE TODO compare to calculateEstimatePose3(X(i)) to only call for latest frame? better for perf.
        post = self.isam.calculateEstimate()
        self._state = gt.NavState(post.atPose3(x), post.atVector3(v)) # NOTE TODO when connecting IMU add velocity tracking (also for ZUPT) need to use v symbol upstream
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
        pose = self.pose
        previous_pose = gt.Pose3(
            gt.Rot3.Ypr(pose.heading, 0, 0),
            gt.Point3(*pose.coordinates),
        )
        zero_motion = gt.Pose3(gt.Rot3.Ypr(0, 0, 0), gt.Point3(0, 0, 0))

        graph.add(gt.BetweenFactorPose3(x_prev, x, zero_motion, ZERO_MOVEMENT_NOISE))
        initial_values.insert(x, previous_pose)

        self.isam.update(graph, initial_values)
        post_pose = self.isam.calculateEstimate().atPose3(x)
        self._state = gt.NavState(post_pose, np.zeros(3))
        
    def pedometer_update():
        pass 
    def custom_odometry_update():
        pass

    ### -------------------------------------------------- INTERNAL COMPUTATION METHODS --------------------------------------------------
    def _insert_init_prior(self, state_key, graph, initial_values, yaw_prior:float, range_observations:list[RangeObservation])->gt.NavState: 
        """
        Constrains the initial state of the graph with a rotation lock & sufficient range data to multilaterate the position. 

        Only to be used in __init__! \n 
        NOTE: **Currently supposing graph is initialized while stationnary!**
        NOTE: To avoid imprecise/unstable measures, ensure anchors are geometrically separated enough to avoid ambiguity in the solution.
        """     
        x0 = gt.symbol('x', state_key)  
        # ROTATION LOCK 
        anchor_observations = [observation for observation in range_observations if observation.is_anchor]
        throwaway_pos = anchors.get_centroid_for(*(observation.target_id for observation in anchor_observations))
        graph.add(gt.PriorFactorPose3(x0, 
                                      gt.Pose3(gt.Rot3.Ypr(yaw_prior, 0, 0), gt.Point3(*throwaway_pos)),
                                      gt.noiseModel.Diagonal.Sigmas([1, 1, 1, 1e5, 1e5, 1e5]))) 
        initial_values.insert(x0, gt.Pose3(gt.Rot3.Ypr(yaw_prior, 0, 0), gt.Point3(*throwaway_pos)))
        # POSITION LOCK 
        self.add_ranging_data(state_key, graph, initial_values, range_observations)
        # GETTING ESTIMATE AND UPDATING INTERNAL TRACKER 
        self.isam.update(graph, initial_values)
        post_pose = self.isam.calculateEstimate().atPose3(x0) 
        covariance = self.isam.marginalCovariance(x0)
        self.covars = (
            covariance[3, 3],
            covariance[4, 4],
            covariance[5, 5],
            covariance[3, 4],
            covariance[3, 5],
            covariance[4, 5],
        )
        return gt.NavState(post_pose, np.zeros(3))  

    def _create_imu_pim_obj(self, state_key, graph, initial): 
        """
        Creates a gtsam.PreintegratedCombinedMeasurements object, which will be used to do IMU pre-integration with the CombinedImuFactor
        """
        # Defining Z axis pointing up. The factor will then handle gravity measurements internally. 
        pim_params = gt.PreintegrationCombinedParams.MakeSharedU(9.806)
        # Defining covariances (noise models) 
        pim_params.setAccelerometerCovariance(self.imu_accel_covar_density)
        pim_params.setGyroscopeCovariance(self.imu_gyro_covar_density)
        pim_params.setIntegrationCovariance(IMU_INTEGRATION_COVAR) 
        pim_params.setBiasAccCovariance( IMU_ACCEL_WALK_COV * np.eye(3)) 
        pim_params.setBiasOmegaCovariance(IMU_GYRO_WALK_COV * np.eye(3)) 
        # Setting prior on IMU bias (calibration isn't perfect, this represents our confidence in it) 
        # Subsequent uses of CombinedImuFactor will allow the bias estimate to evolve through time  
        # NOTE biases will be SUBSTRACTED from readings (coherent with theory/def of 'bias')
        imu_bias = gt.imuBias.ConstantBias(IMU_ACCEL_INITIAL_BIAS, IMU_GYRO_INITIAL_BIAS)
        graph.add(gt.PriorFactorConstantBias(gt.symbol('b', state_key), imu_bias, IMU_INITIAL_BIAS_COV))
        initial.insert(gt.symbol('b', state_key), imu_bias) 

        # Creating the preintegration object 
        # Using combined version as we'll use CombinedImuFactor later
        return gt.PreintegratedCombinedMeasurements(pim_params, imu_bias)

    def validate_update(self, timestamp_ns:int)->bool:
        """
        To be called before an update. Adjusts the internal timestamp and speed for the constant velocity model. 
        Returns bool depending on if the update can be applied. 
        NOTE: eventually, this will be replaced by an IMU factor 
        """
        if timestamp_ns > self.last_measurement_time_ns:
            self.dt = (timestamp_ns - self.last_measurement_time_ns) / NANOSECONDS_PER_SECOND
            self.last_measurement_time_ns = timestamp_ns
            return True 
        else: 
            print("FG.validate_update(): Update not applied, bad timestamp.", 'error', 'loc')
            return False 

    def add_ranging_data(self, state_id, graph, initial_values, range_observations:list[RangeObservation]): 
        """
        Adds anchor and tag range observations to a graph state without adding a prior or solving for it.
        """
        print(f"Adding range observations {range_observations}", 'info', 'loc')
        x = gt.symbol('x', state_id) 
        for observation in range_observations:
            target_id = observation.target_id
            distance = observation.distance_cm
            if observation.is_anchor:
                anchor = gt.symbol('a', target_id)
                if target_id not in self.seen_anchors:
                    # Add a prior only the first time this anchor is observed.
                    anchor_pos = anchors.anchors_dict[target_id]
                    graph.add(gt.PriorFactorPoint3(anchor, gt.Point3(*anchor_pos), ANCHOR_POS_NOISE))
                    initial_values.insert(anchor, gt.Point3(*anchor_pos))
                    self.seen_anchors.add(target_id)
                graph.add(gt.RangeFactor3D(x, anchor, distance, RANGING_NOISE))
            else: 
                # Ranging data from tags is useful only when their pose estimate is known.
                if observation.target_pose is None:
                    continue
                n_pos = observation.target_pose.coordinates
                n_cov = observation.target_pose.covar
                if n_cov is None:
                    raise ValueError(f"Tag range observation for target {target_id} has no pose covariance")
                # Adding the other tag's position to the graph with a special ID that tracks his position and the time
                neighbor = gt.symbol('t', int(f'{target_id}000{state_id}')) 
                # NOTE GTSAM needs a matrix of float types or it crashes out (builds the matrix as NaN -> indeterminate system)
                # If an indeterminate system is thrown when ranging with neighbors, printing out the matrix and checking that it's invertible 
                # is a good debugging step, as gt.noiseModel.Gaussian.Covariance will attempt to invert it to get the Information Matrix
                graph.add(gt.PriorFactorPoint3(neighbor, gt.Point3(*n_pos), gt.noiseModel.Gaussian.Covariance(n_cov.astype(float)))) 
                initial_values.insert(neighbor, gt.Point3(*n_pos))
                # Adding range data 
                graph.add(gt.RangeFactor3D(x, neighbor, distance, RANGING_NOISE))

    def _add_imu_link(self, state_id, graph, initial_values, timed_imu_samples:list[TimedIMUSample]):
        """
        Preintegrates IMU data to add a motion factor from the previous state to the next. \n 
        The new state is initialized with a prior defined by the preintegration. 
        - state_id: Key corresponding to the new state 
        - graph: Factor Graph object 
        - initial_values: Values object related to the graph 
        """
        x_old, v_old, b_old = gt.symbol('x', state_id-1), gt.symbol('v', state_id-1), gt.symbol('b', state_id-1)
        x_new, v_new, b_new = gt.symbol('x', state_id), gt.symbol('v', state_id), gt.symbol('b', state_id)
        # Pre integrating data 
        prev_ts = timed_imu_samples[0].timestamp_ns # preintegration from (start, end]
        for imu_sample in timed_imu_samples[1:]: 
            # NOTE TODO convert to proper units, check for monotonic necessary or not? 
            dt = (imu_sample.timestamp_ns-prev_ts) # convert to S 
            self.pim.integrateMeasurement(imu_sample.acceleration, imu_sample.angular_velocity, dt) 
            prev_ts = imu_sample.timestamp_ns
        # Adding pre-integrated data to the graph as a CombinedImuFactor
        # Using this factor removes the need of an independent BetweenFactor to track the bias (would be needed with ImuFactor) 
        # while keeping taking into account correlations between bias drift and IMU predictions 
        graph.add( gt.CombinedImuFactor(x_old, v_old, x_new, v_new, b_old, b_new, self.pim) )
        # NOTE TODO check if need to track speeds in other functions? range? 

        # Adding initial guess for the state with the IMU's predictions
        # NOTE TODO: can we fetch up to date estimate of bias or do we just put back the initial one? 
        dead_reckoned_state = self.pim.predict(self._state, imu_bias) 
        initial_values.insert(x_new, dead_reckoned_state.pose())
        initial_values.insert(v_new, dead_reckoned_state.velocity())
        initial_values.insert(b_new, imu_bias) 

        # Make sure to clear out preintegration values for the next run 
        self.pim.resetIntegration()

    def add_constant_velocity_link(self, graph, initial_values, current_state_symbol, current_state_id): 
        """
        Uses a constant velocity model to loosely link states with a BetweenFactorPose 
        NOTE TODO The function is ready to use but I haven't configured support to switch to it 
        Leaving it here to facilitate deploying the code on non-IMU platforms in the future if needed. 
        """
        def constant_velocity_model(nav_state): 
            """
            Supposes a constant velocity to estimate the motion of the next step. 
            NOTE: When using the velocity in self.x to compute the expected delta, the result is in the global reference frame. 
            However, BetweenFactor expects a relative movement between poses, in the local frame of the body. 
            If yaw is fixed at 0, this doesn't matter, because this simple model doesn't introduce rotation and the frames stay aligned. 
            However, at any other yaw, the global expected delta needs to be mapped into the local frame to be properly applied. 
            Ex: If the yaw is fixed at 90deg and we compute a movement of (5,0,0) in the global frame, if we don't map it to the relative frame
            then BetweenFactor will apply the 5 value to the local 'x' axis, which would result in global mvt along 'y'. 
            """
            pose, vel = nav_state.pose(), nav_state.velocity()
            # Expected delta in the global reference frame 
            delta_world = gt.Point3(vel[0]*self.dt, vel[1]*self.dt, vel[2]*self.dt)
            # Mapping it to the local frame so BetweenFactor can be applied 
            # this takes the orientation of the previous pose and uses it to convert the global displacement into a local one 
            delta_body = pose.rotation().unrotate(delta_world)
            return delta_body
        ODOMETRY_NOISE = gt.noiseModel.Diagonal.Sigmas([0.05, 0.05, 0.05, 50, 50, 20]) # ~~~constant velocity so very loose (except on z cause expect less mvt that way)
        x = current_state_symbol
        ## Using BetweenFactor to loosely link states. This is the structural analogue of a Kalman Filter's prediction step. 
        # If we didn't have any model here, then the graph would simply have individual state estimates based on ranges, 
        # which would directly be ~equivalent to blindly trilaterating and discarding each step. 
        # Having even a loose motion model allows us to link states and smooth the global trajectory, for example, correcting sudden bad trilaterations due to NLOS. 
        # Any information is useful information to include, as long as it's even slightly relevant, as it will affect the joint posterior. 
        x_prev = gt.symbol('x', current_state_id-1) # fetching past state (corresponding to the current value of self.x)
        # Connecting to previous state through a motion model 
        delta_local_frame = constant_velocity_model(self._state) # Supposing constant velocity, what would be the local vector to the new state?
        mvt = gt.Pose3( gt.Rot3.Ypr(0,0,0), delta_local_frame )    # NOTE simple constant velocity model keeps yaw constant
        graph.add(gt.BetweenFactorPose3(x_prev, x, mvt, ODOMETRY_NOISE))
        initial_values.insert(x, self._state.compose(mvt))