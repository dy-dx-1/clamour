import gtsam as gt 
import numpy as np 

from ...sensor_events import RangeObservation, TimedIMUSample
from ...custom_terminal import print 
from ...interfaces import Anchors, Pose
from ...estimator_clock import NANOSECONDS_PER_SECOND
from ...config import (ANCHOR_POS_UNCERTAINTY_CM,
                       IMU_ACCEL_INITIAL_BIAS_MG, IMU_ACCEL_INITIAL_BIAS_STD_MG, IMU_ACCEL_WALK_COV_MG2_PER_S,
                       IMU_GYRO_INITIAL_BIAS_MDPS, IMU_GYRO_INITIAL_BIAS_STD_MDPS, IMU_GYRO_WALK_COV_MDPS2_PER_S)
ANCHORS = Anchors()

### Defining noise models 
# GTSAM factors use cm-based spatial units and radians for angular quantities.
# Pose converts the posterior angles back to degrees at the public boundary.
ANCHOR_POS_NOISE = gt.noiseModel.Diagonal.Sigmas([ANCHOR_POS_UNCERTAINTY_CM] * 3)
RANGING_NOISE = gt.noiseModel.Isotropic.Sigma(1, 15)  # 15 cm
ZERO_MOVEMENT_NOISE = gt.noiseModel.Diagonal.Sigmas([1, 1, 1, 1, 1, 1])
INITIAL_VELOCITY_NOISE = gt.noiseModel.Isotropic.Sigma(3, 1.0)  # cm/s; stationary bootstrap
CONSTANT_VELOCITY_NOISE = gt.noiseModel.Isotropic.Sigma(3, 50.0)  # cm/s; deliberately loose

GRAVITY_CM_S2 = 980.665  # cm/s^2
MG_TO_CM_PER_S2 = GRAVITY_CM_S2 / 1000
MDPS_TO_RAD_PER_S = 0.001*np.pi/180
IMU_ACCEL_INITIAL_BIAS = IMU_ACCEL_INITIAL_BIAS_MG  * MG_TO_CM_PER_S2
IMU_GYRO_INITIAL_BIAS  = IMU_GYRO_INITIAL_BIAS_MDPS * MDPS_TO_RAD_PER_S
IMU_ACCEL_WALK_COV = IMU_ACCEL_WALK_COV_MG2_PER_S * MG_TO_CM_PER_S2**2
IMU_GYRO_WALK_COV = IMU_GYRO_WALK_COV_MDPS2_PER_S * MDPS_TO_RAD_PER_S**2
IMU_BIAS_STD_DEV = np.concatenate((
            IMU_ACCEL_INITIAL_BIAS_STD_MG * MG_TO_CM_PER_S2,
            IMU_GYRO_INITIAL_BIAS_STD_MDPS * MDPS_TO_RAD_PER_S,
        ))
IMU_INTEGRATION_COVAR = (1e-5)**2 * np.eye(3) # Represents uncertainty due to the discrete numerical integration method. Low importance & hardware independent. Value set to common GTSAM example's. 


class FactorGraph: 
    def __init__(self, range_observations:list[RangeObservation], prior_yaw:float, initial_state_time_ns:int, imu_accel_covar_density_mg2_s:float | None = None, imu_gyro_covar_density_mdps2_s:float | None = None):
        """
        Factor Graph based 3D pose estimator. 
        - range_observations: Range observations used to initialize the graph; at least 3 anchors are required to fully initialize the prior position.
        - prior_yaw: prior yaw value on initialization **[rad]**
        - imu_accel_covar_density_mg2_s: accelerometer measurement covariance density in (mg^2)*s
        - imu_gyro_covar_density_mdps2_s: gyroscope measurement covariance density in (mdps^2)*s
        - initial_state_time_ns: EstimatorClock timestamp of the initial graph state
        NOTE on units: 
        - All angle units are **radians** 
        - All spatial units are **cm-based**
            - IMU samples and covariance densities are converted from mg/mdps-based units at the graph boundary.
        """
        self.last_state_time_ns = initial_state_time_ns
        
        ## Graph trackers 
        self._state_counter = -1   # How many state nodes have been added to the graph
        self.seen_anchors = set()  # Anchors we have previously seen; to avoid re-adding prior factors
        self.isam = gt.ISAM2() 

        ## Initializing graph 
        initial_state_key = self.state_counter 
        graph = gt.NonlinearFactorGraph() 
        initial_values = gt.Values()
        # Defining IMU objects if present 
        if imu_accel_covar_density_mg2_s is not None and imu_gyro_covar_density_mdps2_s is not None:
            self.imu_accel_covar_density = imu_accel_covar_density_mg2_s * MG_TO_CM_PER_S2**2
            self.imu_gyro_covar_density = imu_gyro_covar_density_mdps2_s * MDPS_TO_RAD_PER_S**2
            self.pim  = self._create_imu_pim_obj(initial_state_key, graph, initial_values) # PreintegratedCombinedMeasurements object, used for IMU pre-integration. 
        # Initializing the state by locking rotation and position 
        # Internal state tracker is a NavState for Pose3 and Velocity. Public access is the pose() property, which formats it into a Pose container.
        self._state = self._insert_init_prior(initial_state_key, graph, initial_values, prior_yaw, range_observations) 

    ### Properties
    @property
    def state_counter(self)->int:
        self._state_counter += 1 
        return self._state_counter 

    @property
    def pose(self) -> Pose:
        """The posterior `Pose` in it's corresponding units after running the estimator, or the last estimate if we didn't run it yet"""
        position = self._state.t()  # 1D array [x, y, z]
        rot = self._state.R().ypr() # 1D array [yaw, pitch, roll]
        return Pose(position[0], position[1], position[2], heading=np.rad2deg(rot[0]), pitch=np.rad2deg(rot[1]), roll=np.rad2deg(rot[2]), covar=self.covars)

    ### -------------------------------------------------- EXTERNAL METHODS USED BY estimator.py --------------------------------------------------
    
    def advance_to(self, state_boundary_ns:int, imu_start_sample:TimedIMUSample | None,
                   imu_data:list[TimedIMUSample], range_observations:list[RangeObservation]):
        """
        Creates a new state in the FG, linked to the previous by a motion factor and supported by external measurements if available. 
        Optimizes the graph and internal data with the posterior. 
        This function should be called every STATE_INTERVAL seconds by estimator.py 

        ``imu_start_sample`` is held from the prior state boundary until the
        first item in ``imu_data``.  It is ``None`` only before IMU bootstrap
        or when no IMU is configured, in which case the constant-velocity
        fallback is used.
        """
        if imu_start_sample is not None and not hasattr(self, "pim"):
            raise ValueError("IMU samples were provided to a factor graph initialized without IMU covariance densities")
        # state_boundary_ns is the timestamp of the new cadence state, not
        # the first/last IMU sample or a range timestamp.  The coordinator owns
        # measurement validity; this only protects the graph's state timeline.
        state_start_ns = self.last_state_time_ns
        dt_s = self._prepare_new_state(state_boundary_ns)
        
        ## Initialize the new section of the graph and the new state
        graph = gt.NonlinearFactorGraph()
        initial_values = gt.Values()

        new_state_id = self.state_counter
        x_new = gt.symbol('x', new_state_id)
        v_new = gt.symbol('v', new_state_id)

        ## Motion factor 
        if imu_start_sample is None:
            self._add_constant_velocity_link(new_state_id, graph, initial_values, dt_s)
        else:
            self._add_imu_link(
                new_state_id,
                graph,
                initial_values,
                state_start_ns,
                state_boundary_ns,
                imu_start_sample,
                imu_data,
            )

        ## External measurements 
        self._add_ranging_data(new_state_id, graph, initial_values, range_observations)

        ## Updating graph and internal data with the posterior
        self.isam.update(graph, initial_values)
        # NOTE TODO compare to calculateEstimatePose3(X(i)) to only call for latest frame? better for perf.
        post = self.isam.calculateEstimate()
        self._state = gt.NavState(post.atPose3(x_new), post.atVector3(v_new))
        if imu_start_sample is not None:
            self.pim.resetIntegrationAndSetBias(post.atConstantBias(gt.symbol('b', new_state_id)))
        # Updating covariance. No need to cast to int here as update_covar called in estimator.py will do it
        covariance = self.isam.marginalCovariance(x_new)
        self.covars = (
            covariance[3, 3],
            covariance[4, 4],
            covariance[5, 5],
            covariance[3, 4],
            covariance[3, 5],
            covariance[4, 5],
        )   

    def zero_movement_update(self, state_boundary_ns: int) -> None:
        """Temporarily constrain the next pose to the current one after an input gap.

        This mirrors the EKF's synthetic zero-movement update to limit drift from
        the constant-velocity model. It is valid only when an idle input queue
        really means the tag is stationary; it will be replaced by IMU-based
        propagation and stationary detection.
        """
        self._prepare_new_state(state_boundary_ns)

        graph = gt.NonlinearFactorGraph()
        initial_values = gt.Values()

        current_state_id = self.state_counter
        x = gt.symbol('x', current_state_id)
        x_prev = gt.symbol('x', current_state_id - 1)
        pose = self.pose
        previous_pose = gt.Pose3(
            gt.Rot3.Ypr(np.deg2rad(pose.heading), 0, 0),
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
        v0 = gt.symbol('v', state_key)
        # ROTATION LOCK 
        anchor_observations = [observation for observation in range_observations if observation.is_anchor]
        throwaway_pos = ANCHORS.get_centroid_for(*(observation.target_id for observation in anchor_observations))
        graph.add(gt.PriorFactorPose3(x0, 
                                      gt.Pose3(gt.Rot3.Ypr(yaw_prior, 0, 0), gt.Point3(*throwaway_pos)),
                                      gt.noiseModel.Diagonal.Sigmas([1, 1, 1, 1e5, 1e5, 1e5]))) 
        initial_values.insert(x0, gt.Pose3(gt.Rot3.Ypr(yaw_prior, 0, 0), gt.Point3(*throwaway_pos)))
        # Bootstrap is explicitly stationary.  This both seeds V(0) for the
        # first CombinedImuFactor and prevents an unconstrained velocity key.
        initial_velocity = np.zeros(3)
        graph.add(gt.PriorFactorVector(v0, initial_velocity, INITIAL_VELOCITY_NOISE))
        initial_values.insert(v0, initial_velocity)
        # POSITION LOCK 
        self._add_ranging_data(state_key, graph, initial_values, range_observations)
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
        pim_params = gt.PreintegrationCombinedParams.MakeSharedU(GRAVITY_CM_S2)
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
        bias_noise = gt.noiseModel.Diagonal.Sigmas(IMU_BIAS_STD_DEV)
        graph.add(gt.PriorFactorConstantBias(gt.symbol('b', state_key), imu_bias, bias_noise))
        initial.insert(gt.symbol('b', state_key), imu_bias) 

        # Creating the preintegration object 
        # Using combined version as we'll use CombinedImuFactor later
        return gt.PreintegratedCombinedMeasurements(pim_params, imu_bias)

    def _prepare_new_state(self, state_boundary_ns: int) -> float:
        """Advance the graph-state clock and return its duration in seconds.

        ``state_boundary_ns`` must be the canonical EstimatorClock timestamp for
        the *new state boundary*.  It must not be a range timestamp or the first
        or last IMU sample timestamp.  A non-increasing boundary is a coordinator
        bug, so fail rather than silently omitting a scheduled graph state.
        """
        if state_boundary_ns <= self.last_state_time_ns:
            raise ValueError(
                "FactorGraph received a non-increasing state boundary: "
                f"{state_boundary_ns} <= {self.last_state_time_ns}"
            )

        dt_s = (state_boundary_ns - self.last_state_time_ns) / NANOSECONDS_PER_SECOND
        self.last_state_time_ns = state_boundary_ns
        return dt_s

    def _add_ranging_data(self, state_key, graph, initial_values, range_observations:list[RangeObservation]): 
        """
        Adds anchor and tag range observations to a graph state without adding a prior or solving for it.
        """
        print(f"Adding range observations {[f"{ro.target_id}: {ro.distance_cm}cm" for ro in range_observations]}", 'info', 'loc')
        x = gt.symbol('x', state_key) 
        for observation in range_observations:
            target_id = observation.target_id
            distance = observation.distance_cm
            if observation.is_anchor:
                anchor = gt.symbol('a', target_id)
                if target_id not in self.seen_anchors:
                    # Add a prior only the first time this anchor is observed.
                    anchor_pos = ANCHORS.anchors_dict[target_id]
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
                    raise ValueError(f"Tag range observation for target {target_id} has no pose covariance. This shouldn't happen, check why the task.py check didn't work.")
                # Adding the other tag's position to the graph with a special ID that tracks it's position and the state
                neighbor = gt.symbol('t', int(f'{target_id}000{state_key}')) 
                # NOTE GTSAM needs a matrix of float types or it crashes out (builds the matrix as NaN -> indeterminate system)
                # If an indeterminate system is thrown when ranging with neighbors, printing out the matrix and checking that it's invertible 
                # is a good debugging step, as gt.noiseModel.Gaussian.Covariance will attempt to invert it to get the Information Matrix
                graph.add(gt.PriorFactorPoint3(neighbor, gt.Point3(*n_pos), gt.noiseModel.Gaussian.Covariance(n_cov.astype(float)))) 
                initial_values.insert(neighbor, gt.Point3(*n_pos))
                # Adding range data 
                graph.add(gt.RangeFactor3D(x, neighbor, distance, RANGING_NOISE))

    def _add_imu_link(self, state_id, graph, initial_values, interval_start_ns:int,
                      interval_end_ns:int, start_sample:TimedIMUSample,
                      timed_imu_samples:list[TimedIMUSample]):
        """
        Preintegrates IMU data to add a motion factor from the previous state to the next. \n 
        The new state is initialized with a prior defined by the preintegration. 
        - state_id: Key corresponding to the new state 
        - graph: Factor Graph object 
        - initial_values: Values object related to the graph 
        """
        x_old, v_old, b_old = gt.symbol('x', state_id-1), gt.symbol('v', state_id-1), gt.symbol('b', state_id-1)
        x_new, v_new, b_new = gt.symbol('x', state_id),   gt.symbol('v', state_id),   gt.symbol('b', state_id)
        # Zero-order hold: apply each measurement from its timestamp until the
        # next sample, including both graph-state boundaries.  Consequently the
        # integrated duration is exactly ``interval_end - interval_start``.
        cursor_ns = interval_start_ns
        held_sample = start_sample
        for imu_sample in timed_imu_samples:
            self._integrate_held_imu_sample(held_sample, imu_sample.timestamp_ns - cursor_ns)
            cursor_ns = imu_sample.timestamp_ns
            held_sample = imu_sample
        self._integrate_held_imu_sample(held_sample, interval_end_ns - cursor_ns)

        # Adding pre-integrated data to the graph as a CombinedImuFactor
        # This removes the need of an independent BetweenFactor to track the bias (needed with ImuFactor)
        # while still taking into account correlations between bias drift and IMU predictions
        graph.add( gt.CombinedImuFactor(x_old, v_old, x_new, v_new, b_old, b_new, self.pim) )

        # Using IMU preintegration as initial values. Bias is set to the latest posterior. 
        current_bias = self.isam.calculateEstimate().atConstantBias(b_old)
        dead_reckoned_state = self.pim.predict(self._state, current_bias)
        initial_values.insert(x_new, dead_reckoned_state.pose())
        initial_values.insert(v_new, dead_reckoned_state.velocity())
        initial_values.insert(b_new, current_bias)

    def _integrate_held_imu_sample(self, sample: TimedIMUSample, duration_ns: int) -> None:
        """Integrate one zero-order-held IMU value for a positive duration."""
        if duration_ns < 0:
            raise ValueError("IMU samples must be chronological within a state interval")
        if duration_ns == 0:
            return
        self.pim.integrateMeasurement(
            sample.acceleration * MG_TO_CM_PER_S2,
            sample.angular_velocity * MDPS_TO_RAD_PER_S,
            duration_ns / NANOSECONDS_PER_SECOND,
        )

    def _add_constant_velocity_link(self, current_state_id, graph, initial_values, dt_s: float):
        """
        Uses a constant velocity model to loosely link states with a BetweenFactorPose and uses the relative movement to seed the solver. 

        Note that this simple model cannot track rotations so the orientation is left unchanged. 
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
            delta_world = gt.Point3(vel[0]*dt_s, vel[1]*dt_s, vel[2]*dt_s)
            # Mapping it to the local frame so BetweenFactor can be applied 
            # this takes the orientation of the previous pose and uses it to convert the global displacement into a local one 
            delta_body = pose.rotation().unrotate(delta_world)
            return delta_body
        ODOMETRY_NOISE = gt.noiseModel.Diagonal.Sigmas([0.05, 0.05, 0.05, 50, 50, 20]) # ~~~constant velocity so very loose (except on z cause expect less mvt that way)
        x = gt.symbol('x', current_state_id)
        v = gt.symbol('v', current_state_id)
        ## Using BetweenFactor to loosely link states. This is the structural analogue of a Kalman Filter's prediction step. 
        # If we didn't have any model here, then the graph would simply have individual state estimates based on ranges, 
        # which would directly be ~equivalent to blindly trilaterating and discarding each step. 
        # Having even a loose motion model allows us to link states and smooth the global trajectory, for example, correcting sudden bad trilaterations due to NLOS. 
        # Any information is useful information to include, as long as it's even slightly relevant, as it will affect the joint posterior. 
        x_prev = gt.symbol('x', current_state_id-1) # fetching past state (corresponding to the current value of self.x)
        # Motion model
        delta_local_frame = constant_velocity_model(self._state) # Supposing constant velocity, what would be the local vector to the new state?
        relative_mvt = gt.Pose3( gt.Rot3.Ypr(0,0,0), delta_local_frame )  
        graph.add(gt.BetweenFactorPose3(x_prev, x, relative_mvt, ODOMETRY_NOISE))
        initial_values.insert(x, self._state.compose(relative_mvt))
        # Carry the velocity through range-only intervals so every state has a
        # valid V(k), including the path used by ``advance_to`` to form NavState.
        v_prev = gt.symbol('v', current_state_id - 1)
        previous_velocity = self._state.velocity()
        graph.add(gt.BetweenFactorVector(v_prev, v, np.zeros(3), CONSTANT_VELOCITY_NOISE))
        initial_values.insert(v, previous_velocity)
