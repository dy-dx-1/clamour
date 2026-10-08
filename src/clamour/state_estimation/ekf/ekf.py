from ...custom_terminal import print 
from filterpy.kalman import ExtendedKalmanFilter
from numpy import array, asarray, ndarray, dot, eye, linalg
from scipy.optimize import least_squares

from .customOdometry import CustomOdometry
from ...interfaces import Pose, Anchors
from ...estimator_clock import NANOSECONDS_PER_SECOND

anchors = Anchors()

class CustomEKF(ExtendedKalmanFilter):
    def __init__(self, anchor_data:list[tuple[int, int]], yaw: float):
        super(CustomEKF, self).__init__(dim_x=8, dim_z=4)

        self.dt = 0.1
        self.last_measurement_time_ns: int | None = None
        self.set_qf()
        self.R_pedometer = array([[20, 0, 0, 0],
                                  [0, 20, 0, 0],
                                  [0, 0, 20, 0],
                                  [0, 0, 0, 0.5]])

        self.R_trilateration = array([[20, 0, 0, 0],
                                      [0, 20, 0, 0],
                                      [0, 0, 20, 0],
                                      [0, 0, 0, 0.5]])

        self.R_ranging = array([[25, 0, 0, 0],
                                [0, 25, 0, 0],
                                [0, 0, 25, 0],
                                [0, 0, 0, 0.5]])

        self.R_zero_movement = array([[1, 0, 0, 0],
                                      [0, 1, 0, 0],
                                      [0, 0, 1, 0],
                                      [0, 0, 0, 1]])

        self.observation_matrix = array([[1, 0, 0, 0, 0, 0, 0, 0],
                                         [0, 0, 1, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 1, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0, 1, 0]])

        # anchor_data is a list of (id, range) for available anchors. we use the centroid as rough initial guess
        # anyways, in estimator.py, a subsequent call to incorporate_ranging_data will fix the position properly
        # we just need self.x to be a rough guess initially to provide a starting value for the non-linear optimization
        position = anchors.get_centroid_for(*[data[0] for data in anchor_data])
        self.x = array([position[0], 0, position[1], 0, position[2], 0, yaw, 0])

    @property
    def pose(self) -> Pose:
        """The posterior pose after running the estimator, or the last estimate if we didn't run it yet"""
        pose = Pose(self.x[0], self.x[2], self.x[4], heading=self.x[6])
        pose.update_covar((
            self.P[0, 0],
            self.P[2, 2],
            self.P[4, 4],
            self.P[0, 2],
            self.P[0, 4],
            self.P[2, 4],
        ))
        return pose

    def set_qf(self):
        # As we integrate to find position, we lose precision. Thus we trust x less than dx/dt, hence the dt*2 vs dt.
        self.Q = array([[self.dt * 2, 0, 0, 0, 0, 0, 0, 0],
                        [0, self.dt, 0, 0, 0, 0, 0, 0],
                        [0, 0, self.dt * 2, 0, 0, 0, 0, 0],
                        [0, 0, 0, self.dt, 0, 0, 0, 0],
                        [0, 0, 0, 0, self.dt * 2, 0, 0, 0],
                        [0, 0, 0, 0, 0, self.dt, 0, 0],
                        [0, 0, 0, 0, 0, 0, self.dt * 2, 0],
                        [0, 0, 0, 0, 0, 0, 0, self.dt]])

        self.F = eye(8) + array([[0, self.dt, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, self.dt, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, self.dt, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, self.dt],
                                 [0, 0, 0, 0, 0, 0, 0, 0]])

    def hx_pedometer(self, x) -> ndarray:
        return dot(self.observation_matrix, x)

    def hx_trilateration(self, x) -> ndarray:
        return dot(self.observation_matrix, x)

    def hx_zero_movement(self, x) -> ndarray:
        return dot(self.observation_matrix, x)

    def hx_custom_odometry(self, x) -> ndarray:
        return dot(self.observation_matrix, x)

    @staticmethod
    def hx_ranging(x, neighbor_positions: ndarray, yaw: float) -> ndarray:
        nb_neighbors = neighbor_positions.shape[0]

        hx = array([0, 0, 0, yaw])
        for i in range(3):
            if nb_neighbors > i:
                hx[i] = linalg.norm([x[0] - neighbor_positions[i][0],
                                     x[2] - neighbor_positions[i][1],
                                     x[4] - neighbor_positions[i][2]])

        return hx

    @staticmethod
    def h_ranging(x, nei_pose) -> array:
        """Compute Jacobian of H matrix for state x """
        num_nei = nei_pose.shape
        deltas = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        for i in range(3):
            if num_nei[0] > i:
                norm = linalg.norm([x[0] - nei_pose[i][0], x[2] - nei_pose[i][1], x[4] - nei_pose[i][2]])
                for j in range(3):
                    deltas[i * 3 + j] = 0 if norm == 0 else (x[j * 2] - nei_pose[i][j]) / norm

        return array([[deltas[0], 0, deltas[1], 0, deltas[2], 0, 0, 0],
                      [deltas[3], 0, deltas[4], 0, deltas[5], 0, 0, 0],
                      [deltas[6], 0, deltas[7], 0, deltas[8], 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1, 0]])

    def pre_update(self, timestamp_ns: int) -> bool:
        """Prepare a measurement update using canonical nanosecond event time."""
        if self.last_measurement_time_ns is None:
            self.last_measurement_time_ns = timestamp_ns
            return True
        if timestamp_ns <= self.last_measurement_time_ns:
            print("CustomEKF.pre_update(): Received message with bad timestamp", 'error', 'loc')
            return False

        self.dt = (timestamp_ns - self.last_measurement_time_ns) / NANOSECONDS_PER_SECOND
        self.last_measurement_time_ns = timestamp_ns
        self.set_qf()
        self.predict()
        return True

    def custom_odometry_update(self, position: Pose, yaw: float, R, timestamp: float) -> None:
        print("CustomEKF.custom_odometry_update(): Custom odometry update", 'error', 'loc')
        if not self.pre_update(timestamp):
            return

        super(CustomEKF, self).update(
            asarray([position.x, position.y, position.z, yaw]),
            lambda _: self.observation_matrix,
            self.hx_custom_odometry,
            R
        )

    def pedometer_update(self, position: Pose, yaw: float, timestamp: float) -> None:
        if not self.pre_update(timestamp):
            return

        super(CustomEKF, self).update(asarray([position.x, position.y, position.z, yaw]),
                                      lambda _: self.observation_matrix,
                                      self.hx_pedometer, self.R_pedometer)

    def incorporate_ranging_data(self, timestamp: float, range_observations:list, raw_yaw:float):
        """Apply a single batched list of range observations to the EKF."""
        anchor_ranges = [(obs.target_id, obs.distance_cm) for obs in range_observations if obs.is_anchor]
        tag_ranges = [(obs.target_id, obs.target_pose, obs.distance_cm) for obs in range_observations if not obs.is_anchor and obs.target_pose is not None]

        if len(anchor_ranges) >= 3:  # Enough anchors for trilateration update, trilaterate position and update EKF
            anchor_pos = []
            anchor_dist = []
            for id, dist in anchor_ranges:
                anchor_pos.append(anchors.anchors_dict[id])
                anchor_dist.append(dist)
            # Residual function (Error = Calculated Distance - Measured Distance)
            def equations(position):
                calculated_distances = linalg.norm(anchor_pos - position, axis=1)
                return calculated_distances - anchor_dist
            # Solving with Non-linear Least Squares (Levenberg-Marquardt)
            raw_pos = least_squares(equations, array(self.pose.coordinates), method='lm')
            self.trilateration_update(Pose(raw_pos.x[0], raw_pos.x[1], raw_pos.x[2]), raw_yaw, timestamp)

        else:  # Not enough anchors for trilateration; add multiple ranging updates
            for id, z in anchor_ranges:
                formatted_dist = Pose(z, 0, 0)
                formatted_target_pos = array([anchors.anchors_dict[id]])
                self.ranging_update(formatted_dist, raw_yaw, timestamp, formatted_target_pos)

            for n_id, n_pos, z in tag_ranges:
                formatted_dist = Pose(z, 0, 0)
                formatted_target_pos = array([[n_pos.x, n_pos.y, n_pos.z]])
                self.ranging_update(formatted_dist, raw_yaw, timestamp, formatted_target_pos)

    def trilateration_update(self, position: Pose, yaw: float, timestamp: float) -> None:
        if not self.pre_update(timestamp):
            return

        super(CustomEKF, self).update(asarray([position.x, position.y, position.z, yaw]),
                                      lambda _: self.observation_matrix,
                                      self.hx_trilateration, self.R_trilateration)

    def ranging_update(self, distance: Pose, yaw: float, timestamp: float, neighbor_position: ndarray) -> None:
        if not self.pre_update(timestamp):
            return

        super(CustomEKF, self).update(asarray([distance.x, distance.y, distance.z, yaw]),
                                      self.h_ranging, self.hx_ranging, self.R_ranging,
                                      args=neighbor_position,
                                      hx_args=(neighbor_position, yaw))

    def zero_movement_update(self, timestamp: float) -> None:
        """This function updates the filter with its previous state.
        This allows to keep the dt relatively small and avoid drift.
        Indeed, if dt is too big, the process noise increase even if there was no change to the state."""

        if not self.pre_update(timestamp):
            return
        pose = self.pose
        super(CustomEKF, self).update(asarray([pose.x, pose.y, pose.z, pose.heading]),
                                      lambda _: self.observation_matrix,
                                      self.hx_zero_movement, self.R_zero_movement)

    def add_custom_odometry(self, custom_odometry: CustomOdometry):
        self.custom_R.append(array(custom_odometry.get_R()))
