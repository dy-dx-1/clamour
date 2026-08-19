from ...custom_terminal import print 
from filterpy.kalman import ExtendedKalmanFilter
from numpy import array, asarray, ndarray, dot, eye, linalg
from scipy.optimize import least_squares

from .customOdometry import CustomOdometry
from ...interfaces import Coordinates, Anchors

anchors = Anchors().anchors_dict

class CustomEKF(ExtendedKalmanFilter):
    def __init__(self, position: Coordinates, yaw: float):
        super(CustomEKF, self).__init__(dim_x=8, dim_z=4)

        self.dt = 0.1
        self.last_measurement_time = 0
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

        self.x = array([position.x, 0, position.y, 0, position.z, 0, yaw, 0])

    def get_position(self) -> Coordinates:
        return Coordinates(self.x[0], self.x[2], self.x[4])

    def get_yaw(self) -> float:
        return self.x[6]

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

    def pre_update(self, timestamp: float) -> None:
        if timestamp > self.last_measurement_time:
            self.dt = timestamp - self.last_measurement_time
            self.last_measurement_time = timestamp
            self.set_qf()
        else:
            print("CustomEKF.pre_update(): Received message with bad timestamp", 'error', 'loc')
        self.predict()

    def custom_odometry_update(self, position: Coordinates, yaw: float, R, timestamp: float) -> None:
        print("CustomEKF.custom_odometry_update(): Custom odometry update", 'error', 'loc')
        self.pre_update(timestamp)

        super(CustomEKF, self).update(
            asarray([position.x, position.y, position.z, yaw]),
            lambda _: self.observation_matrix,
            self.hx_custom_odometry,
            R
        )

    def pedometer_update(self, position: Coordinates, yaw: float, timestamp: float) -> None:
        self.pre_update(timestamp)

        super(CustomEKF, self).update(asarray([position.x, position.y, position.z, yaw]),
                                      lambda _: self.observation_matrix,
                                      self.hx_pedometer, self.R_pedometer)

    def incorporate_ranging_data(self, timestamp: float, anchors_ranging_data:list[tuple[int, int]], tags_ranging_data:list[tuple], raw_yaw:float):
        """
        This function is called for UpdateType.RANGING updates 
        It determines if enough anchor ranges are passed for a trilateration update
        and if not, makes multiple simple ranging updates with individual elements. 
        """
        if len(anchors_ranging_data)>=3: # Enough anchors for trilateration update, trilaterate position and update EKF 
            anchor_pos = [] 
            anchor_dist = [] 
            for id, dist in anchors_ranging_data: 
                anchor_pos.append(anchors[id].data) # .data to extract the list version of the Coordinates object 
                anchor_dist.append(dist) 
            # Residual function (Error = Calculated Distance - Measured Distance)
            def equations(position):
                calculated_distances =linalg.norm(anchor_pos - position, axis=1)
                return calculated_distances - anchor_dist
            # Solving with Non-linear Least Squares (Levenberg-Marquardt)
            raw_pos = least_squares(equations, array(self.get_position().data), method='lm')
            self.trilateration_update(Coordinates(raw_pos.x[0], raw_pos.x[1], raw_pos.x[2]), raw_yaw, timestamp)

        else: # Not enough anchors for trilateration; add multiple ranging updates 
            print("ADDING RANGE UPDATES", 'ok', 'loc')
            print(f"{anchors_ranging_data}", 'ok', 'loc')
            print(f"{tags_ranging_data}", 'ok', 'loc')
            raise ValueError("uninplemented, need to first ensure anchors_ranging_data and tags_ranging data are joined correctly.")
            for target_position, distance in anchors_ranging_data+tags_ranging_data: 
                formatted_distance = Coordinates(distance, 0, 0)
                formatted_target_pos = array([[target_position.x, target_position.y, target_position.z]])
                # NOTE 2026-08-17, in the past, neighbor positions were casted with atleast_2d. 
                # If conversion problems arise, try to add it to formatted_target_pos / see jacobian methods 
                self.ranging_update(formatted_distance, raw_yaw, timestamp, formatted_target_pos)

    def trilateration_update(self, position: Coordinates, yaw: float, timestamp: float) -> None:
        self.pre_update(timestamp)

        super(CustomEKF, self).update(asarray([position.x, position.y, position.z, yaw]),
                                      lambda _: self.observation_matrix,
                                      self.hx_trilateration, self.R_trilateration)

    def ranging_update(self, distance: Coordinates, yaw: float, timestamp: float, neighbor_position: ndarray) -> None:
        self.pre_update(timestamp)

        super(CustomEKF, self).update(asarray([distance.x, distance.y, distance.z, yaw]),
                                      self.h_ranging, self.hx_ranging, self.R_ranging,
                                      args=neighbor_position,
                                      hx_args=(neighbor_position, yaw))

    def zero_movement_update(self, position: Coordinates, yaw: float, timestamp: float) -> None:
        """This function updates the filter with its previous state.
        This allows to keep the dt relatively small and avoid drift.
        Indeed, if dt is too big, the process noise increase even if there was no change to the state."""

        self.pre_update(timestamp)

        super(CustomEKF, self).update(asarray([position.x, position.y, position.z, yaw]),
                                      lambda _: self.observation_matrix,
                                      self.hx_zero_movement, self.R_zero_movement)

    def add_custom_odometry(self, custom_odometry: CustomOdometry):
        self.custom_R.append(array(custom_odometry.get_R()))