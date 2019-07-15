from filterpy.kalman import ExtendedKalmanFilter
from math import atan2
from numpy import array, asarray, ndarray, dot, eye, linalg
from pypozyx import Coordinates


class CustomEKF(ExtendedKalmanFilter):
    def __init__(self, position: Coordinates, yaw: float):
        super(CustomEKF, self).__init__(dim_x=8, dim_z=4)

        self.dt = 0.1
        self.set_qf()
        self.R_pedometer = array([[20, 0, 0, 0],
                                  [0, 20, 0, 0],
                                  [0, 0, 20, 0],
                                  [0, 0, 0, 10]])

        self.R_trilateration = array([[15, 0, 0, 0],
                                      [0, 15, 0, 0],
                                      [0, 0, 15, 0],
                                      [0, 0, 0, 10]])

        self.R_ranging = array([[25, 0, 0, 0],
                                [0, 25, 0, 0],
                                [0, 0, 25, 0],
                                [0, 0, 0, 10]])

        self.observation_matrix = array([[1, 0, 0, 0, 0, 0, 0, 0],
                                         [0, 0, 1, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 1, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0, 1, 0]])

        self.x = array([position.x / 10, 0, position.y / 10, 0, position.z / 10, 0, yaw, 0])

    def set_qf(self):
        self.Q = array([[0, 0, 0, 0, 0, 0, 0, 0],
                        [0, self.dt / 10, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, self.dt / 10, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, self.dt / 10, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, self.dt / 10]])

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

    @staticmethod
    def hx_ranging(x, neighbor_positions: ndarray, yaw: float) -> ndarray:
        nb_neighbors = neighbor_positions.shape[0]
        hx = array([0, 0, 0, x[1], x[3], x[5], yaw, 0])

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
        deltas = [0, 0, 0, 0, 0, 0, 0, 0]

        for i in range(3):
            if num_nei[0] > i:
                norm = linalg.norm([x[0] - nei_pose[i][0], x[2] - nei_pose[i][1], x[4] - nei_pose[i][2]])
                for j in range(3):
                    deltas[i * 3 + j] = 0 if norm == 0 else (x[j * 2] - nei_pose[i][j]) / norm

        return array([[deltas[0], 0, deltas[1], 0, deltas[2], 0, 0, 0],
                      [deltas[3], 0, deltas[4], 0, deltas[5], 0, 0, 0],
                      [deltas[6], 0, deltas[7], 0, deltas[8], 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1, 0]])

    def pre_update(self, dt):
        if dt != self.dt:
            self.dt = dt
            self.set_qf()
        self.predict()

    def pedometer_update(self, position: Coordinates, yaw: float, delta_time: float):
        self.pre_update(delta_time)

        super(CustomEKF, self).update(asarray([position.x, position.y, position.z, yaw]),
                                      lambda _: self.observation_matrix,
                                      self.hx_pedometer, self.R_pedometer)

    def trilateration_update(self, position: Coordinates, yaw: float, delta_time: float):
        self.pre_update(delta_time)

        super(CustomEKF, self).update(asarray([position.x, position.y, position.y, yaw]),
                                      lambda _: self.observation_matrix,
                                      self.hx_trilateration, self.R_trilateration)

    def ranging_update(self, distance: Coordinates, yaw: float, delta_time: float, neighbor_position: ndarray):
        self.pre_update(delta_time)

        super(CustomEKF, self).update(asarray([distance.x, distance.y, distance.z, yaw]),
                                      self.h_ranging, self.hx_ranging, self.R_ranging,
                                      args=neighbor_position,
                                      hx_args=(neighbor_position, yaw))
