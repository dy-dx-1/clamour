from filterpy.kalman import ExtendedKalmanFilter
from numpy import array, asarray, ndarray, dot, eye
from pypozyx import Coordinates


class PedometerEKF(ExtendedKalmanFilter):
    def __init__(self, position: Coordinates):
        super(PedometerEKF, self).__init__(dim_x=8, dim_z=4)

        self.dt = 0.1
        self.set_qf()
        self.ps = []
        self.R_pedometer = array([[15, 0, 0, 0],
                                  [0, 15, 0, 0],
                                  [0, 0, 15, 0],
                                  [0, 0, 0, 40]])

        self.R_trilateration = array([[15, 0, 0, 0],
                                      [0, 15, 0, 0],
                                      [0, 0, 15, 0],
                                      [0, 0, 0, 40]])

        self.R_ranging = array([[15, 0, 0, 0],
                                [0, 15, 0, 0],
                                [0, 0, 15, 0],
                                [0, 0, 0, 40]])

        self.observation_matrix = array([[1, 0, 0, 0, 0, 0, 0, 0],
                                         [0, 0, 1, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 1, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0, 1, 0]])

        self.x = array([position.x / 10, 0, position.y / 10, 0, position.z / 10, 0, 0, 0])

    def set_qf(self):
        self.Q = array([[0, 0, 0, 0, 0, 0, 0, 0],
                        [0, self.dt / 10, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, self.dt / 10, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, self.dt / 10, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, self.dt / 5]])

        self.F = eye(8) + array([[0, self.dt, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, self.dt, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, self.dt, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, self.dt],
                                 [0, 0, 0, 0, 0, 0, 0, 0]])

    def hx_of_position(self, x):
        """ takes a state variable and returns the measurements that would
        correspond to that state."""
        return dot(self.observation_matrix, x)

    def pre_update(self, dt):
        if dt != self.dt:
            self.dt = dt
            self.set_qf()
        self.predict()

    def pedometer_update(self, position: Coordinates, yaw: float, delta_time: float):
        self.pre_update(delta_time)
        super(PedometerEKF, self).update(asarray([position.x, position.y, position.z, yaw]),
                                         lambda _: self.observation_matrix, self.hx_of_position, self.R_pedometer)

    def trilateration_update(self, position: Coordinates, delta_time: float):
        pass

    def ranging_update(self, distance: Coordinates, delta_time: float, neighbor_position: ndarray):
        pass
