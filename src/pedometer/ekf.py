from filterpy.kalman import ExtendedKalmanFilter
from numpy import array, asarray, dot, eye
from pypozyx import Coordinates


class PedometerEKF(ExtendedKalmanFilter):
    def __init__(self, position: Coordinates):
        super(PedometerEKF, self).__init__(dim_x=6, dim_z=3)

        self.dt = 0.1
        self.set_qf()
        self.ps = []
        self.R = array([[15, 0, 0],
                        [0, 15, 0],
                        [0, 0, 15]])

        # compute Jacobian of H matrix
        self.h_of_position = array([[1, 0, 0, 0, 0, 0],
                                    [0, 0, 1, 0, 0, 0],
                                    [0, 0, 0, 0, 1, 0]])

        self.x = array([position.x / 10, 0, position.y / 10, 0, position.z / 10, 0])

    def set_qf(self):
        self.Q = array([[0, 0, 0, 0, 0, 0],
                        [0, self.dt / 10, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, self.dt / 10, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, self.dt / 10]])

        self.F = eye(6) + array([[0, self.dt, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, self.dt, 0, 0],
                                 [0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, self.dt],
                                 [0, 0, 0, 0, 0, 0]])

    def hx_of_position(self, x):
        """ takes a state variable and returns the measurements that would
        correspond to that state."""
        return dot(self.h_of_position, x)

    def pre_update(self, dt):
        if dt != self.dt:
            self.dt = dt
            self.set_qf()
        self.predict()

    def update_position(self, position, time_between_steps):
        self.pre_update(time_between_steps)
        super(PedometerEKF, self).update(asarray([position.x, position.y, position.z]),
                                         lambda _: self.h_of_position, self.hx_of_position, self.R)
