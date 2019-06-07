from filterpy.kalman import ExtendedKalmanFilter
from numpy import array, asarray, dot, eye, linalg
from pypozyx import Coordinates


class CustomEKF(ExtendedKalmanFilter):
    def __init__(self, position: Coordinates):
        super(CustomEKF, self).__init__(dim_x=9, dim_z=6)

        self.dt = 0.1
        self.set_qf()
        self.ps = []
        self.zone_limits = [[0, 800, -100, 600]]
        self.R = array([[15, 0, 0, 0, 0, 0],
                        [0, 15, 0, 0, 0, 0],
                        [0, 0, 15, 0, 0, 0],
                        [0, 0, 0, 40, 0, 0],
                        [0, 0, 0, 0, 40, 0],
                        [0, 0, 0, 0, 0, 40]])

        self.RRange = array([[2, 0, 0, 0, 0, 0],
                             [0, 2, 0, 0, 0, 0],
                             [0, 0, 2, 0, 0, 0],
                             [0, 0, 0, 40, 0, 0],
                             [0, 0, 0, 0, 40, 0],
                             [0, 0, 0, 0, 0, 40]])

        # compute Jacobian of H matrix
        self.h_of_position = array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, 0, 0],
                                    [0, 0, 1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1]])
        
        self.x = array([position.x/10, 0, 0, position.y/10, 0, 0, position.z/10, 0, 0])

    def set_qf(self):
        self.Q = array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, self.dt/10, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, self.dt, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, self.dt/10, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, self.dt, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, self.dt/10, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, self.dt]])

        self.F = eye(9) + array([[0, self.dt, self.dt*self.dt, 0, 0, 0, 0, 0, 0],
                                 [0, 0, self.dt, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, self.dt, self.dt*self.dt, 0, 0, 0],
                                 [0, 0, 0, 0, 0, self.dt, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, self.dt, self.dt*self.dt],
                                 [0, 0, 0, 0, 0, 0, 0, 0, self.dt],
                                 [0, 0, 0, 0, 0, 0, 0, 0, 0]])

    def hx_of_position(self, x):
        """ takes a state variable and returns the measurements that would
        correspond to that state."""
        return dot(self.h_of_position, x)

    @staticmethod
    def h_of_range(x, nei_pose) -> array:
        """ compute Jacobian of H matrix for state x """
        num_nei = nei_pose.shape
        deltas = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        for i in range(3):
            if num_nei[0] > i:
                norm = linalg.norm([x[0]-nei_pose[i][0], x[3]-nei_pose[i][1], x[6]-nei_pose[i][2]])
                for j in range(3):
                    deltas[j] = (x[j*3]-nei_pose[i][j])/norm

        return array([[deltas[0], 0, 0, deltas[1], 0, 0, deltas[2], 0, 0],
                      [deltas[3], 0, 0, deltas[4], 0, 0, deltas[5], 0, 0],
                      [deltas[6], 0, 0, deltas[7], 0, 0, deltas[8], 0, 0],
                      [0, 0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1]])

    @staticmethod
    def hx_of_range(x, nei_pose):
        """ takes a state variable and returns the measurements that would
        correspond to that state."""
        nb_neighbors = nei_pose.shape[0]
        hx_out = [0, 0, 0, x[2], x[5], x[8]]

        for i in range(3):
            if nb_neighbors[0] > i:
                hx_out[i] = linalg.norm([x[0]-nei_pose[i][0], x[3]-nei_pose[i][1], x[6]-nei_pose[i][2]])

        return hx_out

    def pre_update(self, dt):
        if dt != self.dt:
            self.dt = dt
            self.set_qf()
        self.predict()

    def update_position(self, position, acceleration, dt):
        self.pre_update(dt)
        super(CustomEKF, self).update(asarray([position[0], position[1], position[2],
                                               acceleration.x/10, acceleration.y/10, acceleration.z/10]),
                                      lambda _: self.h_of_position, self.hx_of_position, self.R)

    def update_range(self, new_range, nei_pose, acceleration, dt):
        self.pre_update(dt)
        super(CustomEKF, self).update(asarray([new_range[0], new_range[1], new_range[2],
                                               acceleration.x/10, acceleration.y/10, acceleration.z/10]),
                                      self.h_of_range, self.hx_of_range, self.RRange, args=nei_pose, hx_args=nei_pose)

    def check_zone(self, zone):
        if self.x[0] < self.zone_limits[zone][0]:
            self.x[0] = self.zone_limits[zone][0]

        if self.x[0] > self.zone_limits[zone][1]:
            self.x[0] = self.zone_limits[zone][1]

        if self.x[3] < self.zone_limits[zone][2]:
            self.x[3] = self.zone_limits[zone][2]

        if self.x[3] > self.zone_limits[zone][3]:
            self.x[3] = self.zone_limits[zone][3]
