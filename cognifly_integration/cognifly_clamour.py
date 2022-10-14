from clamour import Clamour, CustomOdometry, PoseMessage
from cognifly.cognifly_controller.cognifly_controller import CogniflyController, PoseEstimator

class ClamourPoseEstimator(PoseEstimator):
    def __init__(self):
        self.last_x = 0
        self.current_x = 0
        self.velocity_x = 0

        self.last_y = 0
        self.current_y = 0
        self.velocity_x = 0

        self.last_z = 0
        self.current_z = 0
        self.velocity_x = 0

        self.last_yaw = 0
        self.current_yaw = 0
        self.velocity_x = 0

        self.fcOdometry = CustomOdometry(
            [[20,  0,  0,   0],
             [ 0, 20,  0,   0],
             [ 0,  0, 20,   0],
             [ 0,  0,  0, 0.5]]
        )
        self.clamour = Clamour([self.fcOdometry])
        self.clamour.start_non_blocking(False, self._on_new_pose)

    def get(self):
        est_x, est_y, est_z, est_yaw, est_vx, est_vy, est_vz, est_w = self.get_fc_estimate()
        self._updateFlightControllerOdometry(est_x, est_y, est_z, est_yaw)
        return self._get_last_calculated_output()
    
    def _updateFlightControllerOdometry(self, x, y, z, yaw):
        self.fcOdometry.update_pose(PoseMessage(x, y, z, yaw))

    def _get_last_calculated_output(self):
        return (
            self.current_x,
            self.current_y,
            self.current_z,
            self.current_yaw,
            self.velocity_x,
            self.velocity_y,
            self.velocity_z,
            self.velocity_yaw
        )

    def _on_new_pose(self, pose: PoseMessage):
        self.current_x = pose.x
        self.velocity_x = self.current_x - self.last_x
        self.last_x = self.current_x

        self.current_y = pose.y
        self.velocity_y = self.current_y - self.last_y
        self.last_y = self.current_y

        self.current_z = pose.z
        self.velocity_z = self.current_z - self.last_z
        self.last_z = self.current_z

        self.current_yaw = pose.yaw
        self.velocity_yaw = self.current_yaw - self.last_yaw
        self.last_yaw = self.current_yaw


if __name__ == '__main__':
    estimator = ClamourPoseEstimator()
    
    controller = CogniflyController(print_screen=False,
                            pose_estimator=estimator,
                            trace_logs=False,  # set to True to save PID logs
                            vel_x_kp=750.0,  # proportional gain for X velocity
                            vel_x_ki=200.0,  # integral gain for X velocity
                            vel_x_kd=10.0,  # derivative gain for X velocity
                            vel_y_kp=750.0,  # proportional gain for Y velocity
                            vel_y_ki=200.0,  # integral gain for Y velocity
                            vel_y_kd=10.0,  # derivative gain for Y velocity
                            vel_z_kp=5.0,  # proportional gain for Z velocity
                            vel_z_ki=2.0,  # integral gain for Z velocity
                            vel_z_kd=0.05,  # derivative gain for Z velocity
                            vel_w_kp=75.0,  # proportional gain for yaw rate
                            vel_w_ki=50.0,  # integral gain for yaw rate
                            vel_w_kd=0.0,  # derivative gain for yaw rate
                            pid_limit=400,  # PID clipping value, should not exceed 500
                            x_vel_gain=0.5,  # gain from X vector to X velocity (position control)
                            y_vel_gain=0.5,  # gain from Y vector to Y velocity (position control)
                            z_vel_gain=0.2)  # gain from Z vector to Z velocity (position control)
    
    controller.run_curses()