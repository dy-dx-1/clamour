from clamour import Clamour, PoseMessage, CustomOdometry, ContextManagedQueue
import sys
import time

def onNewPoseEstimated(poseMsg: PoseMessage):
    print("New pose estimated: x:", poseMsg.x, ", y:", poseMsg.y, ", z:", poseMsg.z, ", yaw:", poseMsg.yaw)

if __name__ == "__main__":
    # An argument of anything else than 0 sets debug to True.
    sound = False
    if len(sys.argv) > 1:
        sound = bool(int(sys.argv[1]))

    imuOdometry = CustomOdometry(
        [[20,  0,  0,   0],
         [ 0, 20,  0,   0],
         [ 0,  0, 20,   0],
         [ 0,  0,  0, 0.5]]
    )

    clamour = Clamour([imuOdometry])
    clamour.start_non_blocking(sound, onNewPoseEstimated)

    while True:
        imuOdometry.update_pose(PoseMessage(1.0, 2.0, 3.0, 4.0))
        time.sleep(1)
