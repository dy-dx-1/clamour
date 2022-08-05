from clamour import Clamour, PoseMessage, ContextManagedQueue
import sys
import time

def onNewPoseEstimated(poseMsg: PoseMessage):
    print("New pose estimated: x:", poseMsg.x, ", y:", poseMsg.y, ", z:", poseMsg.z, ", yaw:", poseMsg.yaw)

if __name__ == "__main__":
    # An argument of anything else than 0 sets debug to True.
    sound = False
    if len(sys.argv) > 1:
        sound = bool(int(sys.argv[1]))

    clamour = Clamour()
    clamour.start_non_blocking(sound, onNewPoseEstimated)

    while True:
        print("Doing other stuff")
        time.sleep(1)
