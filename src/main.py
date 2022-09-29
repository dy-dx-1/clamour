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
        
    communication_queue = ContextManagedQueue()

    clamour = Clamour([])
    clamour.start(sound, onNewPoseEstimated, communication_queue)
