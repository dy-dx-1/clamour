import sys
from rich.traceback import install ; install() # For clearer traceback printing 

from clamour import Clamour, PoseMessage, ContextManagedQueue
from clamour.custom_terminal import print

def onNewPoseEstimated(poseMsg: PoseMessage):
    print(text=f"Pose estimated: x: {poseMsg.x}, y: {poseMsg.y}, z: {poseMsg.z}, yaw: {poseMsg.yaw}", status='info', type='loc')

def main(): 
# An argument of anything else than 0 sets debug to True.
    sound = False
    if len(sys.argv) > 1:
        sound = bool(int(sys.argv[1]))
        
    communication_queue = ContextManagedQueue() # ContextManagedQueue is an extension of a multiprocessing.Queue() with a max size of 20

    clamour = Clamour([])
    clamour.start(sound, onNewPoseEstimated, communication_queue)

if __name__ == "__main__":
    main() 