import sys

#from rich.traceback import install as install_rich_traceback

from clamour.clamour import Clamour
from clamour.contextManagedQueue import ContextManagedQueue
from clamour.custom_terminal import print
from clamour.messages import PoseMessage


def on_new_pose_estimated(pose_msg: PoseMessage) -> None:
    print(
        text=(
            f"Pose estimated: x: {pose_msg.x}, y: {pose_msg.y}, "
            f"z: {pose_msg.z}, yaw: {pose_msg.yaw}"
        ),
        status="info",
        type="loc",
    )


def main() -> None:
    #install_rich_traceback()  # Display readable tracebacks for this CLI entry point.

    sound = False
    if len(sys.argv) > 1:
        sound = bool(int(sys.argv[1]))

    communication_queue = ContextManagedQueue()
    clamour = Clamour([])
    clamour.start(sound, on_new_pose_estimated, communication_queue)


if __name__ == "__main__":
    main()
