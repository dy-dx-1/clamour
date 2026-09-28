import sys

#from rich.traceback import install as install_rich_traceback

from clamour.clamour import Clamour
from clamour.contextManagedQueue import ContextManagedQueue
from clamour.custom_terminal import print
from clamour.interfaces import Pose


def on_new_pose_estimated(pose: Pose) -> None:
    print(
        text=(
            f"Pose estimated: x: {pose.x}, y: {pose.y}, "
            f"z: {pose.z}, yaw: {pose.heading}"
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
