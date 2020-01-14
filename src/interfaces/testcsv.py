from pypozyx import DeviceCoordinates, Coordinates
import csv, sys
from anchors import Anchors

def main(argv):
    # The different levels of context managers are required to ensure everything starts and stops cleanly.
    debug = int(argv[0]) # TODO link with ekfManager self.debug
    print("Starting everything, have a nice visit (", debug, ")")

    atest = Anchors()

if __name__ == "__main__":
    main(sys.argv[1:])