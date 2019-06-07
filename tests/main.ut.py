import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import unittest
from testInterface import anchors, neighborhood, slotAssignment, timing
from testMessages import (messageBox, messageFactory, synchronizationMessage,
                          tdmaControlMessage, uwbMessage)
from testOther import logicalClock, ekf


def main():
    suite = unittest.TestSuite()
    loader = unittest.TestLoader()
    modules = [anchors, neighborhood, slotAssignment, timing,
               messageBox, messageFactory, synchronizationMessage,
               tdmaControlMessage, uwbMessage, logicalClock, ekf]

    for module in modules:
        suite.addTest(loader.loadTestsFromModule(module))
    
    unittest.TextTestRunner(verbosity=3).run(suite)


if __name__ == "__main__":
    main()