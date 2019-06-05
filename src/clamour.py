import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

from .tdmaNode import TDMANode


def main():
    with TDMANode() as node:
        node.run()


if __name__ == "__main__":
    main()
