from tdmaNode import TDMANode


def main():
    with TDMANode() as node:
        node.run()


if __name__ == "__main__":
    main()
