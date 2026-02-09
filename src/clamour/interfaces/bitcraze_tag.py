from .tag import Tag
class BitcrazeTag(Tag):
    def __init__(self, serial_port: str, tag_id: int):
        self.serial_port = serial_port
        self.tag_id = tag_id