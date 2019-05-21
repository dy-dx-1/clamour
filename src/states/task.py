from .constants import State
from .tdmaState import TDMAState

from enum import Enum


class LocalizationMethod(Enum):
    RANGING: 0
    POSITIONNING: 1


class Task(TDMAState):
    def __init__(self, timing, anchors, id, socket):
        self.timing = timing
        self.anchors = anchors
        self.id = id
        self.remote_id = 0
        self.localization_method = LocalizationMethod.RANGING
        self.done = False
        self.extended_kalman_filter = object()
        self.socket = socket
        self.dt = 0
        self.last_measurement_id = 0
        self.last_measurement_data = 0

    def execute(self):
        pass
        
    def next(self) -> State:
        pass
