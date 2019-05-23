from .constants import State
from .tdmaState import TDMAState

from ekf import CustomEKF
from interfaces import Timing, Anchors
from interfaces.timing import tdmaExcStartTime, FrameLen, tdmaExcSlotLen

import struct
from socket import socket as Socket
from enum import Enum
from numpy import array
from pypozyx import (PozyxSerial, LinearAcceleration, SingleRegister, DeviceList,
                     POZYX_ANCHOR_SEL_AUTO, POZYX_DISCOVERY_ANCHORS_ONLY, POZYX_SUCCESS)


class Task(TDMAState):
    def __init__(self, timing: Timing, anchors: Anchors, 
                 id: int, socket: Socket, pozyx: PozyxSerial):
        self.timing = timing
        self.anchors = anchors
        self.id = id
        self.remote_id = 0
        self.localize = self.ranging
        self.done = False
        self.extended_kalman_filter = CustomEKF()
        self.socket = socket
        self.dt = 0
        self.last_measurement_id = [0, 0, 0]
        self.last_measurement_data = array([0, 0, 0], [0, 0, 0], [0, 0, 0])
        self.acceleration = LinearAcceleration()
        self.pozyx = pozyx

    def execute(self) -> State:
        if not self.anchors.discovery_done:
            self.discover_anchors()
            self.select_localization_method()
            self.set_anchors_manually()
            self.anchors.discovery_done = True
        else:
            self.broadcast_positioning_result(self.localize())

        return self.next()
        
    def next(self) -> State:
        if self.timing.current_time_in_cycle > (tdmaExcStartTime + self.timing.frame_id * FrameLen + (self.timing.current_slot_id + 1) * tdmaExcSlotLen):
            return State.LISTEN
        else:
            return State.TASK

    def select_localization_method(self) -> None:
        self.localize = self.positioning if self.anchors.available_anchors >= 4 else self.ranging

    def positioning(self):
        return self.ekf_handler.measure_position()

    def ranging(self):
        return self.ekf_handler.measure_range()

    def discover_anchors(self) -> None:
        self.pozyx.clearDevices()

        if self.pozyx.doDiscovery(POZYX_DISCOVERY_ANCHORS_ONLY, self.remote_id) == POZYX_SUCCESS:
            devices = self.get_devices()

            for device_id in devices:
                if device_id not in self.anchors.available_anchors:
                    self.anchors.available_anchors.append(device_id)
            
            self.anchors.discovery_done = True

    def get_devices(self) -> DeviceList:
        size = SingleRegister()
        status = self.pozyx.getDeviceListSize(size, self.remote_id)

        devices = DeviceList(list_size=size[0])
        status &= self.pozyx.getDeviceIds(devices, self.remote_id)

        return devices

    def set_anchors_manually(self) -> None:
        self.pozyx.clearDevices(self.remote_id)

        for anchor_id in self.anchors.available_anchors:
            self.pozyx.addDevice(self.anchors[anchor_id], self.remote_id)
        
        if len(self.anchors.available_anchors) > 4:
            self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors.available_anchors))


    def broadcast_positioning_result(self, positioning_result) -> None:
        message_data = [self.id - 99] + self.extended_kalman_filter.x.toList() + \
                       [self.localize, self.extended_kalman_filter.dt] + \
                       self.last_measurement_id + list(self.last_measurement_data.flat) + \
                       [self.acceleration.x/10, self.acceleration/10, self.acceleration/10, 
                       self.timing.frame_id, self.timing.current_slot_id, positioning_result]
        
        message = struct.pack("%sf" % len(message_data), *message_data)
        self.socket.send(message)
