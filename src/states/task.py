from .constants import State
from .tdmaState import TDMAState

from ekf import CustomEKF
from interfaces import Timing, Anchors, Neighborhood
from interfaces.timing import tdmaExcStartTime, FrameLen, tdmaExcSlotLen

import random
import struct
from time import time
from socket import socket as Socket
from enum import Enum
from numpy import array, atleast_2d
from pypozyx import (PozyxSerial, LinearAcceleration, SingleRegister, DeviceList, 
                     DeviceRange, Coordinates,
                     POZYX_ANCHOR_SEL_AUTO, POZYX_DISCOVERY_ANCHORS_ONLY, 
                     POZYX_SUCCESS, POZYX_POS_ALG_UWB_ONLY, POZYX_3D)


class Task(TDMAState):
    def __init__(self, timing: Timing, anchors: Anchors, neighborhood: Neighborhood,
                 id: int, socket: Socket, pozyx: PozyxSerial):
        self.timing = timing
        self.anchors = anchors
        self.id = id
        self.remote_id = None
        self.localize = self.ranging
        self.done = False
        self.extended_kalman_filter = CustomEKF()
        self.socket = socket
        self.dt = 0
        self.last_measurement = [0, 0, 0]
        self.last_measurement_data = array([0, 0, 0], [0, 0, 0], [0, 0, 0])
        self.acceleration = LinearAcceleration()
        self.pozyx = pozyx
        self.neighborhood = neighborhood
        self.last_ekf_step_time = 0
        self.dimension = POZYX_3D
        self.height = 1000
        self.position = Coordinates()

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

    def positioning(self) -> int:
        status = self.pozyx.doPositioning(self.position, self.dimension, self.height, POZYX_POS_ALG_UWB_ONLY, remote_id=self.remote_id)
        scaled_position = [self.position.x/10, self.position.y/10, self.position.z/10]

        if status == POZYX_SUCCESS:
            self.dt = time() - self.last_ekf_step_time
            self.extended_kalman_filter.updatePose(scaled_position, self.acceleration, self.dt)
            self.last_ekf_step_time = time()
            self.extended_kalman_filter.dt = self.dt

        return status

    def ranging(self) -> int:
        ranging_target_id = self.select_ranging_target()
        device_range = DeviceRange()
        
        if ranging_target_id > 0:
            status = self.pozyx.doRanging(ranging_target_id, device_range)
        
        self.last_measurement = ([device_range.data[1]/10] + [0, 0])
        self.last_measurement_data = array([self.anchors.anchors_dict[self.id][2]/10,
                                            self.anchors.anchors_dict[self.id][3]/10, 
                                            self.anchors.anchors_dict[self.id][4]/10],
                                            [0, 0, 0], [0, 0, 0])
        
        if status == POZYX_SUCCESS:
            self.dt = time() - self.last_ekf_step_time
            self.extended_kalman_filter.updateRange(self.last_measurement, atleast_2d(self.last_measurement_data[0]), 
                                                    self.acceleration, int(self.dt * 100) / 100)
            self.last_ekf_step_time = time()
            self.extended_kalman_filter.dt = self.dt

        return status

    def select_ranging_target(self) -> int:
        """We select a target for doing a range measurement.
        Anchors are prioritized because of their lower uncertainty."""

        if len(self.anchors.available_anchors) > 0:
            return random.choice(self.anchors.available_anchors)
        elif len(self.neighborhood.current_neighbors) > 0:
            return random.choice(list(self.neighborhood.current_neighbors))
        else:
            return -1

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
                       self.last_measurement + list(self.last_measurement_data.flat) + \
                       [self.acceleration.x/10, self.acceleration/10, self.acceleration/10, 
                       self.timing.frame_id, self.timing.current_slot_id, positioning_result]
        
        message = struct.pack("%sf" % len(message_data), *message_data)
        self.socket.send(message)
