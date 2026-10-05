import csv
import math 
from dataclasses import dataclass, field
from queue import Empty
from time import monotonic, sleep
from typing import Literal
from multiprocessing.synchronize import Lock

from .ekf import CustomEKF
from .factor_graph import FactorGraph
from ..custom_terminal import print 
from ..config import SAVE_TO_CSV
from ..interfaces import Tag, Pose
from ..contextManagedQueue import ContextManagedQueue
from ..messages.updateMessage import UpdateMessage
from ..messages.soundMessage import SoundMessage
from ..messages.types import UpdateType
from ..rooms import Floorplan

WAIT_TIME_DURING_INIT = 0.001 # s to wait at each iter while waiting for com queue to load message
# This is deliberately local while the fusion configuration is being introduced.  It
# is the cadence of *states*, not necessarily the IMU sample, range, or output rate.
STATE_INTERVAL = 0.1  # seconds


@dataclass
class SensorBatch:
    """Measurements collected for one future graph-state interval.

    ``boundary_time`` is on the estimator's monotonic clock.  Existing producers
    still emit wall-clock timestamps, so their timestamps must not be compared to
    it yet.  Until every producer is migrated to the common event-time clock, the
    coordinator groups received messages by ingress time only.  The message
    objects remain intact so their acquisition timestamps can be used by legacy
    estimator adapters where necessary.
    """
    start_time: float
    boundary_time: float
    messages: list[UpdateMessage] = field(default_factory=list)
    imu_samples: list = field(default_factory=list)

class StateEstimator: 
    """
    General interface for pose estimation. 
    Continuously runs the .run() method in it's own ContextManagedProcess.

    ARGS: 
    - tag: Tag object to track 
    - tag_lock: Multiprocessing Lock for the tag
    - estimator_type: EKF or Factor Graph
    - pose_callback: Function that takes a Pose and prints it on pose update # TODO remove? just put inside?
    - communication_queue: Queue where pose updates / messages to process appear 
    - sound_queue: Optional, if a Queue is passed, will send updates to it to allow for sound playing 
    """
    def __init__(self, tag: Tag, tag_lock: Lock,
                  estimator_type: Literal['EKF', 'FG'], pose_callback, 
                  communication_queue: ContextManagedQueue, sound_queue: None|ContextManagedQueue):
        self.tag = tag 
        self.tag_lock = tag_lock 

        self.estimator = None 
        self.estimator_type = estimator_type # validity check done in config  

        self.yaw_offset = 0  # Measured in degrees relative to global coordinates X-Axis
        self.last_know_neighbors = {}
        
        self.pose_callback = pose_callback # NOTE future eval if can just put this in here, idk why need to pass it as arg 

        self.sound_queue = sound_queue
        self.com_queue = communication_queue
        
        # The coordinator is the sole owner of these buffers.  Sensor processes
        # only publish immutable, timestamped measurements into input queues.
        self._pending_messages: list[UpdateMessage] = []
        self._last_state_boundary: float | None = None
        self.state_csv, self.writer = self.initialize_csv()

        self.floorplan = Floorplan() # NOTE TODO: currently unused 
        self.current_room = self.floorplan.rooms['24'] 

    def run(self) -> None: 
        try: 
            self.initialize_estimator()
            self._run_sensor_batching_loop()
        except Exception as e: 
            print(f'State Estimator crashed! Error: {str(e)}', 'error', 'loc')
            raise e

    def _run_sensor_batching_loop(self) -> None:
        """Close fixed-duration sensor intervals and estimate one state per interval.

        This is intentionally the only long-running fusion loop.  It replaces the
        old "one queue message equals one estimator update" loop and never treats
        an empty queue as evidence that the tag is stationary.  IMU collection is
        represented by ``SensorBatch.imu_samples`` for now; an IMU source will be
        connected here once its common-clock timestamp conversion is available.
        """
        self._last_state_boundary = monotonic()
        next_boundary = self._last_state_boundary + STATE_INTERVAL

        while True:
            self._drain_communication_queue()
            now = monotonic()
            if now < next_boundary:
                # Keep latency low without using a repeated fixed sleep, which
                # would accumulate scheduler drift over a long run.
                sleep(min(WAIT_TIME_DURING_INIT, next_boundary - now))
                continue

            # Drain once more so messages that arrived at the boundary are not
            # unnecessarily delayed by a complete state interval.
            self._drain_communication_queue()
            batch = SensorBatch(
                start_time=self._last_state_boundary,
                boundary_time=next_boundary,
                messages=self._pending_messages,
                # IMU samples will be drained from the IMU FIFO here, not placed
                # onto the general communication queue at IMU sample rate.
                imu_samples=[],
            )
            self._pending_messages = []
            self.process_sensor_batch(batch)

            self._last_state_boundary = next_boundary
            # Advance from the previous deadline rather than from ``now``.  When
            # processing overruns, the following iterations catch up by closing
            # the missed state intervals instead of permanently shifting cadence.
            next_boundary += STATE_INTERVAL

    def _drain_communication_queue(self) -> None:
        """Move all currently available producer events into coordinator ownership."""
        while True:
            try:
                self._pending_messages.append(UpdateMessage.load(*self.com_queue.get_nowait()))
            except Empty:
                return

    def process_sensor_batch(self, batch: SensorBatch) -> None:
        """Skeleton for one state transition in the new fusion architecture.

        The eventual order is deliberate: preintegrate IMU, create the new state
        and its motion factor, add interval observations, then optimise once.  The
        existing estimator implementations cannot yet perform that whole contract,
        so the range block below is a temporary compatibility adapter.  It batches
        ranges into one legacy backend call while leaving step and pose events out
        of the old pseudo-measurement paths.
        """
        # 1. IMU: drain and preintegrate every sample in (start, boundary] exactly
        # once.  This belongs to the estimator/coordinator, not the range producer.
        # self._preintegrate_imu(batch.imu_samples, batch.start_time, batch.boundary_time)

        # 2. Non-geometric metadata does not create a state or a measurement.
        for message in batch.messages:
            if message.update_type == UpdateType.TOPOLOGY:
                self.update_neighbors(message.topology)

        range_messages = [
            message for message in batch.messages
            if message.update_type == UpdateType.RANGING
        ]
        step_messages = [
            message for message in batch.messages
            if message.update_type == UpdateType.PEDOMETER
        ]

        # 3. Create a state and its motion link.  Future FactorGraph support will
        # use IMU preintegration when available, otherwise its explicit
        # constant-velocity/random-walk factor.  No synthetic zero-motion factor
        # belongs here.
        # self.estimator.advance_to(batch.boundary_time, preintegrated_imu=...)

        # 4. Add all range observations for the interval, then optimise once.  The
        # temporary adapters expose the intended batching behaviour to the EKF and
        # current factor-graph APIs; the final API will accept a SensorBatch.
        if range_messages:
            self._apply_batched_ranges(range_messages)

        # A step must become a relative step factor between timestamp-bracketing
        # states.  Do not route it through pedometer_yaw_to_coords(): that method
        # manufactures an absolute observation from the current estimate.
        if step_messages:
            self._queue_step_constraints(step_messages, batch)

        # CustomOdometry is intentionally not forwarded here.  It will either be
        # removed with the retired Cognifly integration or reintroduced later as a
        # generic, explicitly framed PoseObservation factor.

        # 5. Optimise and publish at the state cadence.  Current backends optimise
        # inside _apply_batched_ranges; a no-range interval therefore publishes the
        # latest posterior until their explicit advance_to() contract is added.
        self.publish_state(range_messages[-1] if range_messages else None)
        self._publish_sound()

    def _apply_batched_ranges(self, messages: list[UpdateMessage]) -> None:
        """Temporary adapter from interval range batches to the legacy API."""
        anchors_ranging = []
        tags_ranging = []
        for message in messages:
            self.update_neighbors(message.topology)
            anchors_ranging.extend(message.anchors_ranging_data or [])
            tags_ranging.extend(message.tags_ranging_data or [])

        # Preserve legacy timestamp semantics until all producers use estimator
        # event time.  The final batch API will receive individual range times.
        latest = messages[-1]
        self.estimator.incorporate_ranging_data(
            latest.timestamp,
            anchors_ranging,
            tags_ranging,
            latest.measured_yaw,
        )

    def _queue_step_constraints(self, messages: list[UpdateMessage], batch: SensorBatch) -> None:
        """Extension point for timestamped relative pedometer factors.

        Step events may be delivered after their physical event time.  The final
        implementation will place each one between retained graph states using
        that event time, rather than treating it as a current absolute pose.
        """
        # TODO: convert the pedometer producer to emit a StepEvent with peak time,
        # stride/heading uncertainty, and a common-clock timestamp.
        return

    def _publish_sound(self) -> None:
        if self.sound_queue is not None:
            self.sound_queue.put(SoundMessage.save(SoundMessage(self.estimator.pose)))

    def run_legacy_message_loop(self) -> None:
        """Deprecated compatibility loop; retained temporarily for diagnosis only."""
        while True:
            self.process_latest_state_info()

    def initialize_estimator(self) -> None: 
        """
        Wait for a trilateration-sufficient update to arrive in communication queue. Use it to init estimator. 
        Need to start from a fully constrained position to lock in the global reference frame.
        """
        while self.estimator is None: 
            if not self.com_queue.empty():
                msg = UpdateMessage.load(*self.com_queue.get_nowait())
                if msg.update_type == UpdateType.RANGING: 
                    if len(msg.anchors_ranging_data)<3:  # Need a fully constrained measurement for initialization
                        continue 
                    self.yaw_offset = msg.measured_yaw  # Store initial value, which we'll use to correct further poses  
                    raw_yaw = self.correct_yaw(msg.measured_yaw)
                    
                    if self.estimator_type == 'EKF': 
                        self.estimator = CustomEKF(msg.anchors_ranging_data, raw_yaw)
                        # For the EKF, incorporating ranging data with >3 anchors will directly trigger a trilateration update
                        self.estimator.incorporate_ranging_data(msg.timestamp, msg.anchors_ranging_data, msg.tags_ranging_data, raw_yaw)
                    elif self.estimator_type == 'FG':
                        self.estimator = FactorGraph(msg.anchors_ranging_data, raw_yaw, msg.timestamp)
                        # Not calling incorporate_ranging_data yet, as need to update timestamp first 
                        # however should add a way to set factors in init as using directly incorporate_... will add a BetweenFactor
                        # separate adding functions inside incorporate and just put the ones adding the anchors and tags inside of the init part? 

                    # Estimator initialized. Internalise and publish the posterior.  
                    self.publish_state(msg) 
            else:
                sleep(WAIT_TIME_DURING_INIT)  
        print(f"ESTIMATOR ({self.estimator_type}) INITIALIZATION DONE", 'ok', 'loc')

    def process_latest_state_info(self): 
        """
        Get and process an update through the communication queue.
        Updates the estimator, saves and prints the current measurement.  
        """
        if not self.com_queue.empty(): 
            msg = UpdateMessage.load(*self.com_queue.get_nowait())
            ts, anchors_ranging, tags_ranging, raw_yaw = msg.timestamp, msg.anchors_ranging_data, msg.tags_ranging_data, msg.measured_yaw
            # TODO add an in-bounds of the room check somewhere 
            match msg.update_type: 
                case UpdateType.PEDOMETER: 
                    self.estimator.pedometer_update(self.pedometer_yaw_to_coords(msg.measured_yaw), raw_yaw, ts) 
                case UpdateType.RANGING: 
                    self.update_neighbors(msg.topology) 
                    self.estimator.incorporate_ranging_data(ts, anchors_ranging, tags_ranging, raw_yaw) 
                case UpdateType.TOPOLOGY:  
                    self.update_neighbors(msg.topology) 
                case UpdateType.CUSTOM_POSE: # TODO remove / replace by IMU factor? 
                    self.estimator.custom_odometry_update(Pose(msg.pose.x, msg.pose.y, msg.pose.z), msg.pose.heading, msg.R, msg.timestamp)
            
            self.publish_state(msg) 

            if self.sound_queue != None: 
                sound_message = SoundMessage(self.estimator.pose)
                self.sound_queue.put(SoundMessage.save(sound_message))

        else: 
            # Queue idleness is not a stationary measurement.  The cadence-driven
            # coordinator now owns prediction and any future explicit ZUPT logic.
            sleep(WAIT_TIME_DURING_INIT) 

    def update_neighbors(self, neighbors: dict):
        self.last_know_neighbors = neighbors

    def pedometer_yaw_to_coords(self, measured_yaw: float) -> Pose:
        """When new information arrives from the pedometer, it is in the form of a yaw and timestamp.
        Since the step length is constant, we can infer cartesian coordinates from yaw and last know position."""

        step_length = 75  # centimeters

        delta_position_x = step_length * -math.cos(math.radians(self.correct_yaw(measured_yaw)))
        delta_position_y = step_length * math.sin(math.radians(self.correct_yaw(measured_yaw)))

        # The pedometer cannot measure height; we assumed it is constant.
        return Pose(self.estimator.x[0] + delta_position_x, self.estimator.x[2] + delta_position_y, self.estimator.x[4])

    def correct_yaw(self, measured_yaw: float) -> float:
        """
        The initial yaw that is measured is '0', subsequent ones need to be corrected to stay consistent. 
        """
        new_yaw = measured_yaw - self.yaw_offset
        return new_yaw if new_yaw >= 0 else 360 + new_yaw 

    def validate_new_state(self, new_pose: Pose) -> bool:
        """
        Makes sure the proposed coordinates stay within the same room or a logically accessible room.
        TODO NOTE: This is currently unused, previously, there was a commented check in process_latest_state_info
        that used to do a zero mvt update if out of bounds. In the future, evaluate if can still do something interesting
        with this information. 
        2026-08-12 
        """
        if self.current_room.within_bounds(new_pose):
            return True

        new_neighbor = self.current_room.within_neighbor_bounds(new_pose, self.floorplan.rooms)
        if new_neighbor is not None:
            print("Changed room.", 'info', 'loc')
            self.current_room = self.floorplan.rooms[new_neighbor]
            return True

        return False

    def publish_state(self, message: UpdateMessage | None):
        """
        - Saves the pose and covariance in its Tag object
        - Prints out the current posterior from the estimator
        - Saves to CSV if configured to do so (config.py) 
        """
        pose = self.estimator.pose

        with self.tag_lock:
            self.tag.pose = pose
 
        self.pose_callback(pose)

        if SAVE_TO_CSV and message is not None:
            self.save_to_csv(message)

    @staticmethod 
    def initialize_csv(): 
        if not SAVE_TO_CSV: 
            return None, None 
        filepath = 'pose_estimation.csv'
        fieldnames = ['tag_id', 'timestamp', 'synchronized_clock', 'offset', 'update_type',
                      'estimator_x', 'estimator_y', 'estimator_z', 'estimator_yaw', 
                      'covariance_matrix', 'slots', 'two_hop_neighbors']

        state_csv = open(filepath, 'w')
        writer = csv.DictWriter(state_csv, delimiter=',', fieldnames=fieldnames)
        writer.writeheader()

        return state_csv, writer
    
    def save_to_csv(self, message: UpdateMessage):
        """
        NOTE: Previously, this function also printed the raw position pre-EKF. During the move to FGs, I've removed this option. 
        This is because we now directly deal with multiple ranges, so we'd either need to:
        - Add a function inside the estimator that returns the 'non optimized' geometric value 
        - Add a function on here that quickly computes an estimation to generate raw values (not ideal, as we'd be doing the calc twice)
        ALSO: would need to figure out how to consider the IMU effect on the 'raw pos'. Do we incorporate it? Or do we just print out geometric results?
        What would be the benefit?

        Since the answer to these questions may be clearer in the future, leaving it without raw position for now (2026-08-18). 
        """
        if message.update_type == UpdateType.RANGING: # TODO add compatibility to IMU in future 
            pose = self.estimator.pose
            csv_data = {
                'tag_id': self.tag.tag_id,
                'timestamp': message.timestamp,
                'synchronized_clock': message.synchronized_clock,
                'offset': message.offset,
                'update_type': message.update_type,
                'estimator_x': pose.x,
                'estimator_y': pose.y,
                'estimator_z': pose.z,
                'estimator_yaw': pose.heading,
                'covariance_matrix': "to implement", # previously: np.linalg.det(self.estimator.P)
                'slots': message.slots,
                'two_hop_neighbors': self.last_know_neighbors
            }

            self.writer.writerow(csv_data)
            self.state_csv.flush()
