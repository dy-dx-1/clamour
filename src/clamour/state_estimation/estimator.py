import csv
from collections import deque
from dataclasses import dataclass, field
from math import ceil
from queue import Empty
from time import sleep
from typing import Literal
from multiprocessing.synchronize import Lock

from .ekf import CustomEKF
from .factor_graph import FactorGraph
from ..estimator_clock import EstimatorClock, SensorClockMapper
from ..custom_terminal import print 
from ..config import SAVE_TO_CSV
from ..interfaces import IMU, Tag, Pose
from ..contextManagedQueue import ContextManagedQueue
from ..messages.updateMessage import UpdateMessage
from ..messages.soundMessage import SoundMessage
from ..messages.types import UpdateType
from ..rooms import Floorplan

# This is deliberately local while the fusion configuration is being introduced.  It
# is the cadence of *states*, not necessarily the IMU sample, range, or output rate.
STATE_INTERVAL = 0.1  # seconds
STATE_INTERVAL_NS = round(STATE_INTERVAL * 1_000_000_000)
# The IMU -> EstimatorClock mapping must be refreshed periodically to correct drift 
IMU_CLOCK_REFRESH_INTERVAL = 1.0  # seconds; ten times the state interval
IMU_CLOCK_REFRESH_INTERVAL_NS = round(IMU_CLOCK_REFRESH_INTERVAL * 1_000_000_000)
# FIFO emptying safeties - we empty the FIFO before the limit to avoid losing info
# poll_interval = SF × time_to_overflow
# time_to_overflow = (capacity_words - watermark_words) / fifo_word_rate
# fifo_word_rate = sample_rate_hz × IMU_FIFO_WORST_CASE_WORDS_PER_SAMPLE
IMU_FIFO_WATERMARK_FRACTION = 0.70
IMU_FIFO_WORST_CASE_WORDS_PER_SAMPLE = 3 # Words per grouped FIFO sample: ex: accel, gyro, timestamp 
IMU_FIFO_POLL_SAFETY_FACTOR = 0.5       

@dataclass(frozen=True)
class TimedIMUSample:
    """IMU sample on the shared EstimatorClock time"""
    timestamp_ns: int
    acceleration: object | None
    angular_velocity: object | None

@dataclass
class SensorBatch:
    """
    Measurements collected for one future graph-state interval.

    Boundary and measurement times are integer nanoseconds on the shared estimator clock
    """
    start_time_ns: int
    boundary_time_ns: int
    messages: list[UpdateMessage] = field(default_factory=list)
    imu_samples: list[TimedIMUSample] = field(default_factory=list)

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
    def __init__(self, tag: Tag, tag_lock: Lock, imu: IMU|None,
                  estimator_clock: EstimatorClock, estimator_type: Literal['EKF', 'FG'], 
                  pose_callback, communication_queue: ContextManagedQueue, sound_queue: None|ContextManagedQueue):
        self.tag = tag 
        self.tag_lock = tag_lock 

        self.estimator = None 
        self.estimator_type = estimator_type # validity check done in config  

        self.yaw_offset = 0  # Measured in degrees relative to global coordinates X-Axis
        self.last_known_neighbors = {}

        self.pose_callback = pose_callback # NOTE future eval if can just put this in here, idk why need to pass it as arg

        self.sound_queue = sound_queue
        self.com_queue = communication_queue

        # Clamour shares this immutable epoch with all local producer processes.
        self.estimator_clock = estimator_clock
        self.imu = imu
        if imu is not None:
            self.imu_clock_mapper = SensorClockMapper(self.imu.timestamp_tick_ns, self.imu.timestamp_counter_bits)
            self.imu_fifo_watermark_words = ceil(self.imu.fifo_capacity_words * IMU_FIFO_WATERMARK_FRACTION)
            fifo_word_rate = self.imu.sample_rate_hz * IMU_FIFO_WORST_CASE_WORDS_PER_SAMPLE
            if fifo_word_rate <= 0 or self.imu_fifo_watermark_words >= self.imu.fifo_capacity_words:
                raise ValueError("IMU FIFO capacity, watermark, and sample rate must leave overflow headroom")
            time_from_watermark_to_overflow_s = (self.imu.fifo_capacity_words - self.imu_fifo_watermark_words) / fifo_word_rate
            self.imu_fifo_poll_interval_ns = max(1,round(time_from_watermark_to_overflow_s * IMU_FIFO_POLL_SAFETY_FACTOR * 1_000_000_000))

        self._last_imu_clock_observation_ns: int | None = None
        self._pending_imu_samples: deque[TimedIMUSample] = deque()

        self.state_csv, self.writer = self.initialize_csv()

        self.floorplan = Floorplan() # NOTE TODO: currently unused 
        self.current_room = self.floorplan.rooms['24'] 

    def run(self) -> None: 
        try: 
            self.initialize_estimator()
            self._run_processing_loop()
        except Exception as e: 
            print(f'State Estimator crashed! Error: {str(e)}', 'error', 'loc')
            raise e

    def initialize_estimator(self) -> None: 
        """
        Wait for a trilateration-sufficient update to arrive in the communication queue.
        Required to initialize the estimator with a fully constrained position in the global reference frame.  
        """
        if self.imu is not None:
            self._initialize_imu_clock_mapper()
        while self.estimator is None:
            # No graph state exists yet, so pre-initialisation samples cannot be
            # integrated.  The deterministic discard makes that intentional
            # rather than relying on continuous-mode FIFO overwrite semantics.
            if self.imu is not None:
                self._discard_preinitialization_imu_samples()
            try:
                msg = UpdateMessage.load(*self.com_queue.get_nowait())
            except Empty:
                sleep(self.imu_fifo_poll_interval_ns / 1_000_000_000 if (self.imu is not None) else STATE_INTERVAL)
                continue

            if msg.update_type == UpdateType.RANGING:
                anchor_ranges = [obs for obs in msg.range_observations if obs.is_anchor]
                if len(anchor_ranges) < 3:  # Need a fully constrained measurement for initialization
                    continue
                self.yaw_offset = msg.measured_yaw  # Store initial value, which we'll use to correct further poses
                raw_yaw = self.correct_yaw(msg.measured_yaw)

                anchor_data = [(obs.target_id, obs.distance_cm) for obs in anchor_ranges]
                if self.estimator_type == 'EKF':
                    self.estimator = CustomEKF(anchor_data, raw_yaw)
                    self.estimator.incorporate_ranging_data(msg.timestamp, msg.range_observations, raw_yaw)
                elif self.estimator_type == 'FG':
                    self.estimator = FactorGraph(anchor_data, raw_yaw, msg.timestamp)
                    self.estimator.incorporate_ranging_data(msg.timestamp, msg.range_observations, raw_yaw)

                # Estimator initialized. Internalise and publish the posterior.
                self.publish_state(msg)
        print(f"ESTIMATOR ({self.estimator_type}) INITIALIZATION DONE", 'ok', 'loc')

    def _run_processing_loop(self) -> None: 
        """
        Continuously estimates the posterior state every STATE_INTERVAL seconds.  

        For each interval ]t, t+STATE_INTERVAL]:
        1. Fetch all external measurements from the communication queue 
        2. Collect all IMU samples if available 
        3. Aggregate measures in a SensorBatch dataclass 
        4. Process the SensorBatch
            - Add all measures to the new state and use IMU info as a motion link. If no IMU, assume constant-velocity.
            - Update the estimator posterior and publish it 

        NOTE: All sensor timings are compared on the same EstimatorClock reference 
        """
        last_state_boundary = self.estimator_clock.now_ns()
        next_boundary = last_state_boundary + STATE_INTERVAL_NS
        pending_messages: list[UpdateMessage] = []
        
        while True:
            # Fetching measurements from comm queue.  Retain them until the
            # interval closes; recreating this list on each poll would lose events.
            pending_messages.extend(self._drain_communication_queue())
            if self.imu is not None:
                self._monitor_imu_fifo()
            now = self.estimator_clock.now_ns()
            if now < next_boundary:
                # Keep latency low without using a repeated fixed sleep, which
                # would accumulate scheduler drift over a long run.  In IMU mode,
                # wake often enough to service the FIFO before it can overflow.
                sleep_ns = next_boundary - now
                if self.imu is not None:
                    sleep_ns = min(sleep_ns, self.imu_fifo_poll_interval_ns)
                sleep(sleep_ns / 1_000_000_000)
                continue
            # Drain once more so we don't miss messages at boundary 
            pending_messages.extend(self._drain_communication_queue()) 
            # Force one IMU final read so samples below the watermark are included 
            if self.imu is not None:
                imu_samples = self._drain_imu_samples(last_state_boundary, next_boundary)
                self._refresh_imu_clock_if_due() # Periodically refresh conversion of IMU clock to correct drift
            else:
                imu_samples = []

            # Aggregate all messages and process them 
            batch = SensorBatch(
                start_time_ns=last_state_boundary,
                boundary_time_ns=next_boundary,
                messages=pending_messages,
                imu_samples=imu_samples,
            )
            self._process_sensor_batch(batch)
            pending_messages.clear() 

            # Advance from the previous deadline rather than from ``now``.  When
            # processing overruns, the following iterations catch up by closing
            # the missed state intervals instead of permanently shifting cadence.
            last_state_boundary = next_boundary
            next_boundary += STATE_INTERVAL_NS

    def _monitor_imu_fifo(self) -> None:
        """Drain a sufficiently full FIFO into the estimator-owned pending buffer.

        Reading early does not close an estimator interval or integrate a factor.
        ``_drain_imu_samples`` later selects these timestamped samples for the
        appropriate state interval.
        """
        if self.imu.get_FIFO_count() >= self.imu_fifo_watermark_words:
            self._read_imu_fifo_into_pending()

    def _drain_communication_queue(self) -> list[UpdateMessage]:
        """Returns a list of all currently available producer events and clears the queue"""
        pending_messages = [] 
        while True:
            try:
                pending_messages.append(UpdateMessage.load(*self.com_queue.get_nowait()))
            except Empty:
                return pending_messages

    def _drain_imu_samples(self, start_time_ns: int, boundary_time_ns: int) -> list[TimedIMUSample]:
        """Return IMU samples in (start_time_ns, boundary_time_ns], corrected for EstimatorClock time.

        Process pending and newly read FIFO samples in timestamp order in one
        pass. Samples after the boundary remain pending for the next interval;
        samples at or before the start are discarded.
        """
        # The boundary always forces a final read.  Earlier watermark reads have
        # already appended their samples to the same pending deque.
        self._read_imu_fifo_into_pending()
        pending_samples = self._pending_imu_samples
        self._pending_imu_samples = deque()
        interval_samples = []
        future_sample_pending = False
        for sample in pending_samples:
            if future_sample_pending or sample.timestamp_ns > boundary_time_ns:
                self._pending_imu_samples.append(sample)
                future_sample_pending = True
            elif sample.timestamp_ns > start_time_ns:
                interval_samples.append(sample)
        return interval_samples

    def _read_imu_fifo_into_pending(self) -> None:
        """Read all currently available FIFO words without assigning a state yet."""
        raw_samples = self.imu.read_FIFO(apply_bias=False)
        for raw_tick, acceleration, angular_velocity in raw_samples:
            if raw_tick is None or acceleration is None or angular_velocity is None:
                continue
            self._pending_imu_samples.append(TimedIMUSample(
                timestamp_ns=self.imu_clock_mapper.to_estimator_ns(raw_tick),
                acceleration=acceleration,
                angular_velocity=angular_velocity,
            ))

    def _initialize_imu_clock_mapper(self) -> None:
        """Establish a clock anchor after intentionally dropping startup FIFO data."""
        # An observation is newer than all samples currently in the FIFO.  Clear
        # that pre-estimation data before observing it so SensorClockMapper sees
        # raw ticks strictly in chronological order from this point forward.
        self.imu.read_FIFO(apply_bias=False)
        self._update_sensor_clock_mapper()

    def _discard_preinitialization_imu_samples(self) -> None:
        """Avoid FIFO overflow while waiting for a ranging-based initial state."""
        if self.imu.get_FIFO_count() >= self.imu_fifo_watermark_words:
            self.imu.read_FIFO(apply_bias=False)

    def _refresh_imu_clock_if_due(self) -> None:
        if self._last_imu_clock_observation_ns is None:
            return

        if self.estimator_clock.now_ns() - self._last_imu_clock_observation_ns < IMU_CLOCK_REFRESH_INTERVAL_NS:
            return

        self._update_sensor_clock_mapper()

    def _update_sensor_clock_mapper(self) -> None:
        """Update the IMU tick mapping without discarding unread FIFO samples."""
        before_ns = self.estimator_clock.now_ns()
        raw_tick = self.imu.get_timestamp()
        after_ns = self.estimator_clock.now_ns()
        estimator_time_ns = (before_ns + after_ns) // 2
        self.imu_clock_mapper.observe(raw_tick, estimator_time_ns)
        self._last_imu_clock_observation_ns = estimator_time_ns

    def _process_sensor_batch(self, batch: SensorBatch) -> None:
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
        # self._preintegrate_imu(batch.imu_samples, batch.start_time_ns, batch.boundary_time_ns)

        # 2. Parsing messages
        range_messages = [] 
        step_messages = [] 
        for message in batch.messages:
            if message.update_type == UpdateType.TOPOLOGY:
                self.update_neighbors(message.topology)
            elif message.update_type == UpdateType.RANGING: 
                range_messages.append(message)
            elif message.update_type == UpdateType.PEDOMETER: 
                step_messages.append(message)

        # 3. Create a state and its motion link.  Future FactorGraph support will
        # use IMU preintegration when available, otherwise its explicit
        # constant-velocity/random-walk factor.  No synthetic zero-motion factor
        # belongs here.
        # self.estimator.advance_to(batch.boundary_time_ns, preintegrated_imu=...)

        # 4. Add all range observations for the interval, then optimise once.  The
        # temporary adapters expose the intended batching behaviour to the EKF and
        # current factor-graph APIs; the final API will accept a SensorBatch.
        if range_messages:
            self._apply_batched_ranges(range_messages)

        # A step must become a relative step factor between timestamp-bracketing
        # states; it is not an absolute pose update and should not be treated as
        # one during the state-interval batch.
        if step_messages:
            self._queue_step_constraints(step_messages, batch)

        # CustomOdometry is intentionally not forwarded here.  It will either be
        # removed with the retired Cognifly integration or reintroduced later as a
        # generic, explicitly framed PoseObservation factor.

        # 5. Optimise and publish at the state cadence.  Current backends optimise
        # inside _apply_batched_ranges; a no-range interval therefore publishes the
        # latest posterior until their explicit advance_to() contract is added.
        self.publish_state(range_messages[-1] if range_messages else None)

    def _apply_batched_ranges(self, messages: list[UpdateMessage]) -> None:
        """Apply a batched range observation list to the estimator backend."""
        all_range_observations = []
        for message in messages:
            self.update_neighbors(message.topology)
            all_range_observations.extend(message.range_observations)

        latest = messages[-1]
        self.estimator.incorporate_ranging_data(
            latest.timestamp,
            all_range_observations,
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

    def update_neighbors(self, neighbors: dict):
        self.last_known_neighbors = neighbors

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
        - Updates the sound queue if available 
        """
        pose = self.estimator.pose

        with self.tag_lock:
            self.tag.pose = pose
 
        self.pose_callback(pose)

        if SAVE_TO_CSV and message is not None:
            self.save_to_csv(message)

        if self.sound_queue is not None:
            self.sound_queue.put(SoundMessage.save(SoundMessage(self.estimator.pose)))

    @staticmethod 
    def initialize_csv(): 
        if not SAVE_TO_CSV: 
            return None, None 
        filepath = 'pose_estimation.csv'
        fieldnames = ['tag_id', 'timestamp', 'update_type',
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
                'update_type': message.update_type,
                'estimator_x': pose.x,
                'estimator_y': pose.y,
                'estimator_z': pose.z,
                'estimator_yaw': pose.heading,
                'covariance_matrix': "to implement", # previously: np.linalg.det(self.estimator.P)
                'slots': message.slots,
                'two_hop_neighbors': self.last_known_neighbors
            }

            self.writer.writerow(csv_data)
            self.state_csv.flush()
