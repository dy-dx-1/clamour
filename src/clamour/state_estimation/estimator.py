import csv
from time import sleep

from statistics import median
from math import ceil
from collections import defaultdict, deque
from dataclasses import dataclass
from queue import Empty
from typing import Callable, ContextManager, Literal
from multiprocessing.synchronize import Lock
import socket
import struct
import numpy as np

# NOTE: CustomEKF and FactorGraph are imported in _initialize_estimator so the EKF can run without gtsam installed
from ..estimator_clock import EstimatorClock, SensorClockMapper
from ..custom_terminal import print 
from ..config import SAVE_TO_CSV, DYNAMIC_MIST, UDP_HOST, UDP_PORT
from ..interfaces import IMU, Tag, Pose
from ..sensor_events import RangeObservation, TimedIMUSample
from ..contextManagedQueue import ContextManagedQueue
from ..messages.updateMessage import UpdateMessage
from ..messages.soundMessage import SoundMessage
from ..messages.types import UpdateType
from ..rooms import Floorplan

# This is deliberately local while the fusion configuration is being introduced.  It
# is the cadence of *states*, not necessarily the IMU sample, range, or output rate.
NS_PER_S = 1_000_000_000
STATE_INTERVAL = 0.1  # seconds
STATE_INTERVAL_NS = round(STATE_INTERVAL * NS_PER_S)
# Ranges are sent in batches, so the oldest range of a batch reaches the queue late.
# Tolerance covers one failed TWR exchange (2 × 15 ms DW listen timeouts) plus margin for queue transit.
RANGE_LATENESS_NS = 35 * 1_000_000
assert RANGE_LATENESS_NS < STATE_INTERVAL_NS
# The IMU -> EstimatorClock mapping must be refreshed periodically to correct drift.
# 5s was the value decided after testing. It's a cheap check and tracks temp changes due to startup easily. 
# Could even be longer, as the error is very small, but it really doesn't cost much to do it every 5s. 
IMU_CLOCK_REFRESH_INTERVAL_NS = round(5.0 * NS_PER_S)
IMU_CLOCK_FIT_OBSERVATIONS = 16
# Stationary IMU initialization: clock observations at these delays (s) after the first one.  
# Nominal tick rate is ~0.6% off. Early increasingly spaced observations correct the scale quickly while growing the baseline that averages pairing noise.
# NOTE: IF CHANGING <2S IN THE FUTURE, NEED TO ENSURE IMU STILL GETS >=1.5S TO INITIALIZE IT'S SFLP BLOCK FOR INITIAL ATTITUDE ESTIMATION!
# NOTE: FIRST DELAY MUST STAY ABOVE 0.1S TO ALLOW DISCARDING INITIAL BAD GYRO SAMPLES (see _initialize_imu)
IMU_INIT_OBSERVATION_DELAYS = (0.25, 0.5, 1.0, 2.0)
# FIFO emptying safeties - we empty the FIFO before the limit to avoid losing info
# poll_interval = SF × time_to_overflow
# time_to_overflow = (capacity_words - watermark_words) / fifo_word_rate
# fifo_word_rate = sample_rate_hz × IMU_FIFO_WORST_CASE_WORDS_PER_SAMPLE
IMU_FIFO_WATERMARK_FRACTION = 0.70
IMU_FIFO_WORST_CASE_WORDS_PER_SAMPLE = 3 # Words per grouped FIFO sample: ex: accel, gyro, timestamp 
IMU_FIFO_POLL_SAFETY_FACTOR = 0.5

@dataclass
class SensorBatch:
    """
    Measurements collected for one future graph-state interval.

    ``imu_start_sample`` is the latest sample at or before ``start_time_ns``;
    it supplies the initial zero-order-held value. Boundary and measurement
    times are integer nanoseconds on the shared estimator clock.
    """
    start_time_ns: int
    boundary_time_ns: int
    messages: list[UpdateMessage]
    imu_start_sample: TimedIMUSample | None
    imu_samples: list[TimedIMUSample]

class StateEstimator:
    """
    General interface for pose estimation. 
    Continuously runs the .run() method in it's own ContextManagedProcess.

    ARGS: 
    - tag: Tag object to track 
    - imu_factory: function that creates an IMU instance or None if not available 
    - estimator_clock: EstimatorClock object that will serve as common clock time
    - estimator_type: EKF or Factor Graph
    - pose_callback: Function that takes a Pose and prints it on pose update # TODO remove? just put inside?
    - communication_queue: Queue where pose updates / messages to process appear 
    - sound_queue: Optional, if a Queue is passed, will send updates to it to allow for sound playing 
    """
    def __init__(self, tag: Tag, imu_factory: Callable[[], ContextManager[IMU | None]],
                  estimator_clock: EstimatorClock, estimator_type: Literal['EKF', 'FG'], 
                  pose_callback, communication_queue: ContextManagedQueue, sound_queue: None|ContextManagedQueue):
        self.tag = tag 

        self.estimator = None 
        self.estimator_type = estimator_type # validity check done in config  

        self.last_known_neighbors = {}

        self.pose_callback = pose_callback # NOTE future eval if can just put this in here, idk why need to pass it as arg

        self.sound_queue = sound_queue
        self.com_queue = communication_queue

        # Clamour shares this immutable epoch with all local producer processes.
        self.estimator_clock = estimator_clock
        self.imu_factory = imu_factory
        self.imu: IMU | None = None

        self._last_imu_clock_observation_ns: int | None = None
        self._pending_imu_samples: deque[TimedIMUSample] = deque()
        # The latest measurement at or before the current state boundary.  Its
        # value is held until the next sample for zero-order-hold integration.
        self._held_imu_sample: TimedIMUSample | None = None

        if SAVE_TO_CSV:
            self.state_csv = open('pose_estimation.csv', 'w')
            self.writer = csv.DictWriter(self.state_csv, fieldnames=[
                'tag_id', 'timestamp', 'update_type',
                'estimator_x', 'estimator_y', 'estimator_z', 'estimator_yaw',
                'covariance_matrix', 'slots', 'two_hop_neighbors'])
            self.writer.writeheader()
        if DYNAMIC_MIST:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_dest = (UDP_HOST, UDP_PORT)

        self.floorplan = Floorplan() # NOTE TODO: currently unused 
        self.current_room = self.floorplan.rooms['24'] 

    def run(self) -> None: 
        try: 
            with self.imu_factory() as imu:
                first_state_time_ns = self._initialize_estimator(imu)
                self._run_fusion_loop(first_state_time_ns)
        except Exception as e:
            print(f'State Estimator crashed! Error: {e}', 'error', 'loc')
            raise
        finally:
            if DYNAMIC_MIST:
                self._socket.close()

    def _initialize_estimator(self, imu: IMU|None) -> int:
        """
        Wait for a trilateration-sufficient update to arrive in the communication queue.
        Required to initialize the estimator with a fully constrained position in the global reference frame.
        Returns the EstimatorClock time (ns) of the first estimator state.
        """
        print("Estimator initialization started. Keep the tag **stationary** until full initialization completes.", 'info', 'loc')
        # (yaw, pitch, roll) in degrees, IMU convention, or None without an IMU.
        # The EKF ignores it: it has no pitch/roll states and its yaw is 0 by definition.
        # Initial gyro bias in mdps, or None without an IMU. Only used by the FG.
        initial_attitude, initial_gyro_bias_mdps, initial_gyro_bias_std_mdps = self._initialize_imu(imu)
        print(f"{initial_gyro_bias_std_mdps=}", 'error', 'loc')
        while True:
            try:
                msg = UpdateMessage.load(*self.com_queue.get_nowait())
            except Empty:
                # Initial state not set yet, so IMU samples can't be integrated: drop them
                self._idle(STATE_INTERVAL_NS)
                self._pending_imu_samples.clear()
                continue

            if msg.update_type != UpdateType.RANGING:
                continue
            anchor_ros = [obs for obs in msg.range_observations if obs.is_anchor]
            if len(anchor_ros) < 4:  # Need a fully constrained measurement for initialization
                continue

            # The global frame starts at the initial heading, so the initial yaw is 0 by definition.
            # Anchoring first graph state to when we first get sufficient ranges
            first_state_time_ns = self.estimator_clock.now_ns()
            if self.estimator_type == 'EKF':
                from .ekf import CustomEKF
                self.estimator = CustomEKF(anchor_ros, 0.0, first_state_time_ns, STATE_INTERVAL)
                self.estimator.incorporate_ranging_data(msg.range_observations, 0.0)
            elif self.estimator_type == 'FG':
                from .factor_graph import FactorGraph
                if self.imu is not None:
                    accel_covar_density, gyro_covar_density = self.imu.accel_covar_density_mg, self.imu.gyro_covar_density_mg
                else:
                    accel_covar_density = gyro_covar_density = None
                # Yaw is 0 (global frame starts at the initial heading)
                prior_ypr = (0.0, *initial_attitude[1:]) if initial_attitude is not None else (0.0, 0.0, 0.0)
                self.estimator = FactorGraph(anchor_ros, prior_ypr, first_state_time_ns, accel_covar_density, gyro_covar_density,
                                             initial_gyro_bias_mdps, initial_gyro_bias_std_mdps)
            # Estimator initialized. Internalise and publish the posterior.
            self._publish_pose(msg)
            print(f"{self.estimator_type} INITIALIZATION DONE! INITIAL ATTITUDE: (pitch: {int(prior_ypr[1])}, roll: {int(prior_ypr[2])}, yaw: {int(prior_ypr[0])} degs)", 'ok', 'loc')
            return first_state_time_ns

    def _initialize_imu(self, imu: IMU|None) -> tuple[tuple[float, float, float] | None, np.ndarray | None, np.ndarray | None]:
        """
        Stationary IMU initialization, run before listening for ranging observations.
        - Assigns self.imu
        - Creates self.imu_clock_mapper
        - Sets safe FIFO reading parameters
        - Calibrates the IMU clock mapping with observations at IMU_INIT_OBSERVATION_DELAYS.
        - Reads the initial attitude once the clock calibration wait has let it stabilize.
        - Estimates the initial gyro bias and its std from the stationary FIFO gyro samples.
        - Raises a RuntimeError if the gyro samples show the tag moved during initialization.

        Returns (initial_attitude, gyro_bias_mdps, gyro_bias_std_mdps), all None without an IMU:
        - initial_attitude: (yaw, pitch, roll) in degrees where (X=pitch, Y=roll, Z=yaw)
        - gyro_bias_mdps: per-axis gyro bias in mdps, scale-corrected like the FIFO samples. Let's us always start the graph with fresh gyro data (changes from power up to power up)
        - gyro_bias_std_mdps: per-axis std of the gyro bias estimate in mdps (noise only), at least IMU_INIT_GYRO_BIAS_STD_FLOOR_MDPS
        """
        self.imu = imu
        if imu is None:
            return None, None, None

        # Initializing clock mapper & safe FIFO reading params 
        self.imu_clock_mapper = SensorClockMapper(imu.timestamp_tick_ns, imu.timestamp_counter_bits,
                                                  IMU_CLOCK_FIT_OBSERVATIONS)
        self.imu_fifo_watermark_words = ceil(imu.fifo_capacity_words * IMU_FIFO_WATERMARK_FRACTION)
        fifo_word_rate = imu.sample_rate_hz * IMU_FIFO_WORST_CASE_WORDS_PER_SAMPLE
        if fifo_word_rate <= 0 or self.imu_fifo_watermark_words >= imu.fifo_capacity_words:
            raise ValueError("IMU FIFO capacity, watermark, and sample rate must leave overflow headroom")
        time_from_watermark_to_overflow_s = (imu.fifo_capacity_words - self.imu_fifo_watermark_words) / fifo_word_rate
        self.imu_fifo_poll_interval_ns = max(1, round(time_from_watermark_to_overflow_s * IMU_FIFO_POLL_SAFETY_FACTOR * NS_PER_S))

        # Initial IMU->EstimatorClock mapping 
        self.imu.discard_FIFO()
        self._observe_imu_clock()
        start_ns = self._last_imu_clock_observation_ns
        for i, delay in enumerate(IMU_INIT_OBSERVATION_DELAYS):
            due_ns = start_ns + round(delay * NS_PER_S)
            while (now := self.estimator_clock.now_ns()) < due_ns:
                self._idle(due_ns - now)
            self._observe_imu_clock()  # Also drains the FIFO into pending samples
            if i == 0:
                # Drop the first drain (0.25s): it holds the gyro's turn-on bad samples which corrupts the gyro bias estimate
                self._pending_imu_samples.clear()

        # The calibration wait above (~2s, stationary) covers the attitude's 1.5s calibration + stabilization time
        initial_attitude = imu.get_attitude()
        
        # Estimating bias
        IMU_INIT_GYRO_BIAS_BLOCKS = 10 # STD dev is estimated in blocks to avoid assuming low pass filter values are independent
        # Per-sample gyro std above which the tag is considered to have moved (stationary noise is ~30mdps)
        if len(self._pending_imu_samples) < IMU_INIT_GYRO_BIAS_BLOCKS:
            raise RuntimeError("Not enough IMU samples collected during stationary initialization to estimate the gyro bias")
        gyro_samples = np.array([sample.angular_velocity for sample in self._pending_imu_samples])
        block_means = np.array([block.mean(axis=0) for block in np.array_split(gyro_samples, IMU_INIT_GYRO_BIAS_BLOCKS)])
        gyro_bias_mdps = gyro_samples.mean(axis=0)
        # Setting 2.0 mdps floor (datasheet noise) to ensure the bias is not too confident
        gyro_bias_std_mdps = np.maximum(block_means.std(axis=0, ddof=1) / np.sqrt(IMU_INIT_GYRO_BIAS_BLOCKS), 2.0)

        # Motion check: a moving tag invalidates both the gyro bias and the SFLP attitude
        IMU_INIT_MAX_STATIONARY_GYRO_STD_MDPS = 100.0
        sample_std_mdps = gyro_samples.std(axis=0, ddof=1)
        if np.any(sample_std_mdps > IMU_INIT_MAX_STATIONARY_GYRO_STD_MDPS):
            raise RuntimeError(f"Tag moved during IMU initialization (gyro sample std {np.round(sample_std_mdps, 1)} mdps > "
                               f"{IMU_INIT_MAX_STATIONARY_GYRO_STD_MDPS} mdps). Restart while keeping the tag stationary.")
        
        # Samples were taken before the first state, they can't be integrated
        self._pending_imu_samples.clear()
        return initial_attitude, gyro_bias_mdps, gyro_bias_std_mdps

    def _idle(self, max_sleep_ns: int) -> None:
        """
        Sleep for up to max_sleep_ns while keeping the IMU serviced.
        - Without an IMU: plain sleep of max_sleep_ns.
        - With an IMU: first empties the FIFO into pending samples if it is near full, then
          caps the sleep at the FIFO poll interval so the FIFO can't overflow. Callers loop
          until their deadline, so a shortened sleep is fine.
        """
        if self.imu is not None:
            if self.imu.get_FIFO_count() >= self.imu_fifo_watermark_words:
                self._read_imu_fifo_into_pending()
            max_sleep_ns = min(max_sleep_ns, self.imu_fifo_poll_interval_ns)
        sleep(max_sleep_ns / NS_PER_S)

    def _run_fusion_loop(self, first_state_time_ns: int) -> None:
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
        NOTE: Each interval is closed RANGE_LATENESS_NS after its boundary, which delays publishing by the same amount
        """
        # Start from the first graph state's time
        last_state_boundary = first_state_time_ns
        next_boundary = last_state_boundary + STATE_INTERVAL_NS
        pending_messages: list[UpdateMessage] = []

        while True:
            # Wait RANGE_LATENESS_NS past the boundary before closing the interval, so
            # batched ranges acquired just before it can still reach the queue.
            # Later events are kept as future measurements by their event time.
            close_time = next_boundary + RANGE_LATENESS_NS
            now = self.estimator_clock.now_ns()
            # Get messages from comm queue. now_ns() is read before draining so on the closing pass
            # drain happens after close_time and ensures we get all relevant msgs 
            pending_messages.extend(self._drain_communication_queue())
            if now < close_time:
                # Sleep until the close time rather than a fixed period, which would
                # accumulate scheduler drift over a long run.
                # Also fetch IMU samples if needed
                self._idle(close_time - now)
                continue

            # Aggregate interval IMU samples if available and refresh IMU clock mapping if needed
            imu_start_sample, imu_samples = self._take_interval_imu_samples(last_state_boundary, next_boundary)

            # Aggregate all messages and process them 
            batch = SensorBatch(
                start_time_ns=last_state_boundary,
                boundary_time_ns=next_boundary,
                messages=pending_messages,
                imu_start_sample=imu_start_sample,
                imu_samples=imu_samples,
            )
            pending_messages = self._process_sensor_batch(batch)

            # Advance from the previous deadline rather than from ``now``.  When
            # processing overruns, the following iterations catch up by closing
            # the missed state intervals instead of permanently shifting cadence.
            last_state_boundary = next_boundary
            next_boundary += STATE_INTERVAL_NS        

    def _drain_communication_queue(self) -> list[UpdateMessage]:
        """Returns a list of all currently available producer events and clears the queue"""
        pending_messages = []
        while True:
            try:
                pending_messages.append(UpdateMessage.load(*self.com_queue.get_nowait()))
            except Empty:
                return pending_messages

    def _take_interval_imu_samples(self, start_time_ns: int, boundary_time_ns: int) -> tuple[TimedIMUSample | None, list[TimedIMUSample]]:
        """Return the held start sample and samples in ``(start, boundary]``.

        The graph uses the returned start sample as a zero-order-held control
        value from the prior state boundary until the first new sample.

        Returns ``(None, [])`` without an IMU. Also refreshes the IMU clock mapping when it is due.
        """
        if self.imu is None:
            return None, []
        # The boundary always forces a final read.  Earlier watermark reads have
        # already appended their samples to the same pending deque.
        # Samples past the boundary stay pending for the next interval.
        self._read_imu_fifo_into_pending()
        interval_samples = []
        while self._pending_imu_samples:
            sample = self._pending_imu_samples.popleft()
            if sample.timestamp_ns > boundary_time_ns:
                # It is the first sample in the next interval.  Keeping it at
                # the left avoids scanning/rebuilding the future FIFO backlog.
                self._pending_imu_samples.appendleft(sample)
                break
            if sample.timestamp_ns <= start_time_ns:
                self._held_imu_sample = sample
            else:
                interval_samples.append(sample)

        start_sample = self._held_imu_sample
        # DEBUG: next_ahead_ms > 0 is how far the first unaccepted sample lies past the boundary
        #next_ahead_ms = (self._pending_imu_samples[0].timestamp_ns - boundary_time_ns) / 1e6 if self._pending_imu_samples else None
        #print(f"IMU drain: held={start_sample is not None} n_interval={len(interval_samples)} "
        #      f"n_pending={len(self._pending_imu_samples)} next_ahead_ms={next_ahead_ms}", 'info', 'loc')
        if interval_samples:
            self._held_imu_sample = interval_samples[-1]

        # Periodically refresh conversion of IMU clock to correct drift
        if self.estimator_clock.now_ns() - self._last_imu_clock_observation_ns >= IMU_CLOCK_REFRESH_INTERVAL_NS:
            self._observe_imu_clock()
        return start_sample, interval_samples

    def _read_imu_fifo_into_pending(self) -> None:
        """Read all currently available FIFO words into self._pending_imu_samples."""
        raw_samples = self.imu.read_FIFO(apply_bias=False)
        for raw_tick, acceleration, angular_velocity in raw_samples:
            sample = TimedIMUSample(
                timestamp_ns=self.imu_clock_mapper.to_estimator_ns(raw_tick),
                acceleration=acceleration,
                angular_velocity=angular_velocity,
            )
            if (self._pending_imu_samples and
                    sample.timestamp_ns < self._pending_imu_samples[-1].timestamp_ns):
                # Sorting would hide a FIFO/parser/clock-mapping defect and add
                # O(n log n) work every interval.  FIFO order is a source contract.
                raise RuntimeError("IMU samples are not chronological after clock conversion")
            self._pending_imu_samples.append(sample)

    def _observe_imu_clock(self) -> None:
        """Update the IMU tick mapping after mapping the queued FIFO samples.

        A current hardware-clock read is newer than FIFO samples still in the
        device.  ``SensorClockMapper`` tolerates this (observations are excluded
        from its sample ordering check), since samples written during the drain
        or the timestamp read cannot be avoided.  Draining first still keeps
        most queued samples on the calibration in force when they were acquired.
        """
        # Skipped on the first call: the FIFO was just discarded and the mapper has
        # no observation yet, so it couldn't convert sample ticks.
        if self._last_imu_clock_observation_ns is not None:
            self._read_imu_fifo_into_pending()
        before_ns = self.estimator_clock.now_ns()
        raw_tick = self.imu.get_timestamp()
        after_ns = self.estimator_clock.now_ns()
        estimator_time_ns = (before_ns + after_ns) // 2
        self.imu_clock_mapper.observe(raw_tick, estimator_time_ns)
        self._last_imu_clock_observation_ns = estimator_time_ns

    def _process_sensor_batch(self, batch: SensorBatch) -> list[UpdateMessage]:
        """
        Generate a new posterior state update and retain range messages whose
        event times belong to a future state interval.
        """
        # Parse messages. Ranges are split into the current interval, future intervals, and late drops.
        interval_ranges = []
        future_range_messages = []
        interval_range_message = None  # Last message with a range in this interval, used for CSV
        step_messages = []
        for message in batch.messages:
            if message.update_type in (UpdateType.TOPOLOGY, UpdateType.RANGING):
                self.last_known_neighbors = message.topology
            if message.update_type == UpdateType.RANGING:
                future_observations = []
                for observation in message.range_observations:
                    if observation.event_time_ns <= batch.start_time_ns:
                        # TODO: Assign delayed measurements to their historical graph state if needed
                        print(f"WARNING: Dropping late range to {observation.target_id} acquired at {observation.event_time_ns} ns; "
                              f"current interval starts at {batch.start_time_ns} ns.", 'error', 'loc')
                    elif observation.event_time_ns <= batch.boundary_time_ns:
                        interval_ranges.append(observation)
                        interval_range_message = message
                    else:
                        future_observations.append(observation)
                if future_observations:
                    # The message is kept for a later interval, holding only its future ranges
                    message.range_observations = future_observations
                    future_range_messages.append(message)
            elif message.update_type == UpdateType.PEDOMETER:
                step_messages.append(message)
        range_observations = self._coalesce_ranges(interval_ranges)

        # Create the new state and integrate the info from the sensor batch. 
        self.estimator.advance_to(
            batch.boundary_time_ns,
            batch.imu_start_sample,
            batch.imu_samples,
            range_observations,
        )
            
        """ 
        PEDOMETER AND CUSTOM POSES NOT YET IMPLEMENTED
        # A step must become a relative step factor between timestamp-bracketing states

        # CustomOdometry is intentionally not forwarded here.  It will either be
        # removed with the retired Cognifly integration or reintroduced later as a
        # generic, explicitly framed PoseObservation factor.
        """
        
        # Optimise and publish
        self._publish_pose(interval_range_message)
        return future_range_messages

    @staticmethod
    def _coalesce_ranges(observations: list[RangeObservation]) -> list[RangeObservation]:
        """Group anchor ranges by median; preserve tag ranges as independent factors.

        Median limits the influence of a short NLOS outlier burst without
        pretending repeated UWB samples are independent. Tag ranges are kept separate 
        because the neighbor's position and covariance might evolve significantly during the batch. 
        """
        anchor_groups: dict[int, list[RangeObservation]] = defaultdict(list)
        tag_observations = []
        for observation in observations:
            if observation.is_anchor:
                anchor_groups[observation.target_id].append(observation)
            else:
                tag_observations.append(observation)

        coalesced_anchors = [
            RangeObservation(
                target_id=target_id,
                distance_cm=median(item.distance_cm for item in grouped),
                event_time_ns=max(item.event_time_ns for item in grouped),
                is_anchor=True,
            )
            for target_id, grouped in anchor_groups.items()
        ]
        return coalesced_anchors + tag_observations

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

    def _publish_pose(self, message: UpdateMessage | None):
        """
        - Saves the pose and covariance in its Tag object
        - Prints out the current posterior from the estimator
        - Saves to CSV if configured to do so (config.py) 
        - Updates the sound queue if available 
        TODO: remove requirement of a message arg. Update CSV to not need this as it makes it less clean
        """
        pose = self.estimator.pose

        # No tag lock: the pose is a SharedPose with its own lock, and the TDMA process holds the
        # tag lock during ranging/listening (DW timeouts of ~15ms), which would stall the fusion loop.
        self.tag.pose = pose

        self.pose_callback(pose)

        if DYNAMIC_MIST:
            meters = [coord/100 for coord in pose.coordinates]
            packet = struct.pack("<ifff", self.tag.tag_id, *meters)
            self._socket.sendto(packet, self.sock_dest)

        if SAVE_TO_CSV and message is not None:
            self._write_csv_row(message, pose)

        if self.sound_queue is not None:
            self.sound_queue.put(SoundMessage.save(SoundMessage(pose)))

    def _write_csv_row(self, message: UpdateMessage, pose: Pose):
        """
        NOTE: Previously, this function also printed the raw position pre-EKF. During the move to FGs, I've removed this option. 
        This is because we now directly deal with multiple ranges, so we'd either need to:
        - Add a function inside the estimator that returns the 'non optimized' geometric value 
        - Add a function on here that quickly computes an estimation to generate raw values (not ideal, as we'd be doing the calc twice)
        ALSO: would need to figure out how to consider the IMU effect on the 'raw pos'. Do we incorporate it? Or do we just print out geometric results?
        What would be the benefit?

        Since the answer to these questions may be clearer in the future, leaving it without raw position for now (2026-08-18).

        NOTE: ``message`` is always a RANGING message. TODO add compatibility to IMU in future
        """
        self.writer.writerow({
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
        })
        self.state_csv.flush()
