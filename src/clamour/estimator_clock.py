"""Common timebase primitives for sensor fusion.

Estimator time is an integer count of nanoseconds since one host monotonic epoch.
Never mix it with wall-clock time or TDMA logical time.  Preserve raw sensor
timestamps alongside converted values so future clock-calibration work remains
auditable.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from time import monotonic_ns


NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class EstimatorClock:
    """Process-shareable estimator clock with a single monotonic host epoch."""

    epoch_monotonic_ns: int

    @classmethod
    def start(cls) -> "EstimatorClock":
        """Create once in ``Clamour.start`` before sensor processes are spawned."""
        return cls(epoch_monotonic_ns=monotonic_ns())

    def now_ns(self) -> int:
        return monotonic_ns() - self.epoch_monotonic_ns

    def sample_ns(self) -> tuple[int, int]:
        """Return ``(estimator_ns, raw_host_monotonic_ns)`` from one clock read."""
        raw_host_ns = monotonic_ns()
        return raw_host_ns - self.epoch_monotonic_ns, raw_host_ns

    def from_host_monotonic_ns(self, host_time_ns: int) -> int:
        return host_time_ns - self.epoch_monotonic_ns


@dataclass(frozen=True)
class ClockMapping:
    """Snapshot of an affine sensor-clock to estimator-clock conversion."""

    scale_ns_per_tick: float
    offset_ns: float
    residual_sigma_ns: float | None
    version: int


class SensorClockMapper:
    """Map one wrapping hardware counter onto :class:`EstimatorClock` time.

    ``observe`` receives occasional paired hardware/host readings and estimates
    ``estimator_ns = scale * unwrapped_ticks + offset``.  The rolling fit tracks
    oscillator drift.  It is deliberately small scaffolding, not a transport
    synchronization protocol: callers must ensure a host observation represents
    the hardware time they pair with it, rather than FIFO delivery time.
    """

    def __init__(self, nominal_tick_ns: float, *, counter_bits: int | None = None,
                 max_observations: int = 64):
        if nominal_tick_ns <= 0:
            raise ValueError("nominal_tick_ns must be positive")
        if counter_bits is not None and counter_bits <= 0:
            raise ValueError("counter_bits must be positive")
        if max_observations < 2:
            raise ValueError("max_observations must be at least two")

        self.nominal_tick_ns = nominal_tick_ns
        self.counter_bits = counter_bits
        self._modulus = (1 << counter_bits) if counter_bits is not None else None
        self._last_raw_tick: int | None = None
        self._wrap_offset = 0
        self._observations: deque[tuple[int, int]] = deque(maxlen=max_observations)
        self._mapping: ClockMapping | None = None

    @property
    def mapping(self) -> ClockMapping | None:
        return self._mapping

    def unwrap_tick(self, raw_tick: int) -> int:
        """Unwrap ordered hardware ticks; FIFO samples must be supplied in order."""
        if raw_tick < 0:
            raise ValueError("raw_tick must be non-negative")
        if self._modulus is not None:
            raw_tick %= self._modulus
            if self._last_raw_tick is not None:
                # A large backwards jump is counter rollover; smaller backwards
                # jumps mean the caller supplied samples out of chronological order.
                if raw_tick < self._last_raw_tick:
                    if self._last_raw_tick - raw_tick > self._modulus // 2:
                        self._wrap_offset += self._modulus
                    else:
                        raise ValueError("sensor ticks arrived out of order")
        elif self._last_raw_tick is not None and raw_tick < self._last_raw_tick:
            raise ValueError("sensor ticks arrived out of order")

        self._last_raw_tick = raw_tick
        return self._wrap_offset + raw_tick

    def observe(self, raw_tick: int, estimator_time_ns: int) -> ClockMapping:
        """Add a paired clock observation and refresh the affine mapping."""
        tick = self.unwrap_tick(raw_tick)
        self._observations.append((tick, estimator_time_ns))

        if len(self._observations) == 1:
            scale = self.nominal_tick_ns
            offset = estimator_time_ns - scale * tick
            residual_sigma = None
        else:
            ticks, times = zip(*self._observations)
            mean_tick = sum(ticks) / len(ticks)
            mean_time = sum(times) / len(times)
            denominator = sum((tick - mean_tick) ** 2 for tick in ticks)
            scale = self.nominal_tick_ns if denominator == 0 else sum(
                (tick - mean_tick) * (timestamp - mean_time)
                for tick, timestamp in zip(ticks, times)
            ) / denominator
            if scale <= 0:
                # A bad host pairing must not invert time.  Keep the nominal rate
                # and let the residual expose the problem to the caller.
                scale = self.nominal_tick_ns
            offset = mean_time - scale * mean_tick
            residuals = [timestamp - (scale * tick + offset)
                         for tick, timestamp in zip(ticks, times)]
            residual_sigma = (sum(residual ** 2 for residual in residuals) /
                              len(residuals)) ** 0.5

        version = 1 if self._mapping is None else self._mapping.version + 1
        self._mapping = ClockMapping(scale, offset, residual_sigma, version)
        return self._mapping

    def to_estimator_ns(self, raw_tick: int) -> tuple[int, ClockMapping]:
        """Convert the next ordered sensor tick using the latest calibration."""
        if self._mapping is None:
            raise RuntimeError("clock mapper needs an initial paired observation")
        tick = self.unwrap_tick(raw_tick)
        mapped_ns = round(self._mapping.scale_ns_per_tick * tick + self._mapping.offset_ns)
        return mapped_ns, self._mapping
