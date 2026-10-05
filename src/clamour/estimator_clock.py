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
        self._modulus = (1 << counter_bits) if counter_bits is not None else None
        self._last_raw_tick: int | None = None
        self._wrap_offset = 0
        self._observations: deque[tuple[int, int]] = deque(maxlen=max_observations)
        self._scale_ns_per_tick: float | None = None
        self._offset_ns: float | None = None

    def _unwrap_tick(self, raw_tick: int) -> int:
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

    def observe(self, raw_tick: int, estimator_time_ns: int) -> None:
        """Add a paired clock observation and refresh the affine mapping."""
        tick = self._unwrap_tick(raw_tick)
        self._observations.append((tick, estimator_time_ns))

        if len(self._observations) == 1:
            scale = self.nominal_tick_ns
            offset = estimator_time_ns - scale * tick
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
                # rather than allowing time to run backwards.
                scale = self.nominal_tick_ns
            offset = mean_time - scale * mean_tick
        self._scale_ns_per_tick = scale
        self._offset_ns = offset

    def to_estimator_ns(self, raw_tick: int) -> int:
        """Convert the next ordered sensor tick using the latest calibration."""
        if self._scale_ns_per_tick is None or self._offset_ns is None:
            raise RuntimeError("clock mapper needs an initial paired observation")
        tick = self._unwrap_tick(raw_tick)
        return round(self._scale_ns_per_tick * tick + self._offset_ns)
