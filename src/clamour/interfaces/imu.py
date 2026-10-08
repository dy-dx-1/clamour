from abc import ABC, abstractmethod

# Expected format is (timestamp, acceleration, gyro) 
RawIMUSample = tuple[int | None, object | None, object | None]


class IMU(ABC):
    """Generic IMU interface; timestamps remain raw sensor ticks."""

    @property
    @abstractmethod
    def timestamp_tick_ns(self) -> float:
        """Nominal duration in nanoseconds of one hardware timestamp tick."""

    @property
    @abstractmethod
    def timestamp_counter_bits(self) -> int | None:
        """Counter width, or ``None`` for a non-wrapping counter."""

    @property
    @abstractmethod
    def sample_rate_hz(self) -> float:
        """Configured rate of grouped accel/gyro samples in the FIFO."""

    @abstractmethod
    def get_FIFO_count(self) -> int:
        """Return the number of FIFO words currently available."""

    @abstractmethod
    def read_FIFO(self, apply_bias: bool = False, word_count: int | None = None) -> list[RawIMUSample]:
        """Read up to ``word_count`` FIFO words as raw-tick sample tuples."""

    @abstractmethod
    def get_timestamp(self) -> int:
        """Return the current raw hardware timestamp tick."""

    @abstractmethod
    def close(self) -> None:
        """Release the hardware connection."""

    def __enter__(self) -> "IMU":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()