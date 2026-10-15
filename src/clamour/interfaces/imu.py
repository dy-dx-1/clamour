from abc import ABC, abstractmethod
import numpy as np 
from numpy.typing import NDArray

# Complete FIFO sample: (timestamp, acceleration, gyro).
# Timestamp is in raw ticks; acceleration is in mg and gyro in mdps.
# Drivers must omit incomplete temporal groups rather than returning ``None``
# members in this tuple.
RawIMUSample = tuple[int, NDArray[np.float64], NDArray[np.float64]]


class IMU(ABC):
    """
    Generic IMU interface; timestamps remain raw sensor ticks. These values should all be from the datasheet.
    
    The IMU must internally apply scale factor correction but not bias as the FG will handle this. 
    Values read from FIFO must be raw tick timestamps, mg for acceleration and mdps for gyro.
    """
    ### ACCEL/GYRO SCALE FACTOR 
    # IMU must internally apply scale factor correction to values read from FIFO 
    # Bias is not an abstractclass requirement as it needs to be defined in config.py . IMU samples should NOT be bias corrected. 
    @property
    @abstractmethod
    def accel_scale_factor_mg(self) -> np.ndarray: 
        """3x3 scale factor matrix in mgs. Will be overwritten by config value on IMU instanciation.
           Here to highlight that the IMU must internally apply this to it's FIFO-read values."""
    @property
    @abstractmethod
    def gyro_scale_factor_mg(self) -> np.ndarray: 
        """3x3 scale factor matrix in mdps. Will be overwritten by config value on IMU instanciation.
           Here to highlight that the IMU must internally apply this to it's FIFO-read values."""        

    ### ACCEL/GYRO COVARIANCE  
    # Measurement values (from datasheet) (GTSAM expects them in setAccelerometerCovariance/setGyroscopeCovariance) 
    # GTSAM expects a density as it will multiply per 1/delta_t during pre-integration 
    @property
    @abstractmethod
    def accel_covar_density_mg(self) -> float: 
        """Covariance density of accelerometer measurements in (mg**2)*s"""

    @property
    @abstractmethod
    def gyro_covar_density_mg(self) -> float: 
        """Covariance density of gyroscope measurements in (mdps**2)*s"""

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

    @property
    @abstractmethod
    def fifo_capacity_words(self) -> int:
        """Maximum number of FIFO words retained before samples are overwritten."""

    @abstractmethod
    def get_FIFO_count(self) -> int:
        """Return the number of FIFO words currently available."""

    @abstractmethod
    def read_FIFO(self, apply_bias: bool = False, word_count: int | None = None) -> list[RawIMUSample]:
        """Read up to ``word_count`` FIFO words as complete raw-tick sample tuples.

        Incomplete timestamp/acceleration/gyro groups must be omitted. A driver
        may retain a group split across FIFO reads and return it once complete.
        """

    @abstractmethod
    def discard_FIFO(self) -> None:
        """Discard queued FIFO data and any driver-local partial sample state."""

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
