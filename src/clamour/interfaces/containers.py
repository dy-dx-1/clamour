"""Containers that transport spatial and angular tag state within Clamour."""

from multiprocessing import Array

import numpy as np


class Pose:
    """A tag pose with spatial coordinates, Euler angles, and spatial covariance.

    Spatial coordinates ``x``, ``y``, and ``z`` are expressed in centimetres;
    ``heading``, ``roll``, and ``pitch`` are expressed in degrees. All six values
    are stored and returned as integers because the supported interfaces cannot
    provide more precision.

    Covariance is ``None`` by default. To initialize it, pass a six-item tuple
    in the same order accepted by :meth:`update_covar`::

        Pose(x=10, y=20, z=30, covar=(xx, yy, zz, xy, xz, yz))

    The covariance describes the spatial coordinates and is stored as a 3x3
    integer matrix in cm^2.
    """

    def __init__(self, x: int = 0, y: int = 0, z: int = 0,
                 heading: int = 0, roll: int = 0, pitch: int = 0,
                 covar: tuple[int, int, int, int, int, int] | None = None):
        self._coordinates = [int(x), int(y), int(z)]
        self._angles = [int(heading), int(roll), int(pitch)]
        self._covar = None
        if covar is not None:
            self.update_covar(covar)

    def __repr__(self):
        return (f"Pose: ({self.x}, {self.y}, {self.z}) | "
                f"({self.heading}, {self.roll}, {self.pitch}) | Covar: {self._covar}")

    def __str__(self):
        return f"Heading: {self.heading}, Roll: {self.roll}, Pitch: {self.pitch}"

    @property
    def coordinates(self) -> list[int]:
        """Spatial coordinates in ``[x, y, z]`` centimetres."""
        return self._coordinates

    @property
    def angles(self) -> list[int]:
        """Euler angles in ``[heading, roll, pitch]`` degrees."""
        return self._angles

    @property
    def covar(self) -> np.ndarray | None:
        """The 3x3 integer spatial covariance matrix in cm^2, if available."""
        return self._covar

    @covar.setter
    def covar(self, new_matrix: np.ndarray):
        self._covar = new_matrix

    def update_covar(self, covariances: tuple[int, int, int, int, int, int]):
        """Set covariance from ``(xx, yy, zz, xy, xz, yz)``."""
        xx, yy, zz, xy, xz, yz = covariances
        self._covar = np.array([
            [int(xx), int(xy), int(xz)],
            [int(xy), int(yy), int(yz)],
            [int(xz), int(yz), int(zz)],
        ])

    def load(self, coordinates: list[int], angles: list[int] | None = None):
        """Replace spatial coordinates and, optionally, Euler angles."""
        self._coordinates = [int(value) for value in coordinates]
        if angles is not None:
            self._angles = [int(value) for value in angles]

    @property
    def x(self):
        return self._coordinates[0]

    @x.setter
    def x(self, value):
        self._coordinates[0] = int(value)

    @property
    def y(self):
        return self._coordinates[1]

    @y.setter
    def y(self, value):
        self._coordinates[1] = int(value)

    @property
    def z(self):
        return self._coordinates[2]

    @z.setter
    def z(self, value):
        self._coordinates[2] = int(value)

    @property
    def heading(self):
        return self._angles[0]

    @heading.setter
    def heading(self, value):
        self._angles[0] = int(value)

    @property
    def roll(self):
        return self._angles[1]

    @roll.setter
    def roll(self, value):
        self._angles[1] = int(value)

    @property
    def pitch(self):
        return self._angles[2]

    @pitch.setter
    def pitch(self, value):
        self._angles[2] = int(value)


class SharedPose:
    """Process-shared storage for one :class:`Pose`.

    Clamour hands one ``Tag`` object to several processes (estimator, TDMA), and each
    process gets its own copy of it. A plain ``Pose`` attribute would therefore only
    change in the process that wrote it. This buffer lives in shared memory, so a pose
    written by the estimator is the pose the TDMA process reads (e.g. to answer a
    ranging request).

    Create it before the child processes start (i.e. in the tag's ``__init__``).
    Reads and writes are atomic through the buffer's own lock. ``read`` returns a copy:
    modifying it does not change the shared pose, call ``write`` instead.
    """

    # Layout: x, y, z, heading, roll, pitch, has_covar, xx, yy, zz, xy, xz, yz
    _SIZE = 13

    def __init__(self, pose: Pose | None = None):
        self._buffer = Array('q', self._SIZE)
        self.write(pose if pose is not None else Pose())

    def read(self) -> Pose:
        with self._buffer.get_lock():
            values = self._buffer[:]
        covar = tuple(values[7:]) if values[6] else None
        return Pose(*values[:6], covar=covar)

    def write(self, pose: Pose) -> None:
        c = pose.covar
        covar_terms = [0] * 7 if c is None else [1, c[0][0], c[1][1], c[2][2], c[0][1], c[0][2], c[1][2]]
        values = [int(value) for value in (*pose.coordinates, *pose.angles, *covar_terms)]
        with self._buffer.get_lock():
            self._buffer[:] = values
