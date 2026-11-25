"""
Smooth ground-truth trajectories for simulated tags, in SI units (m, s, rad).

A trajectory is a geometric path (cubic spline through 2D waypoints, parameterized by chord length u) travelled with a
speed profile u(t): still before t=0, smooth ramp up to the cruise speed, cruise, smooth ramp down to a stop at the last
waypoint, then still forever. t is the time since motion started (the world decides when that is).

Keeping the path separate from its timing makes heading (the path tangent) defined even when stopped, and keeps
velocity, acceleration and yaw rate continuous and zero at rest, which the simulated IMU needs.
Cruise speed and acceleration are nominal: chord length only approximates arc length, so actual values vary slightly.
"""
from dataclasses import dataclass
import numpy as np
from scipy.interpolate import CubicSpline

from maps import BaseMap, RESOLUTION

MAX_TURN_RAD = np.deg2rad(120)    # Random paths: max direction change at a waypoint, avoids near-reversals that loop tightly
RANDOM_WAYPOINT_ATTEMPTS = 100    # Random paths: candidates drawn per waypoint before restarting the path
RANDOM_PATH_ATTEMPTS = 100        # Random paths: full paths drawn before giving up

@dataclass(frozen=True)
class TrajectoryState:
    position: np.ndarray      # (3,) m
    velocity: np.ndarray      # (3,) m/s
    acceleration: np.ndarray  # (3,) m/s^2, kinematic only (no gravity)
    heading: float            # rad, direction of travel, counter-clockwise from +x
    yaw_rate: float           # rad/s

class Trajectory:
    """
    ARGS:
        - waypoints: (N>=2, 2) waypoints in m, consecutive ones distinct. The tag starts at the first and stops at the last.
        - z: fixed height in m
        - speed: nominal cruise speed in m/s
        - max_accel: nominal peak acceleration of the start/stop ramps in m/s^2
        - world_map: the path must stay in its free space, else ValueError
    """
    def __init__(self, waypoints, z: float, speed: float, max_accel: float, world_map: BaseMap):
        waypoints = np.asarray(waypoints, dtype=float)
        if waypoints.ndim != 2 or waypoints.shape[1] != 2 or len(waypoints) < 2:
            raise ValueError("A trajectory needs at least two (x, y) waypoints")
        segment_lengths = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
        if np.any(segment_lengths == 0):
            raise ValueError("Consecutive waypoints must differ")
        knots = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        self._path = CubicSpline(knots, waypoints, bc_type='natural')
        self._path_d1 = self._path.derivative(1)
        self._path_d2 = self._path.derivative(2)
        self.length = knots[-1]
        self.z = z

        # Ramps follow a smoothstep speed V*(3s^2 - 2s^3), s in [0, 1] over ramp_time: acceleration is continuous
        # and peaks at 1.5*V/ramp_time. If the path is too short to reach V, lower V so the two ramps meet.
        cruise_speed = speed
        if 1.5 * cruise_speed**2 / max_accel > self.length:
            cruise_speed = np.sqrt(self.length * max_accel / 1.5)
        self._cruise_speed = cruise_speed
        self._ramp_time = 1.5 * cruise_speed / max_accel
        self.duration = 2 * self._ramp_time + (self.length - cruise_speed * self._ramp_time) / cruise_speed

        samples = np.linspace(0.0, self.length, int(np.ceil(self.length / (RESOLUTION / 2))) + 1)
        if not world_map.are_free(self._path(samples)).all():
            raise ValueError("Trajectory crosses an obstacle or leaves the map")

    def _progress(self, t: float) -> tuple[float, float, float]:
        """Path parameter u and its first two time derivatives at time t."""
        v, ramp = self._cruise_speed, self._ramp_time
        if t <= 0:
            return 0.0, 0.0, 0.0
        if t >= self.duration:
            return self.length, 0.0, 0.0
        if t < ramp:
            s = t / ramp
            return v * ramp * (s**3 - s**4 / 2), v * (3 * s**2 - 2 * s**3), v / ramp * (6 * s - 6 * s**2)
        remaining = self.duration - t
        if remaining < ramp:  # Mirror of the ramp up
            s = remaining / ramp
            return self.length - v * ramp * (s**3 - s**4 / 2), v * (3 * s**2 - 2 * s**3), -v / ramp * (6 * s - 6 * s**2)
        return v * ramp / 2 + v * (t - ramp), v, 0.0

    def state(self, t: float) -> TrajectoryState:
        """Ground truth at t seconds since motion started (t < 0: still at the first waypoint)."""
        u, du, ddu = self._progress(t)
        tangent, curve = self._path_d1(u), self._path_d2(u)
        velocity = tangent * du
        acceleration = curve * du**2 + tangent * ddu
        yaw_rate = (tangent[0] * curve[1] - tangent[1] * curve[0]) / (tangent @ tangent) * du
        return TrajectoryState(position=np.array([*self._path(u), self.z]),
                               velocity=np.array([*velocity, 0.0]),
                               acceleration=np.array([*acceleration, 0.0]),
                               heading=float(np.arctan2(tangent[1], tangent[0])),
                               yaw_rate=float(yaw_rate))

def random_trajectory(start, n_waypoints: int, segment_length: tuple[float, float], z: float, speed: float,
                      max_accel: float, world_map: BaseMap, rng: np.random.Generator) -> Trajectory:
    """
    Random smooth path from start through n_waypoints random waypoints. Consecutive waypoints are segment_length
    (min, max) apart with line of sight, and the direction changes by at most MAX_TURN_RAD at each waypoint.
    """
    for _ in range(RANDOM_PATH_ATTEMPTS):
        waypoints = [np.asarray(start, dtype=float)]
        direction = None
        while len(waypoints) <= n_waypoints:
            for _ in range(RANDOM_WAYPOINT_ATTEMPTS):
                angle = rng.uniform(-np.pi, np.pi) if direction is None else direction + rng.uniform(-MAX_TURN_RAD, MAX_TURN_RAD)
                candidate = waypoints[-1] + rng.uniform(*segment_length) * np.array([np.cos(angle), np.sin(angle)])
                if world_map.is_free(*candidate) and world_map.line_of_sight(waypoints[-1], candidate):
                    waypoints.append(candidate)
                    direction = angle
                    break
            else:
                break  # Stuck (e.g. facing a corner): restart the path
        if len(waypoints) > n_waypoints:
            try:
                return Trajectory(waypoints, z, speed, max_accel, world_map)
            except ValueError:
                pass  # The spline between waypoints clipped an obstacle: draw another path
    raise RuntimeError(f"No valid random path found from {tuple(start)}. Check the start point, segment lengths and map.")

def build_trajectory(spec: dict, max_accel: float, segment_length: tuple[float, float],
                     world_map: BaseMap, rng: np.random.Generator) -> Trajectory:
    """Build a tag's trajectory from its sim config spec (see config.TAGS)."""
    match spec['motion']:
        case 'waypoints':
            return Trajectory(spec['waypoints'], spec['z'], spec['speed'], max_accel, world_map)
        case 'random':
            return random_trajectory(spec['start'], spec['n_waypoints'], segment_length, spec['z'], spec['speed'],
                                     max_accel, world_map, rng)
        case motion:
            raise ValueError(f"Unknown motion type: {motion}")
