"""
Simulation-specific configuration, in SI units (m, s, rad).
Tags all run exactly the clamour code, so if you don't see a setting here, it's probably in clamour's config.py
"""
from maps import SquareMap

# Seeds every random draw of the sim (random paths, measurement noise). Each tag draws from its own (SEED, tag_id) stream.
SEED = 0

### Map, any BaseMap from maps/
MAP = SquareMap(side_length=7.0, obstacles=False)

### Anchors {id: (x, y, z)} in m. If empty, clamour's config.ANCHORS is used (converted from cm).
# Anchors must be in the map's free space.
ANCHORS = {}

### Tags {tag_id: spec}. IDs follow clamour's rules: > 10, with unique low bytes (id & 0xFF) below SCHEDULING_SLOT_COUNT (tdma/timing.py).
# Every spec has:     motion, z (fixed height, m), speed (nominal cruise speed, m/s)
# motion="waypoints": waypoints: [(x, y), ...]. Starts at the first waypoint and stops at the last one.
# motion="random":    start: (x, y), n_waypoints: number of random waypoints after start. Stops at the last one.
TAGS = {
    11: {'motion': 'waypoints', 'z': 0.4, 'speed': 0.5, 'waypoints': [(1.0, 1.0), (5.0, 1.5), (5.0, 5.0), (2.0, 5.0)]},
    12: {'motion': 'random', 'z': 0.4, 'speed': 0.5, 'start': (3.0, 3.0), 'n_waypoints': 8},
}

### Motion
# Tags stay still until every tag has published its first estimate (clamour needs them still for IMU init, and
# TDMA sync + scheduling takes tens of seconds before the first ranges), then all start moving after this delay.
MOTION_START_DELAY_S = 2.0
TAG_MAX_ACCEL = 0.5                 # m/s^2, nominal peak acceleration when starting/stopping
RANDOM_SEGMENT_LENGTH = (1.0, 3.0)  # m, (min, max) distance between consecutive random waypoints

### Simulated UWB
UWB_RANGE = 30.0   # m, no range measurement beyond this distance. TODO: gradual reliability drop-out model
UWB_NOISE = 0.15   # m, std dev of the Gaussian range noise

### ----------------------- VALIDATION CHECKS ----------------------- ###
assert UWB_RANGE > 0 and UWB_NOISE >= 0
assert MOTION_START_DELAY_S >= 0 and TAG_MAX_ACCEL > 0
assert 0 < RANDOM_SEGMENT_LENGTH[0] <= RANDOM_SEGMENT_LENGTH[1]
for _tag_id, _spec in TAGS.items():
    assert _spec['motion'] in ('waypoints', 'random'), f"Tag {_tag_id}: unknown motion {_spec['motion']}"
    assert _spec['z'] > 0 and _spec['speed'] > 0, f"Tag {_tag_id}: z and speed must be positive"
