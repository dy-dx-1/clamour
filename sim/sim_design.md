# Clamour Simulation — Design & Plan

Living document: plan, decisions, and things not to forget.

## Goal

Simulate multiple tags moving in a 2D map. Each tag runs the **unmodified clamour stack** (TDMA + estimator processes) behind
simulated hardware interfaces. A `world` holds the ground truth (GT) and produces realistic measurements on request. Tags only see
those measurements and do their own estimation. `sim.py` collects GT and estimates for terminal output (GUI later).

## Decisions

| # | Topic | Decision |
|---|-------|----------|
| 1 | Time model | **Real time, 1:1.** Runs are not bit-for-bit reproducible (OS scheduling, TDMA random backoffs). Seed all sim noise and log GT + estimates so runs can be compared offline. |
| 2 | Radio fidelity | **Abstract.** Only estimation is evaluated. Any reachable target answers a ranging request; broadcasts reach reachable tags. No collisions, no busy responders. |
| 3 | Motion | Config option for **both** scripted waypoints and random smooth paths. Tags stop at their last waypoint; heading = direction of travel. |
| 4 | IMU | **Idealized** IMU (constant biases + white noise), as a new clamour interface `simulated_imu`. |
| 5 | Output | **Linux terminal output first.** GUI later (probably matplotlib via port forwarding). |
| 6 | Clamour changes | Allowed, but **every change touching `src/clamour` must be approved first.** Simulated interfaces (`SimulationTag`, `simulated_imu`) live in clamour. |
| 7 | Scale | ≤ 5 tags for initial testing. |
| 8 | Ranging model | Gaussian noise, max range, fully opaque walls. A richer model (NLOS bias, attenuation, dropouts) comes later. |
| 9 | Units | **SI (m, s, rad) in the sim.** Clamour is still integer cm (moving to metres later). Convert only at the boundary: `SimulationTag` (m → cm) and `simulated_imu` (m/s² → mg, rad/s → mdps). |

## Architecture

```
sim.py (main process)
 ├─ World + server thread ── GT, map, anchors, measurements, published poses, broadcasts
 ├─ snapshot loop ── samples GT + estimates for terminal output / logs (does NOT drive time)
 └─ per tag: spawned process → patch clamour.config → import clamour → Clamour.start()
       ├─ estimator process ── simulated_imu  ──┐
       └─ TDMA process      ── SimulationTag  ──┴─► IPC requests to the World server
```

- ✅ **World** (`sim/world.py`): a function of time with no stepped update, and thread-safe, because the server calls it from
  one thread per client. Time is the host monotonic clock (`time.monotonic()`).
  - **Reachable** = within `UWB_RANGE` (3D distance) and 2D line of sight. The same rule covers ranging, discovery and broadcasts.
  - **Motion** starts when `sim.py` calls `start_motion(MOTION_START_DELAY_S)`, after every tag's first estimate has arrived
    through `pose_callback`. All tags start together.
  - **Noise:** random streams are seeded with `(SEED, tag_id, stream)`.
  - **IMU** (`imu_spec`, `imu_samples`, `imu_attitude`): body frame with x along the heading and z up.
    - Accel = specific force (+g on z at rest) + bias + white noise. Gyro = yaw rate + bias + white noise.
    - Noise densities match the LSM6DSV320X values clamour uses.
    - Each tag's accel bias is drawn from clamour's own prior (`IMU_ACCEL_INITIAL_BIAS_MG ± _STD_MG`); the gyro bias is
      random (`IMU_GYRO_BIAS_STD`).
  - Random paths start facing +x, and the World warns if a scripted path starts more than 2° off (see ⚠️ below).
- ✅ **Server** (`sim/world_server.py`): `serve(world)` runs a multiprocessing `BaseManager` from a daemon thread of sim.py and
  returns its address. sim.py uses the World object directly.
  - **Protocol:** typeid `'world'`, exposed methods listed in `EXPOSED`. They return plain types only (including numpy
    arrays), so clients never need sim modules to unpickle results.
  - **Client:** `connect_world()` in `simulation_tag.py`, so clamour never imports from `sim/`. Only the address has to reach
    the tag process; the authkey is inherited from sim.py. Proxies don't survive a fork, so connect in the process that uses
    them.
- ✅ **`SimulationTag`** (`src/clamour/interfaces/simulation_tag.py`) behaves like `BitcrazeTag`, with no clamour TDMA changes:
  - **Ranging:** `world.range()` gives metres, returned to clamour as integer cm. For a tag target, it also returns that
    tag's last published pose.
  - **Pose publishing:** `receive_data()` (TDMA process) publishes its `SharedPose` in Bitcraze's TWR report layout
    (9 ints, no angles, `xx == 0` = no pose). This mirrors Bitcraze answering ranging from inside `receive_data()`, so
    pose-sharing bugs stay visible.
  - **Listen windows:** `receive_data()` → `world.listen(15 ms)` returns the first frame heard. `broadcast` only reaches
    reachable tags that are listening at that moment, so messages arrive without delay (TDMA sync depends on that) and are
    lost otherwise.
  - **Discovery:** anchors come from `reachable_devices()` ∩ clamour's anchors, cached until `clear_anchors()`. Tags are known
    only from messages heard, and dropped after 15 s of silence.
  - **Timing emulation:** a successful TWR takes 4 ms, a failed one 30 ms. Each anchor poll takes 4 ms if it answers, 15 ms if
    not, plus a 5 ms pause. **TODO:** calibrate these placeholders against real Bitcraze timings.
- ✅ **`SimulatedIMU`** (`src/clamour/interfaces/simulated_imu.py`) follows the `IMU` contract like the LSM6DSV320X:
  - **FIFO:** 256 words, 3 per sample, oldest samples lost on overflow. Samples are generated on demand by the World at the
    IMU's own sample times.
  - **Clock:** 32-bit ticks of nominal 21.7 µs from a random start tick, running `IMU_CLOCK_RATE_ERROR` (0.6%) fast. This
    exercises clamour's `SensorClockMapper`.
  - **Units:** mg / mdps, not bias-corrected. `get_attitude()` gives yaw relative to the start heading, with pitch and roll 0.
  - **Constants:** noise densities come from the World. Scale factor matrices are stored but not applied (an ideal sensor
    needs no correction).
  - It connects to the World right away, so build it inside the estimator process (as `create_imu` does).

## Clamour changes

- ✅ Pose shared between a tag's processes (`SharedPose` in all tags; was silently dropping every inter-tag range).
  **Hardware test pending:** with 2 Bitcraze tags, inter-tag ranges reach the estimator and `measured_yaw` is no longer always 0.
- ✅ `fork` pinned in `ContextManagedProcess` (Python 3.14 defaults to `forkserver`, which can't pickle hardware handles).
- ✅ `add_neighbor` was missing `self` in `Tag`, `PozyxTag` and `SyntheticTag`.
- ✅ Estimator writes the pose without the tag lock (`StateEstimator.tag_lock` is now unused).
- ✅ `interfaces/simulation_tag.py`: `SimulationTag(tag_id, world_address)`, see Architecture.
- ✅ `interfaces/simulated_imu.py`: `SimulatedIMU(tag_id, world_address)`, see Architecture.
- ⚠️ **To fix later in clamour: initial yaw.** ⚠️ The FG's initial yaw prior is 0 ± 2° in the *anchor* frame
  (`_initialize_estimator` assumes "the global frame starts at the initial heading"). A tag that doesn't start facing +x
  integrates its IMU in the wrong direction, and the tight prior resists the correction. This applies on hardware too. Until
  it's fixed, the sim starts random paths facing +x and warns about scripted paths that don't.
- Pending, each needs approval:
  - A `"Simulation"` case in the `TAG_TYPE` / `IMU_TYPE` `match` blocks of `clamour.py`, plus config validation.
  - A way for a tag process to get the World server's address (e.g. a config value patched by the sim).

## Things not to forget

- **Config is read at import** (`from .config import X`; `TAG_FACTORY` is chosen when `clamour.clamour` is imported). Tag processes
  use a `spawn` context and patch `clamour.config` *before* importing `clamour.clamour`. Clamour's own children still fork.
- **Anchors must match** between the world and every tag. `sim/config.py` overrides `clamour.config.ANCHORS`. Anchor IDs are 1–10.
- **Tag IDs:** greater than 10, with unique low bytes (`id & 0xFF`) below `SCHEDULING_SLOT_COUNT` (20).
- **Geometry:** each tag has a fixed height. Ranges are 3D distances; line of sight is a 2D check on the grid.
- **Startup:**
  - The IMU init needs the tag still for ~2 s.
  - The estimator starts only after a batch with ≥ 4 anchor ranges.
  - TDMA ranges only after SYNC (5 s) and scheduling (12–24 s), so the first estimate comes ~20–30 s after start. That's why
    motion waits for every tag's first estimate (see World).
- **Shutdown:** `Clamour.start()` never returns, so the sim must terminate each tag's process tree.
- **CPU:** about 2 busy processes per tag, plus GTSAM. Check that TDMA timing stays sane at 5 tags.
- **Pose output:** `pose_callback` runs in the estimator process and must be picklable under `spawn` (a top-level function).
- **Imports:** sim modules import each other by plain name, so run them as scripts from the repo root (`python sim/sim.py`).
  `sim/config.py` puts `src/` on `sys.path` so clamour is importable.

## Implementation plan

1. ✅ **Config & maps.** `sim/config.py` in metres. `BaseMap` occupancy grid (`grid[iy, ix]`, 1 cm cells) with `are_free` and
   `line_of_sight`. `SquareMap` has walls and an optional interior wall.
2. ✅ **Trajectories** (`sim/trajectory.py`). Spline path + smooth speed profile: velocity, acceleration and yaw rate are continuous
   and zero at rest. Paths are checked against the map; random paths are seeded per tag. Check with `sim/plot_trajectories.py`.
3. ✅ **World** (`sim/world.py`), see Architecture. Anchors fall back to clamour's (cm → m), and it warns if a tag starts with
   fewer than 4 anchors reachable.
4. ✅ **World server** (`sim/world_server.py`), see Architecture.
5. ✅ **`SimulationTag`**, see Architecture.
6. ✅ **`SimulatedIMU`**, see Architecture.
7. **Clamour hooks**: `"Simulation"` tag/IMU types + config validation (approval needed).
8. **`sim.py`**: spawn tag processes with config overrides, run the snapshot loop, print a terminal summary (GT vs estimate error),
   shut down cleanly.
9. **Logging**: GT + estimates to file for offline metrics (RMSE etc.).
10. **Validation**: single static tag → single moving tag → 5 tags with inter-tag ranging.

## Future work

- GUI: map, GT, estimates.
- Richer ranging model: NLOS bias, partially attenuating obstacles (grid values between 0 and 1), dropouts.
- Pedometer simulation, once clamour supports it.
- Protocol-level radio (busy responders, collisions), if TDMA itself needs evaluating.
