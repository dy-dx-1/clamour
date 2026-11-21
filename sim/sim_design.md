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
| 2 | Radio fidelity | **Abstract.** Only estimation is evaluated. Any reachable target answers a ranging request; broadcasts are delivered to reachable tags. No collisions, no busy/not-listening responders. |
| 3 | Motion | Config option for **both** scripted waypoints and random smooth paths. |
| 4 | IMU | **Idealized** IMU (bias + white noise), as a new clamour interface `simulated_imu`. |
| 5 | Output | **Linux terminal output first.** GUI later (probably matplotlib via port forwarding). |
| 6 | Clamour changes | Allowed, but **every change touching `src/clamour` must be approved first.** Simulated interfaces (`SimulationTag`, `simulated_imu`) live in clamour. |
| 7 | Scale | ≤ 5 tags for initial testing. |
| 8 | Ranging model | Gaussian noise, max range, fully opaque walls. A richer model (NLOS bias, attenuation, dropouts) comes later. |
| 9 | Units | **The sim works in SI units (metres, seconds, rad).** Clamour will move to metres later. Until then, convert only at the clamour boundary, inside `SimulationTag` (m → integer cm) and `simulated_imu` (m/s² → mg, rad/s → mdps). Removing those conversions is the only sim change needed when clamour switches. |

## Priority 0 — Tag pose not shared between processes (clamour bug)

**Must be fixed in clamour first. It should never happen, in the sim or on hardware.**

- The `Tag` object is created in `Clamour.start()` and handed to the estimator and TDMA processes, so each process has **its own copy**.
- The estimator writes the posterior with `self.tag.pose = pose` (`state_estimation/estimator.py`, `_publish_pose`). That only updates
  the estimator process's copy.
- The TDMA process reads the pose from its own copy, which keeps the default `Pose()` (covariance `None`):
  - `BitcrazeTag.respond_to_ranging_exchange()` runs in the TDMA process (via `Messenger.receive_new_message` → `tag.receive_data()`),
    so it always sends a zero covariance. The requesting tag drops the neighbor info, and `Task.collect_ranges()` then **discards every
    inter-tag range**.
  - `Task.collect_ranges()` sends `yaw=self.tag.pose.heading`, which is therefore always 0.
- Found by reading the code; not yet confirmed on hardware.

**Status: implemented 2026-11-21, not yet tested on hardware.**
- `SharedPose` (`interfaces/containers.py`): a `multiprocessing.Array` holding x, y, z, heading, roll, pitch, a covariance
  flag and 6 covariance terms, with its own lock (atomic reads/writes).
- `BitcrazeTag` and `SyntheticTag` create one in `__init__` (before the child processes start). Their `pose` getter/setter
  read/write it. The getter returns a copy.
- `PozyxTag` also uses `SharedPose` now (it used to store the pose on the device over serial, which would race with TDMA
  once the tag lock was dropped, see finding 3).
- The `Tag` docstrings now state the requirement, and that `pose` is accessed without the tag lock.
- `SimulationTag` must use `SharedPose` too. The sim must **not** hide this kind of bug behind a world-side store.

Hardware test: with 2 Bitcraze tags, check that inter-tag ranges reach the estimator (they were all dropped before), and that
`measured_yaw` in range updates is no longer always 0.

### Audit of other per-process tag state (2026-11-21)

Tag state is fine wherever only one process uses it. Ownership today:
- **Estimator process:** `pose` (write) and `tag_id` (immutable).
- **TDMA process:** everything else (`_active_tags`, `_available_anchors`, `_twr_seq`, the DW1000 handle, `pose` read).
- `Neighborhood`, `SlotAssignment`, `Messenger` and `Anchors` are built per process and only used by TDMA (or are read-only config).

Findings, all fixed 2026-11-21:
1. **Start method → `fork` pinned** in `ContextManagedProcess` (`get_context('fork')`), which every clamour process goes through.
   Python 3.14 switches the Linux default to `forkserver`, which would have to pickle the tag and its hardware handles.
   It is pinned there and not with `set_start_method`, so the sim launcher can use its own `get_context("spawn")` for tag
   processes. Note: `fork` doesn't exist on Windows, so importing clamour there fails (fine: it targets Linux).
2. **`add_neighbor` was missing `self`** in `Tag`, `PozyxTag` and `SyntheticTag` (a `TypeError` when `Messenger` called it). Added.
3. **Estimator no longer takes the tag lock** for the pose write in `_publish_pose` (`SharedPose` has its own lock; the TDMA
   process holds the tag lock through DW listen timeouts). A comment in `estimator.py` explains why.
   - `PozyxTag` was switched to `SharedPose` so its pose write no longer touches the serial port.
   - `StateEstimator.tag_lock` is now unused.

## Architecture

```
sim.py (main process)
 ├─ World server ── GT trajectories, map, anchors, measurement generation, estimate store
 ├─ snapshot loop ── samples GT + estimates for terminal output / logs (does NOT drive time)
 └─ per tag: spawned process → patch clamour.config → import clamour → Clamour.start()
       ├─ estimator process ── simulated_imu  ──┐
       └─ TDMA process      ── SimulationTag  ──┴─► IPC requests to the World server
```

- **World = function of time.** `gt_state(tag_id, t)` is evaluated when a measurement is requested. There is no stepped world update;
  the `sim.py` loop only takes snapshots.
- **Shared time base.** The world and all tags use the host monotonic clock (`time.monotonic_ns`), on one machine.
- **World as a server.** Tag calls come from grandchild processes, so the world answers requests over IPC (e.g. a `multiprocessing`
  manager or pipes). IPC latency (~100 µs) is small next to 25 ms TDMA slots.
- **Abstract radio** (decision 2):
  - `ranging(target)`: if reachable, return the noisy GT distance (world: metres; `SimulationTag` hands clamour integer cm). For a tag target, also return that tag's *published*
    estimated pose (see Priority 0: published by the tag itself, not read from a shortcut).
  - `broadcast(payload)`: put the payload in the receive queue of every reachable tag.
  - `receive_data()`: pop from this tag's queue.
  - `get_device_list()` / `available_anchors` / `active_tags`: devices reachable per the world.
- **Ranging should take realistic time** (a few ms per exchange), because TDMA slot logic depends on how long ranging takes.

## Clamour changes (each needs approval before implementation)

1. ~~Priority 0 fix: shared pose.~~ Done (see above).
2. `interfaces/simulation_tag.py`: `SimulationTag(Tag)` that talks to the World server.
3. `interfaces/simulated_imu.py`: idealized IMU following the `IMU` contract.
4. `"Simulation"` case in the `TAG_TYPE` / `IMU_TYPE` `match` blocks of `clamour.py`, and the matching config validation.
5. A way for a tag process to know how to reach the World server (address / tag id), e.g. config values patched by the sim.

## Things not to forget

- **Config is read at import.** Modules do `from .config import X`, and `TAG_FACTORY` is chosen when `clamour.clamour` is imported.
  Each tag process must use the `spawn` start method and patch `clamour.config` attributes *before* importing `clamour.clamour`.
- **Anchors must match.** The world and every tag must use the same anchor set (`sim/config.py` overrides `clamour.config.ANCHORS`).
  Anchor IDs are 1–10.
- **Tag ID constraints.** `TAG_ID > 10` and `TAG_ID & 0xFF < SCHEDULING_SLOT_COUNT (20)`, with unique low bytes.
- **Units.** The sim is in metres (decision 9). Clamour is still integer cm, 3D. The map is 2D, with a grid resolution of 1 cm
  (given in metres in the map). Use a fixed height per tag, 3D distances for ranges, and a 2D line-of-sight check on the grid.
  Anchors from `clamour.config` (cm) are converted to metres when the world loads them.
- **Startup requirements:**
  - The IMU init needs the tag **stationary for ~2 s** (otherwise `_initialize_imu` raises an error).
  - The estimator only starts after a ranging batch with **≥ 4 anchor ranges**.
  - Scenarios must start with the tags still and in view of ≥ 4 anchors.
- **Smooth trajectories.** The IMU needs continuous position, velocity, acceleration and heading. Use splines through waypoints, and
  smooth random paths. No instantaneous bounces.
- **IMU contract details** (even if idealized):
  - Gravity on z.
  - Accel in mg and gyro in mdps, *without* bias correction.
  - Raw tick timestamps with `timestamp_tick_ns` and `timestamp_counter_bits`.
  - FIFO semantics: `get_FIFO_count`, `read_FIFO`, `discard_FIFO`, generating samples at `sample_rate_hz` between reads.
  - `get_attitude()` returns (yaw, pitch, roll).
  - Covariance densities.
- **Shutdown.** `Clamour.start()` never returns. The sim must terminate each tag's whole process tree cleanly.
- **CPU budget.** About 2 busy processes per tag, plus GTSAM. Check that TDMA timing stays sane at 5 tags (cores vs. tags).
- **Runs on Linux.** gtsam isn't installed on the Windows dev machine.
- **Pose output.** `pose_callback` runs in the estimator process. Under `spawn` it must be picklable (no lambdas); simplest is a
  top-level function that forwards to the World server / a queue.
- **Small draft fixes:** `sim/maps/square.py` should use `from .map_contract import BaseMap`; `side_length` in metres; `world.Tag`
  needs float positions and shouldn't be confused with clamour's `Tag`.

## Implementation plan

0. ~~**Fix the Priority 0 pose bug in clamour.**~~ Done; hardware test pending. Decide on the audit findings.
1. **Sim config & maps.** Finish `sim/config.py` (metres, tags with initial pose, height and motion mode, anchors, map, seed, UWB
   params). Fix `SquareMap` and add obstacle support.
2. **Trajectories.** Waypoint splines + smooth random paths, giving position, velocity, acceleration and heading at any `t`, with an
   initial stationary period.
3. **World.** GT evaluation, LOS raycast on the grid, range model (noise, max range, opaque walls), per-tag message queues,
   estimate store, seeded RNG.
4. **World server / IPC** and the client API used by the sim interfaces.
5. **`SimulationTag`** in clamour (approval needed).
6. **`simulated_imu`** in clamour (approval needed).
7. **Clamour hooks**: `"Simulation"` tag/IMU types + config validation (approval needed).
8. **`sim.py`**: spawn tag processes with config overrides, run the snapshot loop, terminal summary (GT vs estimate error per
   tag), clean shutdown.
9. **Logging**: GT + estimates to file for offline metrics (RMSE etc.).
10. **Validation**: single static tag → single moving tag → 5 tags with inter-tag ranging.

## Future work

- GUI (matplotlib over port forwarding or similar): map, GT, estimates.
- Richer ranging model: NLOS bias, partially attenuating obstacles (grid values between 0 and 1), dropouts.
- Pedometer simulation, once clamour supports it.
- Possibly protocol-level radio (busy responders, collisions) if TDMA itself needs evaluating.
