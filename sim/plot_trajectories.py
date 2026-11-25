"""
Sanity check of the configured world: map, anchors and tag trajectories (no clamour processes involved).
Prints per-tag motion stats and saves the map with every path to trajectories.png.
Run from the repo root: python sim/plot_trajectories.py
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Headless: works over SSH
import matplotlib.pyplot as plt

import config
from world import World

SAMPLE_PERIOD_S = 0.01

def main() -> None:
    world = World.from_config(config)
    width, height = world.map.size
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(world.map.grid, origin='lower', extent=(0, width, 0, height), cmap='Greys', vmin=0, vmax=1)

    for anchor_id, position in world.anchors.items():
        ax.plot(*position[:2], 'k^')
        ax.annotate(str(anchor_id), position[:2], textcoords='offset points', xytext=(4, 4))

    for tag_id, trajectory in world.trajectories.items():
        states = [trajectory.state(t) for t in np.arange(-1.0, trajectory.duration + 1.0, SAMPLE_PERIOD_S)]
        positions = np.array([s.position for s in states])
        print(f"Tag {tag_id}: {trajectory.length:.2f} m in {trajectory.duration:.1f} s | "
              f"max speed {max(np.linalg.norm(s.velocity) for s in states):.2f} m/s, "
              f"max accel {max(np.linalg.norm(s.acceleration) for s in states):.2f} m/s^2, "
              f"max yaw rate {np.rad2deg(max(abs(s.yaw_rate) for s in states)):.1f} deg/s")
        ax.plot(positions[:, 0], positions[:, 1], label=f"Tag {tag_id}")
        ax.plot(*positions[0, :2], 'o', color=ax.lines[-1].get_color())

    ax.set(xlabel='x (m)', ylabel='y (m)', title='Simulated world (^ = anchor, o = tag start)')
    ax.legend()
    fig.savefig('trajectories.png', dpi=120)
    print("Saved trajectories.png")

if __name__ == '__main__':
    main()
