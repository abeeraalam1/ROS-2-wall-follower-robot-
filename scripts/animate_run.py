#!/usr/bin/env python3
"""Top-down animated GIF of a recorded run (ground-truth pose over the course map).

    python3 scripts/animate_run.py logs/wall_traj.csv docs/media/wall_follow.gif
"""

import math
import sys

import matplotlib
matplotlib.use('Agg')
from matplotlib import animation  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

from plot_results import BLUE, draw_walls, INK, ORANGE, read, style  # noqa: E402


def robot_outline(x, y, yaw, length=0.30, width=0.22):
    corners = [(-length / 2, -width / 2), (length / 2, -width / 2),
               (length / 2, width / 2), (-length / 2, width / 2)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in corners]


def main(src, dst, speedup=4.0, fps=15):
    traj = read(src)
    t0 = traj['t'][0]
    step = max(1, int(round(speedup / fps / (traj['t'][1] - traj['t'][0]))))
    idx = list(range(0, len(traj['t']), step))

    fig, ax = plt.subplots(figsize=(7, 5))
    style(ax, f'Gazebo run, ground truth ({speedup:.0f}x speed)')
    draw_walls(ax)
    (path,) = ax.plot([], [], color=BLUE, lw=2)
    body = plt.Polygon(robot_outline(0, 0, 0), color=ORANGE, zorder=3)
    ax.add_patch(body)
    (heading,) = ax.plot([], [], color=INK, lw=2, zorder=4)
    label = ax.text(0.99, 0.02, '', transform=ax.transAxes, ha='right', color=INK)
    fig.tight_layout()

    def update(k):
        i = idx[k]
        x, y, yaw = traj['true_x'][i], traj['true_y'][i], traj['true_yaw'][i]
        path.set_data(traj['true_x'][:i + 1], traj['true_y'][:i + 1])
        body.set_xy(robot_outline(x, y, yaw))
        heading.set_data([x, x + 0.2 * math.cos(yaw)], [y, y + 0.2 * math.sin(yaw)])
        label.set_text(f't = {traj["t"][i] - t0:5.1f} s')
        return path, body, heading, label

    anim = animation.FuncAnimation(fig, update, frames=len(idx), blit=True)
    anim.save(dst, writer=animation.PillowWriter(fps=fps), dpi=70)
    print('wrote', dst)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
