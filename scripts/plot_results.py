#!/usr/bin/env python3
"""Make the README figures from CSV logs written by the launch files.

    python3 scripts/plot_results.py --logs logs --out docs/media

Expects (any missing file is skipped):
    wall_follow.csv, wall_traj.csv       from wall_follow.launch.py
    waypoints.csv, waypoints_traj.csv    from waypoints.launch.py
"""

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

BLUE, ORANGE, GRAY, INK, MUTED = '#2a78d6', '#eb6834', '#9a9994', '#0b0b0b', '#52514e'

# Walls of worlds/wall_course.sdf as (center_x, center_y, size_x, size_y).
WALLS = [(0.5, -0.65, 7.2, 0.1), (0.5, 4.05, 7.2, 0.1), (-3.05, 1.7, 0.1, 4.8),
         (4.05, 1.7, 0.1, 4.8), (3.5, 1.9, 1.0, 1.0), (-0.5, 3.6, 1.0, 0.8)]


def read(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) if k != 'mode' else r[k] for r in rows] for k in rows[0]} if rows else None


def style(ax, title):
    ax.set_title(title, loc='left', fontsize=12, color=INK, pad=22)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRAY)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color='#e6e5e1', linewidth=0.6)
    ax.set_axisbelow(True)


def draw_walls(ax):
    for cx, cy, sx, sy in WALLS:
        ax.add_patch(plt.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color='#c9c8c3', lw=0))
    ax.set_aspect('equal')
    ax.set_xlim(-3.4, 4.4)
    ax.set_ylim(-1.0, 4.4)
    ax.set_xlabel('x [m]', color=MUTED)
    ax.set_ylabel('y [m]', color=MUTED)


def trajectory(ax, traj, title):
    style(ax, title)
    draw_walls(ax)
    ax.plot(traj['true_x'], traj['true_y'], color=BLUE, lw=2, label='Ground truth (Gazebo)')
    ax.plot(traj['odom_x'], traj['odom_y'], color=ORANGE, lw=2, ls='--',
            label='Wheel odometry')
    ax.plot(traj['true_x'][0], traj['true_y'][0], 'o', color=INK, ms=8)
    ax.annotate('start', (traj['true_x'][0], traj['true_y'][0]), xytext=(6, 6),
                textcoords='offset points', color=MUTED, fontsize=9)
    ax.legend(frameon=False, fontsize=9, loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=2,
              labelcolor=MUTED)


def wall_figure(logs, out):
    ctrl, traj = read(os.path.join(logs, 'wall_follow.csv')), read(os.path.join(logs, 'wall_traj.csv'))
    if not ctrl or not traj:
        return
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw={'width_ratios': [1.25, 1]})
    trajectory(a1, traj, 'Wall following around the course')
    style(a2, 'Distance error  (desired 0.5 m − measured)')
    t0 = ctrl['t'][0]
    t = [x - t0 for x in ctrl['t']]
    follow = [e if m == 'follow' else math.nan for e, m in zip(ctrl['error'], ctrl['mode'])]
    a2.plot(t, follow, color=BLUE, lw=1.5)
    for ti, m in zip(t, ctrl['mode']):
        if m == 'corner':
            a2.axvspan(ti - 0.05, ti + 0.05, color='#f3d9cc', lw=0)
    a2.axhline(0, color=GRAY, lw=1)
    a2.set_xlabel('time [s]   (shaded: turning in a corner)', color=MUTED)
    a2.set_ylabel('error [m]', color=MUTED)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'wall_follow.png'), dpi=130)
    plt.close(fig)


def waypoint_figure(logs, out, waypoints):
    ctrl, traj = read(os.path.join(logs, 'waypoints.csv')), read(os.path.join(logs, 'waypoints_traj.csv'))
    if not ctrl or not traj:
        return
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw={'width_ratios': [1.25, 1]})
    trajectory(a1, traj, 'Waypoint following: path through 5 goals')
    for i, (x, y) in enumerate(waypoints):
        a1.plot(x, y, marker='X', ms=10, color=INK, ls='none')
        a1.annotate(str(i), (x, y), xytext=(7, -12), textcoords='offset points', color=INK)
    style(a2, 'Heading error to the current goal')
    t0 = ctrl['t'][0]
    t = [x - t0 for x in ctrl['t']]
    done = next((i for i, v in enumerate(ctrl['v']) if t[i] > 1.0 and v == 0.0 and
                 ctrl['distance_error'][i] == 0.0), len(t))
    a2.plot(t[:done + 10], [math.degrees(h) for h in ctrl['heading_error'][:done + 10]],
            color=BLUE, lw=1.5)
    a2.axhline(0, color=GRAY, lw=1)
    a2.set_xlabel('time [s]   (each spike is a new waypoint)', color=MUTED)
    a2.set_ylabel('heading error [deg]', color=MUTED)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'waypoints.png'), dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--logs', default='logs')
    parser.add_argument('--out', default='docs/media')
    parser.add_argument('--waypoints', default='2.5,0,2.5,2,0.5,2.5,-1,1,0,0')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    flat = [float(v) for v in args.waypoints.split(',')]
    wall_figure(args.logs, args.out)
    waypoint_figure(args.logs, args.out, list(zip(flat[::2], flat[1::2])))
    print('figures written to', args.out)


if __name__ == '__main__':
    main()
