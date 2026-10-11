"""Tests for the wall-following geometry and closed-loop behaviour."""

import math

from diffbot_control.pid import PID
from diffbot_control.wall_follow_logic import (
    LEFT, RIGHT, wall_geometry, WallFollower, WallFollowParams,
)
import pytest
from sim_helpers import distance_to_segment, scan, unicycle


def make(side=RIGHT, desired=0.5):
    params = WallFollowParams(side=side, desired_distance=desired)
    pid = PID(2.5, 0.1, 0.15, output_limits=(-1.5, 1.5), integral_limits=(-0.75, 0.75),
              derivative_tau=0.05)
    return WallFollower(params, pid)


@pytest.mark.parametrize('alpha_deg', [-30.0, -10.0, 0.0, 15.0, 40.0])
def test_geometry_recovers_wall_angle_and_distance(alpha_deg):
    """Build beam ranges for a known wall and check we recover alpha and D."""
    alpha = math.radians(alpha_deg)
    theta = math.radians(50.0)
    distance = 0.7
    # Right-hand wall rotated by alpha: perpendicular distance D.
    b = distance / math.cos(alpha)
    a = distance / math.cos(theta + alpha)
    est_alpha, est_d = wall_geometry(a, b, theta)
    assert est_alpha == pytest.approx(alpha, abs=1e-9)
    assert est_d == pytest.approx(distance, abs=1e-9)


# Start 0.9 m (too far) or 0.25 m (too close) from the wall; target is 0.5 m.
@pytest.mark.parametrize(
    'side,start_y', [(RIGHT, -0.1), (RIGHT, -0.75), (LEFT, 0.1), (LEFT, 0.75)]
)
def test_converges_to_desired_distance_along_straight_wall(side, start_y):
    """A long wall along the x-axis (y=-1 for right side, y=+1 for left)."""
    wall_y = -1.0 if side == RIGHT else 1.0
    wall = [((-5.0, wall_y), (60.0, wall_y))]
    ctrl = make(side=side)
    x, y, yaw, dt = 0.0, start_y, 0.0, 0.05
    for _ in range(int(40.0 / dt)):
        out = ctrl.step(scan(x, y, yaw, wall), dt)
        x, y, yaw = unicycle(x, y, yaw, out.linear, out.angular, dt)
    assert abs(y - wall_y) == pytest.approx(0.5, abs=0.03)
    assert abs(math.atan2(math.sin(yaw), math.cos(yaw))) < math.radians(5.0)


def test_follows_walls_of_a_room_without_collision():
    """Drive several laps around a 6 m x 4 m room following the right wall."""
    w, h = 3.0, 2.0
    room = [((-w, -h), (w, -h)), ((w, -h), (w, h)), ((w, h), (-w, h)), ((-w, h), (-w, -h))]
    ctrl = make(side=RIGHT)
    x, y, yaw, dt = 0.0, -h + 0.5, 0.0, 0.05
    closest = float('inf')
    travelled = 0.0
    for _ in range(int(120.0 / dt)):
        out = ctrl.step(scan(x, y, yaw, room), dt)
        nx, ny, yaw = unicycle(x, y, yaw, out.linear, out.angular, dt)
        travelled += math.hypot(nx - x, ny - y)
        x, y = nx, ny
        closest = min(closest, min(distance_to_segment(x, y, s) for s in room))
    assert closest > 0.2          # robot radius is ~0.17 m: no collision
    # The path 0.5 m inside the walls is 5 m x 3 m = 16 m long: >1.5 laps.
    assert travelled > 24.0


def test_wraps_around_outside_corners_without_clipping():
    """Room with a box sticking out of one wall: two outside corners to wrap."""
    w, h = 3.0, 2.0
    room = [((-w, -h), (w, -h)), ((w, -h), (w, h)), ((w, h), (-w, h)), ((-w, h), (-w, -h))]
    # 1 m x 1 m box attached to the east wall
    box = [((2.0, -0.5), (3.0, -0.5)), ((2.0, -0.5), (2.0, 0.5)), ((2.0, 0.5), (3.0, 0.5))]
    segments = room + box
    ctrl = make(side=RIGHT)
    x, y, yaw, dt = 0.0, -h + 0.5, 0.0, 0.05
    closest = float('inf')
    travelled = 0.0
    for _ in range(int(150.0 / dt)):
        out = ctrl.step(scan(x, y, yaw, segments), dt)
        nx, ny, yaw = unicycle(x, y, yaw, out.linear, out.angular, dt)
        travelled += math.hypot(nx - x, ny - y)
        x, y = nx, ny
        closest = min(closest, min(distance_to_segment(x, y, s) for s in segments))
    assert closest > 0.3          # keeps well clear of the box corners
    assert travelled > 24.0


def test_turns_away_from_wall_in_front():
    wall_ahead = [((0.4, -5.0), (0.4, 5.0))]
    out = make(side=RIGHT).step(scan(0.0, 0.0, 0.0, wall_ahead), 0.05)
    assert out.mode == 'corner'
    assert out.linear == 0.0
    assert out.angular > 0.0      # following the right wall -> turn left


def test_corner_mode_has_hysteresis():
    ctrl = make(side=RIGHT)
    assert ctrl.step(scan(0.0, 0.0, 0.0, [((0.5, -5.0), (0.5, 5.0))]), 0.05).mode == 'corner'
    # Slightly beyond the stop distance, but inside the hysteresis band: keep turning.
    assert ctrl.step(scan(0.0, 0.0, 0.0, [((0.6, -5.0), (0.6, 5.0))]), 0.05).mode == 'corner'
    assert ctrl.step(scan(0.0, 0.0, 0.0, [((0.8, -5.0), (0.8, 5.0))]), 0.05).mode != 'corner'


def test_searches_when_no_wall_visible():
    out = make(side=LEFT).step(scan(0.0, 0.0, 0.0, []), 0.05)
    assert out.mode == 'search'
    assert out.angular > 0.0      # arc towards the left


def test_invalid_side():
    with pytest.raises(ValueError):
        WallFollower(WallFollowParams(side=0), PID(1.0))
