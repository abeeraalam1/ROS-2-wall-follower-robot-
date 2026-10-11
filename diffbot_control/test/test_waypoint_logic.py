"""Tests for the go-to-waypoint controller."""

import math

from diffbot_control.pid import PID
from diffbot_control.waypoint_logic import (
    parse_waypoints, WaypointFollower, WaypointParams, wrap_angle, yaw_from_quaternion,
)
import pytest
from sim_helpers import unicycle


def make(waypoints, loop=False):
    params = WaypointParams(loop=loop)
    heading = PID(3.0, 0.0, 0.3, output_limits=(-1.5, 1.5), derivative_tau=0.05)
    distance = PID(0.8, 0.05, 0.1, output_limits=(0.0, 0.4), integral_limits=(0.0, 0.2),
                   derivative_tau=0.05)
    return WaypointFollower(waypoints, params, heading, distance)


def test_wrap_angle():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert wrap_angle(0.3) == pytest.approx(0.3)


def test_yaw_from_quaternion():
    half = math.radians(30.0) / 2
    assert yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(
        math.radians(30.0))


def test_parse_waypoints():
    assert parse_waypoints([1.0, 2.0, 3.0, 4.0]) == [(1.0, 2.0), (3.0, 4.0)]
    with pytest.raises(ValueError):
        parse_waypoints([1.0, 2.0, 3.0])


def test_visits_all_waypoints_in_order():
    waypoints = [(2.5, 0.0), (2.5, 2.0), (0.5, 2.5), (-1.0, 1.0), (0.0, 0.0)]
    ctrl = make(waypoints)
    x, y, yaw, dt = 0.0, 0.0, 0.0, 0.02
    reached = []
    for _ in range(int(90.0 / dt)):
        out = ctrl.step(x, y, yaw, dt)
        if out.index not in reached:
            reached.append(out.index)
        if out.done:
            break
        x, y, yaw = unicycle(x, y, yaw, out.linear, out.angular, dt)
    assert ctrl.done
    assert reached == list(range(len(waypoints)))
    assert math.hypot(x, y) < WaypointParams().goal_tolerance


def test_turns_in_place_when_goal_is_behind():
    ctrl = make([(-2.0, 0.0)])
    out = ctrl.step(0.0, 0.0, 0.0, 0.02)
    assert out.linear == pytest.approx(0.0, abs=1e-9)
    assert abs(out.angular) > 0.5


def test_loop_mode_restarts():
    ctrl = make([(0.05, 0.0), (0.0, 0.05)], loop=True)
    ctrl.step(0.05, 0.0, 0.0, 0.02)   # reach first
    ctrl.step(0.0, 0.05, 0.0, 0.02)   # reach second -> back to first
    assert ctrl.index == 0 and not ctrl.done


def test_empty_waypoints_raise():
    with pytest.raises(ValueError):
        make([])
