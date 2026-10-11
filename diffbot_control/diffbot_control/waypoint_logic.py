"""Go-to-waypoint controller logic (ROS-free so it can be unit tested).

Two independent PID loops drive a differential-drive (unicycle) robot:

* **Heading loop**: error = wrap(bearing_to_goal - yaw)  ->  angular velocity
* **Distance loop**: error = distance_to_goal            ->  linear velocity

The linear command is multiplied by ``max(0, cos(heading_error))``. When the
goal is behind or beside the robot the robot turns (almost) in place first,
and it only speeds up once it is pointing the right way. This coupling is
what stops a naive two-PID controller from driving in big spirals.

A waypoint counts as reached inside ``goal_tolerance``; both PIDs are then
reset so the integral from one leg does not leak into the next.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

from diffbot_control.pid import PID


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about Z) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class WaypointParams:
    goal_tolerance: float = 0.12
    max_linear_speed: float = 0.4
    max_angular_speed: float = 1.5
    loop: bool = False


@dataclass
class WaypointOutput:
    linear: float
    angular: float
    distance_error: float
    heading_error: float
    index: int
    done: bool


class WaypointFollower:
    """Visits a list of ``(x, y)`` waypoints in order."""

    def __init__(
        self,
        waypoints: Sequence[Tuple[float, float]],
        params: WaypointParams,
        heading_pid: PID,
        distance_pid: PID,
    ) -> None:
        if not waypoints:
            raise ValueError('at least one waypoint is required')
        self.waypoints: List[Tuple[float, float]] = list(waypoints)
        self.params = params
        self.heading_pid = heading_pid
        self.distance_pid = distance_pid
        self.index = 0
        self.done = False

    @property
    def current_goal(self) -> Optional[Tuple[float, float]]:
        return None if self.done else self.waypoints[self.index]

    def step(self, x: float, y: float, yaw: float, dt: float) -> WaypointOutput:
        p = self.params
        if self.done:
            return WaypointOutput(0.0, 0.0, 0.0, 0.0, self.index, True)

        gx, gy = self.waypoints[self.index]
        distance = math.hypot(gx - x, gy - y)

        if distance < p.goal_tolerance:
            self._advance()
            if self.done:
                return WaypointOutput(0.0, 0.0, 0.0, 0.0, self.index, True)
            gx, gy = self.waypoints[self.index]
            distance = math.hypot(gx - x, gy - y)

        heading_error = wrap_angle(math.atan2(gy - y, gx - x) - yaw)

        angular = self.heading_pid.update(heading_error, dt)
        linear = self.distance_pid.update(distance, dt)
        linear *= max(0.0, math.cos(heading_error))

        angular = max(-p.max_angular_speed, min(p.max_angular_speed, angular))
        linear = max(0.0, min(p.max_linear_speed, linear))
        return WaypointOutput(linear, angular, distance, heading_error, self.index, False)

    def _advance(self) -> None:
        self.heading_pid.reset()
        self.distance_pid.reset()
        self.index += 1
        if self.index >= len(self.waypoints):
            if self.params.loop:
                self.index = 0
            else:
                self.index = len(self.waypoints) - 1
                self.done = True


def parse_waypoints(flat: Sequence[float]) -> List[Tuple[float, float]]:
    """Turn ``[x1, y1, x2, y2, ...]`` (a ROS parameter) into ``[(x1, y1), ...]``."""
    if len(flat) % 2 != 0:
        raise ValueError('waypoints must contain an even number of values (x, y pairs)')
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)]
