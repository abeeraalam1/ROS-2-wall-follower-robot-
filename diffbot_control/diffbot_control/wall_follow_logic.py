"""Wall-following controller logic (ROS-free so it can be unit tested).

Geometry
--------
Two lidar beams look at the wall on the followed side:

* ``b`` points straight sideways (90 deg from the heading),
* ``a`` points ``theta`` degrees further forward.

For a straight wall, the angle ``alpha`` between the robot's heading and the
wall, and the perpendicular distance ``D`` to the wall, are::

    alpha = atan2(a*cos(theta) - b, a*sin(theta))
    D     = b * cos(alpha)

Because the robot moves while the controller reacts, we control the distance
the robot *will* have after travelling a look-ahead distance ``L``::

    D_future = D + L * sin(alpha)

``alpha < 0`` means the robot is heading towards the wall, so ``D_future``
shrinks before ``D`` does. This look-ahead acts like extra damping and lets
the PID react to the wall angle, not just the distance.

The two-beam estimate assumes one straight wall. Around an outside corner the
beams hit different faces, the look-ahead overestimates the distance and the
robot cuts the corner. As a safety net we also take the closest return in a
wide sector on the followed side (``side_min``) and control on
``min(D_future, side_min + side_margin)``, so anything clearly closer than the
line estimate wins. The small margin matters: the minimum of ~100 noisy beams
is biased low, and without it the robot would sit a few cm too far away.

Control law
-----------
    error   = desired_distance - min(D_future, side_min + side_margin)
    u       = PID(error)
    omega   = -side * u                            (side: +1 left, -1 right)
    v       = v_max scaled down by |error| and by obstacles in front

Two simple behaviours handle situations the PID alone cannot:

* **Inside corner** (wall in front closer than ``front_stop_distance``):
  stop and turn in place away from the wall. A small hysteresis band keeps
  the robot turning until the front is clearly free, so it does not chatter
  between "corner" and "follow" on every scan.
* **Lost wall** (no valid side return, e.g. an outside corner): drive an
  arc of radius ``desired_distance`` towards the followed side, which wraps
  the robot around the corner and onto the next wall.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from diffbot_control.pid import clamp, PID

LEFT = 1
RIGHT = -1


@dataclass
class ScanView:
    """The subset of ``sensor_msgs/LaserScan`` the controller needs."""

    angle_min: float
    angle_increment: float
    ranges: Sequence[float]
    range_min: float = 0.0
    range_max: float = float('inf')

    def range_at(self, angle: float, window: int = 2) -> Optional[float]:
        """Median of valid ranges within ``window`` beams of ``angle`` (rad).

        Returns ``None`` if the angle is outside the scan or no beam is valid.
        Using a small median makes a single noisy or missing beam harmless.
        """
        n = len(self.ranges)
        if n == 0 or self.angle_increment == 0.0:
            return None
        idx = int(round((angle - self.angle_min) / self.angle_increment))
        if idx < 0 or idx >= n:
            return None
        values = [
            r
            for r in self.ranges[max(0, idx - window):min(n, idx + window + 1)]
            if math.isfinite(r) and self.range_min <= r <= self.range_max
        ]
        if not values:
            return None
        values.sort()
        return values[len(values) // 2]

    def min_in_sector(self, center: float, half_width: float) -> float:
        """Smallest valid range in ``[center - half_width, center + half_width]``."""
        best = float('inf')
        for i, r in enumerate(self.ranges):
            angle = self.angle_min + i * self.angle_increment
            if abs(_wrap(angle - center)) <= half_width and math.isfinite(r):
                if self.range_min <= r <= self.range_max:
                    best = min(best, r)
        return best


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def wall_geometry(a: float, b: float, theta: float) -> Tuple[float, float]:
    """Return ``(alpha, D)``: wall angle [rad] and perpendicular distance [m]."""
    alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
    return alpha, b * math.cos(alpha)


@dataclass
class WallFollowParams:
    side: int = RIGHT
    desired_distance: float = 0.5
    lookahead: float = 0.4
    beam_angle: float = math.radians(50.0)
    max_linear_speed: float = 0.35
    min_linear_speed: float = 0.08
    max_angular_speed: float = 1.5
    front_stop_distance: float = 0.55
    front_slow_distance: float = 1.2
    corner_hysteresis: float = 0.15
    front_half_width: float = math.radians(20.0)
    side_half_width: float = math.radians(50.0)
    side_margin: float = 0.05
    lost_wall_distance: float = 2.5
    search_speed_ratio: float = 0.6
    error_speed_scale: float = 2.0


@dataclass
class WallFollowOutput:
    linear: float
    angular: float
    error: float
    mode: str
    distance: float = float('nan')
    alpha: float = float('nan')


class WallFollower:
    """Turns a laser scan into a ``(v, omega)`` command using a PID."""

    def __init__(self, params: WallFollowParams, pid: PID) -> None:
        if params.side not in (LEFT, RIGHT):
            raise ValueError('side must be LEFT (+1) or RIGHT (-1)')
        self.params = params
        self.pid = pid
        self._mode = 'follow'

    def step(self, scan: ScanView, dt: float) -> WallFollowOutput:
        p = self.params
        s = p.side

        front = scan.min_in_sector(0.0, p.front_half_width)

        # Inside corner: the PID cannot fix "wall straight ahead", so rotate.
        corner_exit = p.front_stop_distance
        if self._mode == 'corner':
            corner_exit += p.corner_hysteresis
        if front < corner_exit:
            self._set_mode('corner')
            return WallFollowOutput(0.0, -s * p.max_angular_speed, 0.0, 'corner')

        b = scan.range_at(s * math.pi / 2.0)
        a = scan.range_at(s * (math.pi / 2.0 - p.beam_angle))

        # Lost wall: arc towards the followed side until it reappears.
        if b is None or b > p.lost_wall_distance:
            self._set_mode('search')
            v = p.max_linear_speed * p.search_speed_ratio
            return WallFollowOutput(v, s * v / p.desired_distance, 0.0, 'search')

        self._set_mode('follow')
        if a is None:
            # Forward beam sees nothing (e.g. wall ends ahead): assume parallel.
            alpha, distance = 0.0, b
        else:
            alpha, distance = wall_geometry(a, b, p.beam_angle)
        future_distance = distance + p.lookahead * math.sin(alpha)
        side_min = scan.min_in_sector(s * math.pi / 2.0, p.side_half_width)
        error = p.desired_distance - min(future_distance, side_min + p.side_margin)

        u = self.pid.update(error, dt)
        angular = clamp(-s * u, (-p.max_angular_speed, p.max_angular_speed))

        # Slow down when the error is large or something is ahead: turning
        # tightly at full speed is what makes wall followers crash.
        speed = p.max_linear_speed / (1.0 + p.error_speed_scale * abs(error))
        if front < p.front_slow_distance:
            slow_band = p.front_slow_distance - p.front_stop_distance
            ratio = (front - p.front_stop_distance) / slow_band
            speed *= clamp(ratio, (0.0, 1.0))
        speed = max(speed, p.min_linear_speed)

        return WallFollowOutput(speed, angular, error, 'follow', distance, alpha)

    def _set_mode(self, mode: str) -> None:
        # Reset the PID when we come back to following so integral and
        # derivative state from before the corner do not cause a jolt.
        if mode == 'follow' and self._mode != 'follow':
            self.pid.reset()
        self._mode = mode
