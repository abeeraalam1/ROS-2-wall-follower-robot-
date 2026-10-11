"""A tiny 2D simulator (unicycle robot + ray-cast lidar) for closed-loop tests."""

import math

from diffbot_control.wall_follow_logic import ScanView


def raycast(x, y, angle, segments, max_range=10.0):
    dx, dy = math.cos(angle), math.sin(angle)
    best = float('inf')
    for (x1, y1), (x2, y2) in segments:
        ex, ey = x2 - x1, y2 - y1
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-12:
            continue
        t = ((x1 - x) * ey - (y1 - y) * ex) / denom
        u = ((x1 - x) * dy - (y1 - y) * dx) / denom
        if t > 0.0 and 0.0 <= u <= 1.0:
            best = min(best, t)
    return best if best <= max_range else float('inf')


def scan(x, y, yaw, segments, beams=360):
    inc = 2.0 * math.pi / beams
    angle_min = -math.pi
    ranges = [raycast(x, y, yaw + angle_min + i * inc, segments) for i in range(beams)]
    return ScanView(angle_min, inc, ranges, 0.12, 10.0)


def unicycle(x, y, yaw, v, w, dt):
    return x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt


def distance_to_segment(px, py, seg):
    (x1, y1), (x2, y2) = seg
    ex, ey = x2 - x1, y2 - y1
    t = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey)))
    return math.hypot(px - (x1 + t * ex), py - (y1 + t * ey))
