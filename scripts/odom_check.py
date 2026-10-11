#!/usr/bin/env python3
"""Drive a fixed manoeuvre and compare wheel odometry with simulator ground truth.

Run while only the simulation is up (no controller), e.g. inside the container:
    ros2 launch diffbot_sim sim.launch.py headless:=true &
    python3 scripts/odom_check.py --ros-args -p use_sim_time:=true
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def yaw(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class OdomCheck(Node):
    # (linear, angular, seconds)
    PLAN = [(0.0, 1.5, 4.0), (0.0, 0.0, 1.0), (0.3, 0.0, 4.0), (0.0, 0.0, 1.0),
            (0.0, -1.5, 4.0), (0.0, 0.0, 1.0)]

    def __init__(self):
        super().__init__('odom_check')
        self.odom = self.truth = None
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(
            Odometry, 'ground_truth/odom', lambda m: setattr(self, 'truth', m), 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.step = 0
        self.start = None
        self.origin = None
        self.create_timer(0.05, self.tick)

    def tick(self):
        if self.odom is None or self.truth is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start = now
            self.origin = (yaw(self.odom), yaw(self.truth))
        v, w, duration = self.PLAN[self.step]
        if now - self.start >= duration:
            if w == 0.0 and v == 0.0:
                self.report()
            self.step += 1
            self.start = now
            if self.step >= len(self.PLAN):
                rclpy.shutdown()
                return
            v, w, duration = self.PLAN[self.step]
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)

    def report(self):
        o, t = self.odom.pose.pose.position, self.truth.pose.pose.position
        self.get_logger().info(
            f'odom: x={o.x:+.3f} y={o.y:+.3f} yaw={math.degrees(yaw(self.odom)):+7.1f} | '
            f'truth: x={t.x:+.3f} y={t.y:+.3f} yaw={math.degrees(yaw(self.truth)):+7.1f}')


def main():
    rclpy.init()
    node = OdomCheck()
    try:
        rclpy.spin(node)
    except Exception:  # noqa: B902  (rclpy raises on shutdown from inside a callback)
        pass


if __name__ == '__main__':
    main()
