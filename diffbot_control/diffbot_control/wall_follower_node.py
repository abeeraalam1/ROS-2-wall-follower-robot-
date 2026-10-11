"""ROS 2 node: follow a wall at a fixed distance using a PID on lidar data.

Subscribes:  /scan     (sensor_msgs/LaserScan)
Publishes:   /cmd_vel  (geometry_msgs/Twist)
             ~/error   (std_msgs/Float64)  distance error, for rqt_plot / tuning
             ~/mode    (std_msgs/String)   follow | corner | search
"""

import math

from diffbot_control.csv_logger import CsvLogger
from diffbot_control.pid import PID
from diffbot_control.wall_follow_logic import (
    LEFT, RIGHT, ScanView, WallFollower, WallFollowParams,
)
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String


class WallFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('wall_follower')

        self.declare_parameter('side', 'right')
        self.declare_parameter('desired_distance', 0.5)
        self.declare_parameter('lookahead', 0.4)
        self.declare_parameter('beam_angle_deg', 50.0)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('front_stop_distance', 0.55)
        self.declare_parameter('front_slow_distance', 1.2)
        self.declare_parameter('kp', 2.5)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('derivative_tau', 0.05)
        self.declare_parameter('log_csv', '')

        side_name = self.get_parameter('side').value.lower()
        if side_name not in ('left', 'right'):
            raise ValueError("parameter 'side' must be 'left' or 'right'")

        g = lambda name: float(self.get_parameter(name).value)  # noqa: E731
        params = WallFollowParams(
            side=LEFT if side_name == 'left' else RIGHT,
            desired_distance=g('desired_distance'),
            lookahead=g('lookahead'),
            beam_angle=math.radians(g('beam_angle_deg')),
            max_linear_speed=g('max_linear_speed'),
            max_angular_speed=g('max_angular_speed'),
            front_stop_distance=g('front_stop_distance'),
            front_slow_distance=g('front_slow_distance'),
        )
        max_w = params.max_angular_speed
        pid = PID(
            kp=g('kp'), ki=g('ki'), kd=g('kd'),
            output_limits=(-max_w, max_w),
            integral_limits=(-0.5 * max_w, 0.5 * max_w),
            derivative_tau=g('derivative_tau'),
        )
        self.controller = WallFollower(params, pid)

        # Gains can be changed live (ros2 param set /wall_follower kp 3.0),
        # which makes tuning in simulation fast.
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.error_pub = self.create_publisher(Float64, '~/error', 10)
        self.mode_pub = self.create_publisher(String, '~/mode', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

        log_path = self.get_parameter('log_csv').value
        columns = ['t', 'error', 'distance', 'alpha', 'v', 'w', 'mode']
        self.logger_csv = CsvLogger(log_path, columns) if log_path else None

        self._last_stamp = None
        self._last_mode = ''
        self.get_logger().info(
            f'Following the {side_name} wall at {params.desired_distance:.2f} m '
            f'(kp={pid.kp}, ki={pid.ki}, kd={pid.kd})'
        )

    def _on_params(self, changed):
        pid = self.controller.pid
        for p in changed:
            if p.name in ('kp', 'ki', 'kd'):
                setattr(pid, p.name, float(p.value))
            elif p.name == 'desired_distance':
                self.controller.params.desired_distance = float(p.value)
        return SetParametersResult(successful=True)

    def _on_scan(self, msg: LaserScan) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.1 if self._last_stamp is None else stamp - self._last_stamp
        self._last_stamp = stamp

        view = ScanView(
            msg.angle_min, msg.angle_increment, msg.ranges, msg.range_min, msg.range_max
        )
        out = self.controller.step(view, dt)

        cmd = Twist()
        cmd.linear.x = out.linear
        cmd.angular.z = out.angular
        self.cmd_pub.publish(cmd)
        self.error_pub.publish(Float64(data=out.error))
        if out.mode != self._last_mode:
            self.get_logger().info(f'mode: {out.mode}')
            self._last_mode = out.mode
        self.mode_pub.publish(String(data=out.mode))
        if self.logger_csv:
            self.logger_csv.write([
                stamp, out.error, out.distance, out.alpha, out.linear, out.angular, out.mode,
            ])

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())
        if self.logger_csv:
            self.logger_csv.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
