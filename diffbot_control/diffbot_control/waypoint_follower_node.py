"""ROS 2 node: drive through a list of (x, y) waypoints with two PID loops.

Subscribes:  /odom                 (nav_msgs/Odometry)
Publishes:   /cmd_vel              (geometry_msgs/Twist)
             ~/path                (nav_msgs/Path)    the waypoints, for RViz
             ~/heading_error       (std_msgs/Float64)
             ~/distance_error      (std_msgs/Float64)
"""

from diffbot_control.csv_logger import CsvLogger
from diffbot_control.pid import PID
from diffbot_control.waypoint_logic import (
    parse_waypoints, WaypointFollower, WaypointParams, yaw_from_quaternion,
)
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64


class WaypointFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_follower')

        self.declare_parameter('waypoints', [2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 0.0, 0.0])
        self.declare_parameter('loop', False)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('heading_kp', 3.0)
        self.declare_parameter('heading_ki', 0.0)
        self.declare_parameter('heading_kd', 0.3)
        self.declare_parameter('distance_kp', 0.8)
        self.declare_parameter('distance_ki', 0.05)
        self.declare_parameter('distance_kd', 0.1)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('log_csv', '')

        g = lambda name: float(self.get_parameter(name).value)  # noqa: E731
        waypoints = parse_waypoints(self.get_parameter('waypoints').value)
        params = WaypointParams(
            goal_tolerance=g('goal_tolerance'),
            max_linear_speed=g('max_linear_speed'),
            max_angular_speed=g('max_angular_speed'),
            loop=bool(self.get_parameter('loop').value),
        )
        heading_pid = PID(
            g('heading_kp'), g('heading_ki'), g('heading_kd'),
            output_limits=(-params.max_angular_speed, params.max_angular_speed),
            derivative_tau=0.05,
        )
        distance_pid = PID(
            g('distance_kp'), g('distance_ki'), g('distance_kd'),
            output_limits=(0.0, params.max_linear_speed),
            integral_limits=(0.0, 0.2),
            derivative_tau=0.05,
        )
        self.controller = WaypointFollower(waypoints, params, heading_pid, distance_pid)
        self.frame_id = self.get_parameter('frame_id').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.heading_pub = self.create_publisher(Float64, '~/heading_error', 10)
        self.distance_pub = self.create_publisher(Float64, '~/distance_error', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '~/path', latched)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)

        log_path = self.get_parameter('log_csv').value
        self.logger_csv = CsvLogger(
            log_path, ['t', 'x', 'y', 'yaw', 'goal', 'distance_error', 'heading_error', 'v', 'w']
        ) if log_path else None

        self._publish_path(waypoints)
        self._last_stamp = None
        self._last_index = -1
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints: {waypoints}')

    def _publish_path(self, waypoints) -> None:
        path = Path()
        path.header.frame_id = self.frame_id
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.02 if self._last_stamp is None else stamp - self._last_stamp
        self._last_stamp = stamp

        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        was_done = self.controller.done
        out = self.controller.step(pos.x, pos.y, yaw, dt)

        cmd = Twist()
        cmd.linear.x = out.linear
        cmd.angular.z = out.angular
        self.cmd_pub.publish(cmd)
        self.heading_pub.publish(Float64(data=out.heading_error))
        self.distance_pub.publish(Float64(data=out.distance_error))

        if out.index != self._last_index and not out.done:
            goal = self.controller.waypoints[out.index]
            self.get_logger().info(f'Heading to waypoint {out.index}: {goal}')
            self._last_index = out.index
        if out.done and not was_done:
            self.get_logger().info('All waypoints reached.')
        if self.logger_csv:
            self.logger_csv.write([
                stamp, pos.x, pos.y, yaw, out.index,
                out.distance_error, out.heading_error, out.linear, out.angular,
            ])

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())
        if self.logger_csv:
            self.logger_csv.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointFollowerNode()
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
