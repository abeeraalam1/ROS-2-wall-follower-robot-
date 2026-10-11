"""ROS 2 node: record wheel odometry and ground truth side by side to a CSV.

Used to make the trajectory plots in the README and to measure how far wheel
odometry drifts from where the robot really is.

Subscribes:  /odom               (nav_msgs/Odometry)  wheel odometry (DiffDrive)
             /ground_truth/odom  (nav_msgs/Odometry)  true pose from Gazebo
"""

from diffbot_control.csv_logger import CsvLogger
from diffbot_control.waypoint_logic import yaw_from_quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class TrajectoryRecorder(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_recorder')
        self.declare_parameter('csv', 'trajectory.csv')
        self.declare_parameter('rate', 10.0)
        self.odom = None
        self.truth = None
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_truth, 10)
        self.csv = CsvLogger(
            self.get_parameter('csv').value,
            ['t', 'odom_x', 'odom_y', 'odom_yaw', 'true_x', 'true_y', 'true_yaw'],
        )
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._write)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def _write(self) -> None:
        if self.odom is None or self.truth is None:
            return
        row = [self.get_clock().now().nanoseconds * 1e-9]
        for msg in (self.odom, self.truth):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            row += [p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)]
        self.csv.write(row)

    def close(self) -> None:
        self.csv.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
