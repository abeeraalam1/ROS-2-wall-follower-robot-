"""
Simulation + PID waypoint follower.

  ros2 launch diffbot_control waypoints.launch.py
  ros2 launch diffbot_control waypoints.launch.py headless:=true log_csv:=/tmp/waypoints.csv
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('diffbot_sim'), 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    params = os.path.join(
        get_package_share_directory('diffbot_control'), 'config', 'waypoints.yaml'
    )
    controller = Node(
        package='diffbot_control',
        executable='waypoint_follower',
        parameters=[params, {'log_csv': LaunchConfiguration('log_csv')}],
        output='screen',
    )
    # Optional: log wheel odometry vs. ground truth for plotting.
    recorder = Node(
        package='diffbot_control',
        executable='trajectory_recorder',
        parameters=[{'csv': LaunchConfiguration('trajectory_csv'), 'use_sim_time': True}],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('trajectory_csv'), "' != ''"])
        ),
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'log_csv', default_value='', description='Write controller data to this CSV'
        ),
        DeclareLaunchArgument(
            'trajectory_csv', default_value='', description='Record odom vs. ground truth here'
        ),
        sim,
        controller,
        recorder,
    ])
