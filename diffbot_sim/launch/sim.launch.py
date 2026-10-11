"""Start Gazebo with the test course, spawn the robot and bridge topics to ROS 2.

Arguments:
  headless:=true   run the Gazebo server only (no GUI), e.g. in Docker/CI
  rviz:=true       also open RViz
  x, y, yaw        spawn pose
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('diffbot_sim')
    world = os.path.join(pkg, 'worlds', 'wall_course.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'diffbot.urdf.xacro')
    bridge_config = os.path.join(pkg, 'config', 'ros_gz_bridge.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'diffbot.rviz')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # "-r" starts the simulation immediately; "-s" runs the server only.
    gz_args = PythonExpression([
        "'-r -v 2 ", world, "' + (' -s --headless-rendering' if '", headless, "' == 'true' else '')"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', '-name', 'diffbot',
            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.01',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # Let Gazebo find meshes/models installed with this package.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg)),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        rviz,
    ])
