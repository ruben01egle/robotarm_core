"""Standalone MoveIt demo with simulated (mock) hardware; no RT PC or firmware repo needed.

Starts:
  - robot_state_publisher        URDF with use_hardware:=false -> mock_components/GenericSystem
  - ros2_control_node            controller_manager with config/ros2_controllers.yaml
  - spawner                      joint_state_broadcaster (active)
  - spawner                      joint_trajectory_controller (active)
  - move_group (+ RViz)          via launch/move_group.launch.py

The mock hardware copies every position command directly to the position state, so the robot
"follows" every trajectory perfectly. Do not start this together with the firmware mock or the real
robot: both would provide /controller_manager and /joint_states (see README, "Known limitations").

Arguments:
  rviz:=true|false   start RViz (default: true)
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('robotarm', package_name='robotarm_moveit_config')
        .robot_description(mappings={'use_hardware': 'false'})
        .to_moveit_configs()
    )
    package_path = Path(moveit_config.package_path)
    ros2_controllers_file = str(package_path / 'config' / 'ros2_controllers.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz with the MotionPlanning panel')

    # Publishes /tf from /joint_states and the URDF on the /robot_description topic.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[moveit_config.robot_description],
    )

    # controller_manager; reads the URDF (incl. ros2_control block) from /robot_description.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[ros2_controllers_file],
        remappings=[('~/robot_description', '/robot_description')],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Spawned active here. On the real robot it is spawned inactive and activated by control_center.
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_path / 'launch' / 'move_group.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        move_group_launch,
    ])
