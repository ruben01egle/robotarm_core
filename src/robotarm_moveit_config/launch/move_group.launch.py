"""Start MoveIt's move_group node and, optionally, RViz with the MotionPlanning panel.

Use this file against a running ros2_control stack (RT PC, robotarm_firmware_lnx). It does NOT
start a controller_manager or a robot_state_publisher; both have to be provided by the robot:
  - /joint_states                                          (joint_state_broadcaster)
  - /tf                                                    (robot_state_publisher)
  - /joint_trajectory_controller/follow_joint_trajectory  (joint_trajectory_controller, active)
launch/demo.launch.py includes this file and adds a mock ros2_control stack for testing.

Arguments:
  rviz:=true|false   start RViz (default: true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Collects all MoveIt parameters from this package (see README, "File overview"):
    #   URDF    <- robotarm_description (path and xacro args from .setup_assistant)
    #   SRDF    <- config/robotarm.srdf
    #   IK      <- config/kinematics.yaml
    #   limits  <- config/joint_limits.yaml, config/pilz_cartesian_limits.yaml
    #   planners<- config/ompl_planning.yaml, config/pilz_industrial_motion_planner_planning.yaml
    #   execution <- config/moveit_controllers.yaml
    moveit_config = (
        MoveItConfigsBuilder('robotarm', package_name='robotarm_moveit_config')
        # use_hardware only selects the ros2_control plugin in the URDF; move_group ignores the
        # ros2_control block, so the mock variant is fine here, also for the real robot.
        # tool: file in robotarm_description/urdf/tools/ that is mounted on the flange and defines
        # the tcp link ("none" = tcp on the flange). Must match the tool used on the RT PC.
        .robot_description(mappings={'use_hardware': 'false', 'tool': 'none'})
        # moveit_manage_controllers=False: MoveIt never switches ros2_control controllers,
        # control_center does (mission request).
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml',
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner'],
            default_planning_pipeline='ompl',
        )
        .to_moveit_configs()
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz with the MotionPlanning panel')

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                # publish the SRDF on /robot_description_semantic for other tools
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                # do not copy velocities/efforts from /joint_states into the planning scene
                'monitor_dynamics': False,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(moveit_config.package_path / 'config' / 'moveit.rviz')],
        # The MotionPlanning panel runs its own robot model, IK and planner lists, so it needs the
        # same parameters as move_group.
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        move_group_node,
        rviz_node,
    ])
