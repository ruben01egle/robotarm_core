"""Start MoveIt's move_group node and, optionally, RViz with the MotionPlanning panel.

Use this file against a running ros2_control stack (RT PC, robotarm_firmware_lnx). It does NOT
start a controller_manager or a robot_state_publisher; both have to be provided by the robot:
  - /joint_states                                          (joint_state_broadcaster)
  - /tf                                                    (robot_state_publisher)
  - /joint_trajectory_controller/follow_joint_trajectory  (joint_trajectory_controller, active)
launch/demo.launch.py includes this file and adds a mock ros2_control stack for testing.

Arguments:
  rviz:=true|false   start RViz (default: true)
  tool:=<name>       tool on the flange, file in robotarm_description/urdf/tools/ without .xacro
                     (default: axis_triad, as on the RT PC; "none" = tcp on the flange)
  use_sim_time:=true|false   use the /clock of a simulation (MuJoCo sim: true; default: false)
  allow_trajectory_execution:=true|false
                     false: move_group only plans (RViz Execute and MoveGroup plan+execute fail)
                     (default: true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

DEFAULT_TOOL = 'axis_triad'


def load_moveit_config(tool):
    """Collect all MoveIt parameters of this package for the given tool."""
    # Sources (see README, "File overview"):
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
        # the tcp link. Must match the tool used on the RT PC, otherwise the tcp is in the wrong
        # place. The collision matrix in the SRDF was generated for DEFAULT_TOOL.
        .robot_description(mappings={'use_hardware': 'false', 'tool': tool})
        # moveit_manage_controllers=False: MoveIt never switches ros2_control controllers,
        # the firmware's mode_manager does (control_center mission request).
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
    return moveit_config


def launch_setup(context):
    moveit_config = load_moveit_config(LaunchConfiguration('tool').perform(context))
    # The MuJoCo sim stamps /joint_states with sim time; with wall time move_group would consider
    # the robot state outdated ("Failed to fetch current robot state").
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                # publish the SRDF on /robot_description_semantic for other tools
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution':
                    LaunchConfiguration('allow_trajectory_execution').perform(context) == 'true',
                # do not copy velocities/efforts from /joint_states into the planning scene
                'monitor_dynamics': False,
                # rate of /monitored_planning_scene (default 4 Hz: robot and objects in RViz
                # and other scene subscribers move in visible steps)
                'publish_planning_scene_hz': 20.0,
            },
            sim_time,
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
            sim_time,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz with the MotionPlanning panel'),
        DeclareLaunchArgument(
            'tool', default_value=DEFAULT_TOOL,
            description='tool on the flange, file in robotarm_description/urdf/tools/'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true: use the /clock of a simulation (MuJoCo sim)'),
        DeclareLaunchArgument(
            'allow_trajectory_execution', default_value='true',
            description='false: plan only, execution through the motion_server'),
        OpaqueFunction(function=launch_setup),
    ])
