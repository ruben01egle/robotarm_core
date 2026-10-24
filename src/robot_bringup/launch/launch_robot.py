import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    description_share = get_package_share_directory('robotarm_description')
    moveit_config_share = get_package_share_directory('robotarm_moveit_config')

    moveit = LaunchConfiguration('moveit')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    tool = LaunchConfiguration('tool')
    use_sim_time = LaunchConfiguration('use_sim_time')
    execution_gate = LaunchConfiguration('execution_gate')
    motion_share = get_package_share_directory('robotarm_motion')

    # MoveIt (move_group) with the MotionPlanning RViz; it replaces the plain RViz below.
    # Needs the RT PC (or the firmware mock) for /joint_states, /tf and joint_trajectory_controller.
    include_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'rviz': rviz, 'tool': tool, 'use_sim_time': use_sim_time,
            # with the gate move_group only plans; the motion_server executes
            'allow_trajectory_execution': PythonExpression(
                ["'false' if '", execution_gate, "' == 'true' else 'true'"]),
        }.items(),
        condition=IfCondition(moveit)
    )

    # execution gate: the only node that executes MoveIt trajectories (inside its own mission)
    include_motion_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motion_share, 'launch', 'motion_server.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(PythonExpression([
            "'", moveit, "' == 'true' and '", execution_gate, "' == 'true'"]))
    )

    # plain RViz (robot model only), used when MoveIt is not started
    include_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'rviz.launch.py')
        ),
        condition=IfCondition(PythonExpression([
            "'", rviz, "' == 'true' and '", moveit, "' != 'true'"]))
    )

    return LaunchDescription([
        DeclareLaunchArgument('moveit', default_value='true',
                              description='Start MoveIt move_group'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz (with MotionPlanning panel if moveit:=true)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the operator GUI'),
        DeclareLaunchArgument('tool', default_value='axis_triad',
                              description='Tool on the flange for MoveIt, must match the RT PC'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='MoveIt on simulation time (true with the MuJoCo sim)'),
        DeclareLaunchArgument('execution_gate', default_value='true',
                              description='true: move_group only plans, the motion_server '
                                          'executes (false: RViz Execute, demo_motions --mission)'),

        Node(
            package='control_center',
            executable='control_center_node',
            name='control_center',
            output='screen',
            parameters=[{
                'hardware_component_name': 'moteus_hardware_system'
            }]
        ),

        Node(
            package='gui',
            executable='gui_ros_node',
            name='gui_node',
            output='screen',
            condition=IfCondition(gui)
        ),

        include_moveit,
        include_motion_server,
        include_rviz
    ])
