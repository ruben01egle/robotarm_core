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

    # MoveIt (move_group) with the MotionPlanning RViz; it replaces the plain RViz below.
    # Needs the RT PC (or the firmware mock) for /joint_states, /tf and joint_trajectory_controller.
    include_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'rviz': rviz}.items(),
        condition=IfCondition(moveit)
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
        include_rviz
    ])
