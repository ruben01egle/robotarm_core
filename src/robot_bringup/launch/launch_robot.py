from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    extra_args = ['--log-level', 'rmw_cyclonedds_cpp:=ERROR']

    return LaunchDescription([
        
        Node(
            package='control_center',
            executable='control_center_node',
            name='control_center',
            output='screen',
            ros_arguments=extra_args
        ),
        
        Node(
            package='gui',
            executable='gui_ros_node',
            name='gui_node',
            output='screen',
            ros_arguments=extra_args
        ),

        Node(
            package='mission_controller',
            executable='mission_controller_node',
            name='mission_controller_node',
            output='screen',
            ros_arguments=extra_args
        ),

        Node(
            package='planner',
            executable='p2p_jointspace_node',
            name='p2p_jointspace_node',
            output='screen',
            ros_arguments=extra_args
        ),
    ])