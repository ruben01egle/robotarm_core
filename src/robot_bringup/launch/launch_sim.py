from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='control_center',
            executable='control_center_node',
            name='control_center',
            output='screen'
        ),
        
        Node(
            package='gui',
            executable='gui_ros_node',
            name='gui_node',
            output='screen'
        ),

        Node(
            package='hardware_simulation',
            executable='hardware_simulation_node',
            name='hardware_simulation',
            output='screen'
        ),
    ])