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
            package='mission_controller',
            executable='mission_controller_node',
            name='mission_controller_node',
            output='screen'
        ),

        Node(
            package='hardware_simulation',
            executable='hardware_simulation_node',
            name='stm32_sim_node',
            output='screen'
        ),

        Node(
            package='planner',
            executable='p2p_jointspace_node',
            name='p2p_jointspace_node',
            output='screen'
        ),
    ])