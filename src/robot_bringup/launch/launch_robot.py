import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    description_share = get_package_share_directory('robotarm_description')

    include_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'rviz.launch.py')
        )
    )

    return LaunchDescription([
        
        Node(
            package='control_center',
            executable='control_center_node',
            name='control_center',
            output='screen',
            parameters=[{
                'hardware_component_name': 'MoteusHardwareSystem'
            }]
        ),
        
        Node(
            package='gui',
            executable='gui_ros_node',
            name='gui_node',
            output='screen'
        ),

        include_rviz
    ])