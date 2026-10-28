"""
Start the motion_server: goals, programs and MoveIt jobs inside a mission (robotarm_motion README).

Launch arguments:
  goals_file:=<path>        goals file (default $ROS_HOME/robotarm_motion/goals.yaml)
  default_scaling:=<0..1>   scaling of jobs that do not set one (default 0.2)
  use_sim_time:=true|false  true with the MuJoCo sim (default false)

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robotarm_motion.goals import default_goals_file


def launch_setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    return [Node(
        package='robotarm_motion',
        executable='motion_server',
        name='motion_server',
        output='screen',
        parameters=[{
            'goals_file': value('goals_file') or default_goals_file(),
            'default_scaling': float(value('default_scaling')),
            'use_sim_time': value('use_sim_time') == 'true',
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goals_file', default_value='',
                              description='goals file, empty: the default in $ROS_HOME'),
        DeclareLaunchArgument('default_scaling', default_value='0.2',
                              description='velocity/acceleration scaling of jobs without one'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true with the MuJoCo sim'),
        OpaqueFunction(function=launch_setup),
    ])
