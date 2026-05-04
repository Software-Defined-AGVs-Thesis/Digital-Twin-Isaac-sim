"""Launch explore_lite + forever_explore companion.

Run once SLAM and Nav2 are up:
    ros2 launch forever_explore forever_explore.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    default_params = PathJoinSubstitution([
        FindPackageShare('forever_explore'), 'config', 'explore_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='forever_explore',
            executable='forever_explore',
            name='forever_explore',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
