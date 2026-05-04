"""Bring up SLAM, Nav2, RViz, and the VR UDP receiver.

cliff_guard is launched separately (in its own tmux window) so its logs
are visible on their own and so a cliff_guard crash cannot tear down
SLAM/Nav2/RViz.

Run inside the Isaac Sim container after `/clock` is publishing:
    source /opt/ros/humble/setup.bash
    source /workspace/omnilrs/install/local_setup.bash
    ros2 launch jetauto_bringup omnilrs_stack.launch.py
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')
    rviz_config = LaunchConfiguration('rviz_config')

    slam_launch = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value='/workspace/omnilrs/jetauto/jetauto_bringup/nav2_params.yaml'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'),

        # t=0s: SLAM first (provides /map and TF needed by Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'base_frame': 'base_footprint',
            }.items(),
        ),

        # t=10s: Nav2 (waits for SLAM to publish /map + TF)
        TimerAction(
            period=10.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam': 'True',
                    'params_file': nav2_params,
                    'cmd_vel_topic': '/cmd_vel',
                }.items(),
            )],
        ),

        # t=20s: RViz
        TimerAction(
            period=20.0,
            actions=[ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config],
                output='screen',
            )],
        ),

        # t=30s: VR UDP receiver
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', '/workspace/omnilrs/vr_container_receiver.py'],
                    output='screen',
                ),
            ],
        ),
    ])
