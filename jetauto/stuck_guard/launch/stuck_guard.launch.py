from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    window_sec = LaunchConfiguration('window_sec')
    manage_nav2 = LaunchConfiguration('manage_nav2')

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic',
                              default_value='/cmd_vel',
                              description='Twist topic Nav2 publishes to'),
        DeclareLaunchArgument('odom_topic',
                              default_value='/odom',
                              description='Wheel odometry topic'),
        DeclareLaunchArgument('window_sec',
                              default_value='7.0',
                              description='Rolling window length (seconds) for stuck detection'),
        DeclareLaunchArgument('manage_nav2',
                              default_value='true',
                              description='Whether to call Nav2 lifecycle services'),

        Node(
            package='stuck_guard',
            executable='stuck_guard',
            name='stuck_guard',
            output='screen',
            parameters=[{
                'cmd_vel_topic': cmd_vel_topic,
                'odom_topic': odom_topic,
                'window_sec': window_sec,
                'manage_nav2': manage_nav2,
            }],
        ),
    ])
