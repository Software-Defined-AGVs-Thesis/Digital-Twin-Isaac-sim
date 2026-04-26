from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_topic = LaunchConfiguration('depth_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    depth_threshold = LaunchConfiguration('depth_threshold')
    manage_nav2 = LaunchConfiguration('manage_nav2')

    return LaunchDescription([
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera/depth',
                              description='Depth image topic from Isaac Sim'),
        DeclareLaunchArgument('cmd_vel_topic',
                              default_value='/cmd_vel',
                              description='Velocity topic to publish zero Twist to'),
        DeclareLaunchArgument('depth_threshold',
                              default_value='1.7',
                              description='Cliff detection depth threshold (meters)'),
        DeclareLaunchArgument('manage_nav2',
                              default_value='true',
                              description='Whether to call Nav2 lifecycle services'),

        Node(
            package='cliff_guard',
            executable='cliff_guard',
            name='cliff_guard',
            output='screen',
            parameters=[{
                'depth_topic': depth_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'depth_threshold': depth_threshold,
                'manage_nav2': manage_nav2,
            }],
        ),
    ])
