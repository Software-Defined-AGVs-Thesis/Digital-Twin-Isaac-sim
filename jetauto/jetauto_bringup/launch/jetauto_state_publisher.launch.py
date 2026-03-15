#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(
        get_package_share_directory('jetauto_description'),
        'urdf',
        'jetauto_flat.urdf')

    robot_desc = Command([
        'xacro ',
        urdf,
        ' namespace:=',
        PythonExpression(['"', namespace, '" + "/" if "', namespace, '" != "" else ""']),
    ])

    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                rsp_params,
                {'use_sim_time': use_sim_time}]),
    ])
