#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='m',
        description='TETRA robot model (m, h, edu, mh, s, sm)'
    )

    tetra_wheel_driver_node = Node(
        package='tetra_wheel_driver', 
        executable='tetra_wheel_driver',
        output='screen',
        parameters=[
            {"robot_model": LaunchConfiguration('robot_model')},
            {"m_bEKF_option": False}
        ]
    )

    return LaunchDescription([
        robot_model_arg,
        tetra_wheel_driver_node
    ])
