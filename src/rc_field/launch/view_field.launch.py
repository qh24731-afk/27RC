#!/usr/bin/env python3
"""
Launch file for viewing the ABU Robocon 2027 competition field in RViz2.

Usage:
    ros2 launch rc_field view_field.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rc_field')

    urdf_file = os.path.join(pkg_share, 'urdf', 'field.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'field_view.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='field_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    joint_state_pub = Node(
        package='rc_field',
        executable='publish_joint_states.py',
        name='field_joint_state_pub',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        robot_state_pub,
        joint_state_pub,
        rviz2,
    ])
