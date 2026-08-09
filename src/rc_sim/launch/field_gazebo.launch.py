#!/usr/bin/env python3
"""在 Gazebo 中加载 ABU Robocon 2027 竞赛场地。

用法:
    ros2 launch rc_sim field_gazebo.launch.py

启动内容:
    1. Gazebo（field.world）
    2. 把 rc_field/urdf/field.urdf 生成进 Gazebo
    3. robot_state_publisher 发布 TF（供 RViz 同步观察）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # 场地 URDF（rc_field 包）
    rc_field_share = get_package_share_directory('rc_field')
    urdf_path = os.path.join(rc_field_share, 'urdf', 'field.urdf')

    # Gazebo 世界文件（rc_sim 包）
    rc_sim_share = get_package_share_directory('rc_sim')
    world_path = os.path.join(rc_sim_share, 'worlds', 'field.world')

    # 读取 URDF 字符串
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # 1. 启动 Gazebo（加载 factory 与 force_system 插件，才能 spawn 实体）
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path,
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_force_system.so'],
        output='screen',
    )

    # 2. 把场地 URDF 生成进 Gazebo
    spawn_field = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rc_field',
            '-file', urdf_path,
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    # 3. 发布场地 TF 树（可选，便于 RViz 查看）
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='field_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    return LaunchDescription([
        gazebo,
        spawn_field,
        robot_state_pub,
    ])
