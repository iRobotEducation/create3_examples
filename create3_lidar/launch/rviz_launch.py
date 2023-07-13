#!/usr/bin/env python3
# Copyright 2023 iRobot Corporation. All Rights Reserved.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lidar_pkg = get_package_share_directory('create3_lidar')
    rviz2_config = PathJoinSubstitution(
        [lidar_pkg, 'rviz', 'create3_lidar.rviz'])

    namespace = LaunchConfiguration('namespace')

    namespace_argument = DeclareLaunchArgument(
        'namespace', 
        default_value='',
        description='Robot namespace')
    
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2',
        arguments=['-d', rviz2_config],
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')],
        namespace=namespace,
        output='screen'
        )

    
    ld = LaunchDescription()

    ld.add_action(namespace_argument)
    ld.add_action(rviz_node)

    return ld 