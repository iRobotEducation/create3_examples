#!/usr/bin/env python3
# Copyright 2022 iRobot Corporation. All Rights Reserved.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    namespace_argument = DeclareLaunchArgument(
        'namespace', 
        default_value='',
        description='Robot namespace')
    
    static_transform_node = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        arguments=['-0.012', '0', '0.144', '0', '0', '0', 'base_footprint', 'laser_frame'],
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')],
        namespace=namespace
        )
    
    rplidar_node = Node(
        package='rplidar_ros',
        parameters=[
                get_package_share_directory("create3_lidar") + '/config/rplidar_node.yaml'
                ],
        executable='rplidar_composition',
        namespace=namespace
        )
    
    return LaunchDescription([
        namespace_argument,
        static_transform_node,
        TimerAction(
            period=2.0,
            actions=[rplidar_node]
        )
    ])
