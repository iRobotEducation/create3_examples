#!/usr/bin/env python3
# Copyright 2022 iRobot Corporation. All Rights Reserved.

from launch import LaunchContext
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
            arguments=['-0.012', '0', '0.144', '0', '0', '0', 'base_footprint', 'laser_frame']),
        Node(
            package='rplidar_ros',
            parameters=[
                get_package_share_directory("create3_lidar") + '/config/rplidar_node.yaml'
                ],
            executable='rplidar_composition'
        ),
    ])
