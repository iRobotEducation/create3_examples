#!/usr/bin/env python3
# Copyright 2022 iRobot Corporation. All Rights Reserved.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Evaluate at launch the value of the launch configuration 'namespace'
    namespace = LaunchConfiguration('namespace')

    # Declares an action to allow users to pass the robot namespace from the
    # CLI into the launch description as an argument.
    namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace')

    # Declares an action that will launch a node when executed by the launch description.
    # This node is responsible for providing a static transform from the robot's base_footprint
    # frame to a new laser_frame, which will be the coordinate frame for the lidar.
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.012', '0', '0.144', '0', '0', '0', 'base_footprint', 'laser_frame'],

        # Remaps topics used by the 'tf2_ros' package from absolute (with slash) to relative (no slash).
        # This is necessary to use namespaces with 'tf2_ros'.
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')],
        namespace=namespace
    )

    # Declares an action that will launch a node when executed by the launch description.
    # This node is responsible for configuring the RPLidar sensor.
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[
            get_package_share_directory("create3_lidar_slam") + '/config/rplidar_node.yaml'
            ],
        namespace=namespace
    )

    # Launches all named actions
    return LaunchDescription([
        namespace_argument,
        static_transform_node,
        TimerAction(
            period=2.0,
            actions=[rplidar_node]
        )
    ])