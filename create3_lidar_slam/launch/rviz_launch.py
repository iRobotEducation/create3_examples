#!/usr/bin/env python3
# Copyright 2023 iRobot Corporation. All Rights Reserved.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Gets the directory of the package and stores it as 'lidar_pkg'
    lidar_pkg = get_package_share_directory('create3_lidar_slam')

    # Generate the path to the rviz configuration file
    rviz2_config = PathJoinSubstitution(
        [lidar_pkg, 'rviz', 'create3_lidar_slam.rviz'])

    # Evaluate at launch the value of the launch configuration 'namespace' 
    namespace = LaunchConfiguration('namespace')

    # Declares an action to allow users to pass the robot namespace from the 
    # CLI into the launch description as an argument.
    namespace_argument = DeclareLaunchArgument(
        'namespace', 
        default_value='',
        description='Robot namespace')

    # Declares an action that will launch a node when executed by the launch description.
    # This node is responsible for configuring and running rviz2.   
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2',
        arguments=['-d', rviz2_config],
        
        # Remaps topics used by the 'rviz2' package from absolute (with slash) to relative (no slash).
        # This is necessary to use namespaces with 'rviz2'.
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')],
        namespace=namespace,
        output='screen'
        )

    
    ld = LaunchDescription()

    ld.add_action(namespace_argument)
    ld.add_action(rviz_node)

    # Launches all named actions
    return ld 