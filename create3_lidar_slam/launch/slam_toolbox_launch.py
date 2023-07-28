#!/usr/bin/env python3
# Copyright 2022 iRobot Corporation. All Rights Reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Evaluate at launch the value of the launch configuration 'use_sim_time' 
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declares an action to allow users to pass the sim time from the 
    # CLI into the launch description as an argument.
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    
    # Evaluate at launch the value of the launch configuration 'namespace' 
    namespace = LaunchConfiguration('namespace')

    # Declares an action to allow users to pass the robot namespace from the 
    # CLI into the launch description as an argument.
    namespace_argument = DeclareLaunchArgument(
        'namespace', 
        default_value='',
        description='Robot namespace')
    
    # Declares an action that will launch a node when executed by the launch description.
    # This node is responsible for configuring and running slam toolbox.  
    start_async_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory("create3_lidar_slam") + '/config/mapper_params_online_async.yaml',
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        # Remaps topics used by the 'slam_toolbox' package from absolute (with slash) to relative (no slash).
        # This is necessary to use namespaces with 'slam_toolbox'.
        remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/scan', 'scan'),
        ('/map', 'map'),
        ('/map_metadata', 'map_metadata')
    ])

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(namespace_argument)
    ld.add_action(start_async_slam_toolbox_node)

    # Launches all named actions
    return ld
