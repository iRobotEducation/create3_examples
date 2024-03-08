#!/usr/bin/env python3
# Copyright 2024 iRobot Corporation. All Rights Reserved.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Fast-DDS env variable check
    rmw_impl_env_var = "RMW_IMPLEMENTATION"
    fastdds_config_env_var = "FASTRTPS_DEFAULT_PROFILES_FILE"

    if rmw_impl_env_var in os.environ:
        print("RMW IMPLEMENTATION: ", os.environ[rmw_impl_env_var])
    if fastdds_config_env_var in os.environ:
        print("FAST-DDS CONFIG FILE: ", os.environ[fastdds_config_env_var])

    republisher_ns = LaunchConfiguration('republisher_ns')
    republisher_ns_argument = DeclareLaunchArgument(
        'republisher_ns', 
        default_value='/repub',
        description='Republisher namespace')
    
    robot_ns = LaunchConfiguration('robot_ns')
    robot_ns_argument = DeclareLaunchArgument(
        'robot_ns', 
        default_value='/',
        description='Create 3 robot namespace')

    params_yaml_file = get_package_share_directory("create3_republisher") + '/bringup/params.yaml'
    print("Using yaml file ", params_yaml_file)

    start_republisher_node = Node(
        parameters=[
          params_yaml_file,
          {'robot_namespace': robot_ns}
        ],
        package='create3_republisher',
        executable='create3_republisher',
        name='create3_repub',
        output='screen',
        namespace=republisher_ns,
    )

    ld = LaunchDescription()

    ld.add_action(republisher_ns_argument)
    ld.add_action(robot_ns_argument)
    ld.add_action(start_republisher_node)

    return ld
