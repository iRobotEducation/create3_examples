#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Alexis Pojomovsky (apojomovsky@irobot.com)
#
# Launches a joystick teleop node.

import os
from typing import Text

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


class JoystickConfigParser(Substitution):
    def __init__(
        self,
        package_name: Text,
        device_type: SomeSubstitutionsType,
        enable_set: SomeSubstitutionsType
    ) -> None:
        self.__package_name = package_name
        self.__device_type = device_type
        self.__enable_set = enable_set

    def perform(
        self,
        context: LaunchContext = None,
    ) -> Text:
        device_type_str = self.__device_type.perform(context)
        enable_set_str = self.__enable_set.perform(context)
        package_str = self.__package_name
        try:
            package_share_dir = get_package_share_directory(
                package_str)
            config_filepath = [package_share_dir, 'config',
                               f'{enable_set_str}.{device_type_str}.config.yaml']
            return os.path.join(*config_filepath)
        except PackageNotFoundError:
            raise PackageNotFoundError(package_str)


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_enable = LaunchConfiguration('joy_enable')

    # Invokes a node that interfaces a generic joystick to ROS 2.
    joy_node = Node(package='joy', executable='joy_node', name='joy_node',
                    parameters=[{
                        'dev': joy_dev,
                        'deadzone': 0.3,
                        'autorepeat_rate': 20.0,
                    }])

    # Retrieve the path to the correct configuration .yaml depending on
    # the joy_config argument
    config_filepath = JoystickConfigParser('create3_teleop', joy_config, joy_enable)

    # Publish unstamped Twist message from an attached USB Joystick.
    teleop_node = Node(package='teleop_twist_joy', executable='teleop_node',
                       name='teleop_twist_joy_node', parameters=[config_filepath])

    # Declare launchfile arguments
    ld_args = []
    ld_args.append(DeclareLaunchArgument('joy_config',
                                         default_value='xbox',
                                         choices=['ps3-ble', 'xbox', 'ps3', 'ps3-holonomic', 'atk3', 'xd3']))
    ld_args.append(DeclareLaunchArgument('joy_dev',
                                         default_value='/dev/input/js0'))
    ld_args.append(DeclareLaunchArgument('joy_enable',
                                         default_value='no-enable',
                                         choices=['no-enable', 'enable']))
    # Define LaunchDescription variable
    ld = LaunchDescription(ld_args)

    # Add nodes to LaunchDescription
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    return ld
