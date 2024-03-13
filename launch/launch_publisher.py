#!/usr/bin/env python3
# Copyright 2022 iRobot Corporation. All Rights Reserved.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Evaluate at launch the value of the launch configuration 'namespace'
    namespace = LaunchConfiguration('namespace')

    # Declares an action to allow users to pass the robot namespace from the
    # CLI into the launch description as an argument.
    namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace')

    # Test
    nolidar = Node(
        package='create3_project',
        name='ir_intensity_publisher',
        executable='ir_intensity_publisher',
        parameters=[],
        output='screen'
    )

    ir_intensity_vector_node = Node(
        package='irobot_create_nodes',
        name='ir_intensity_vector_publisher',
        executable='ir_intensity_vector_publisher',
        parameters=[],
        output='screen',
    )

    # Launches all named actions
    return LaunchDescription([
        nolidar,
        ir_intensity_vector_node
    ])
