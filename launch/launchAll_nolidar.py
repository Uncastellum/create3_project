#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Gets the directory of the package and stores it as 'lidar_pkg'
    lidar_pkg = get_package_share_directory('create3_project')

    # Generate the path to the rviz configuration file
    rviz2_config = PathJoinSubstitution(
        [lidar_pkg, 'config', 'nolidar_slam.rviz'])

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

    nolidar = Node(
        package='create3_project',
        name='ir_intensity_publisher',
        executable='ir_intensity_publisher',
        parameters=[],
        output='screen'
    )

    # Declares an action that will launch a node when executed by the launch description.
    # This node is responsible for configuring and running slam toolbox.
    start_async_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory("create3_project") + '/config/mapper_params_online_async.yaml',
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
    # sensors_launch.py
    ld.add_action(static_transform_node)
    ld.add_action(nolidar)
    # slam_toolbox_launch.py
    ld.add_action(TimerAction(
        period=3.0,
        actions=[start_async_slam_toolbox_node]
    ))
    ld.add_action(TimerAction(
        period=5.0,
        actions=[rviz_node]
    ))

    # Launches all named actions
    return ld
