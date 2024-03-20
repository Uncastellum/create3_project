#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Evaluate at launch the value of the launch configuration 'namespace'
    namespace = LaunchConfiguration('namespace')
    # Evaluate at launch the value of the launch configuration 'use_sim_time'
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declares an action to allow users to pass the robot namespace from the
    # CLI into the launch description as an argument.
    namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace')
    # Declares an action to allow users to pass the sim time from the
    # CLI into the launch description as an argument.
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    # Gets the directory of the package and stores it as 'pkg_dir'
    pkg_dir = get_package_share_directory('create3_project')
    # Generate the path to the rviz configuration file
    rviz2_config = PathJoinSubstitution(
        [pkg_dir, 'config', 'lidar_slam.rviz'])

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
            get_package_share_directory("create3_project") + '/config/rplidar_node.yaml'
            ],
        namespace=namespace
    )
    # Declares an action that will launch a node when executed by the launch description.
    # This node is responsible for configuring and running slam toolbox.
    start_async_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory("create3_project") + '/config/mapper_params_online_async.yaml',
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

    # LAUNCH DESCRIPTION
    ld = LaunchDescription()

    ld.add_action(namespace_argument)
    ld.add_action(use_sim_time_argument)
    # sensors_launch.py
    ld.add_action(static_transform_node)
    ld.add_action(TimerAction(
        period=2.0,
        actions=[rplidar_node]
    ))
    # slam_toolbox_launch.py
    ld.add_action(TimerAction(
        period=4.0,
        actions=[start_async_slam_toolbox_node]
    ))
    # rviz_launch.py
    ld.add_action(TimerAction(
        period=6.0,
        actions=[rviz_node]
    ))

    # Launches all named actions
    return ld
