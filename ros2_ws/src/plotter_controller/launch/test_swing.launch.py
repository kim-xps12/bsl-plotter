#!/usr/bin/env python3
"""
Launch file for BSL Plotter test swing demonstration.

Launches the test_swing node along with robot visualization in RViz2.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directories
    description_pkg = get_package_share_directory('bsl_plotter_description')
    controller_pkg = get_package_share_directory('plotter_controller')

    # Include robot display launch
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'display.launch.py')
        ),
        launch_arguments={'gui': 'false'}.items()
    )

    # Test swing node
    test_swing_node = Node(
        package='plotter_controller',
        executable='test_swing.py',
        name='test_swing',
        output='screen'
    )

    tf_trajectory_node = Node(
        package='plotter_controller',
        executable='tf_trajectory_publisher.py',
        name='tf_trajectory_publisher',
        parameters=[{
            'target_frame': 'link_finger',
            'reference_frame': 'base_link',
            'duration': 10.0,
        }],
    )

    return LaunchDescription([
        display_launch,
        test_swing_node,
        tf_trajectory_node,
    ])
