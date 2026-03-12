#!/usr/bin/env python3
"""
Launch file for BSL Plotter circle drawing demonstration.

Launches the draw_circle node along with robot visualization in RViz2.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directories
    description_pkg = get_package_share_directory('bsl_plotter_description')

    # Include robot display launch (RViz only, no joint_state_publisher_gui)
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'display.launch.py')
        ),
        launch_arguments={'gui': 'false'}.items()
    )

    # Draw circle node
    draw_circle_node = Node(
        package='plotter_controller',
        executable='draw_circle.py',
        name='draw_circle',
        output='screen'
    )

    return LaunchDescription([
        display_launch,
        draw_circle_node,
    ])
