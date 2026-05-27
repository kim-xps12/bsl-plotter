#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('bsl_plotter_description')

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'display.launch.py')
        ),
        launch_arguments={'gui': 'false'}.items()
    )

    example_node = Node(
        package='plotter_controller',
        executable='example_up_down_pen.py',
        name='example_up_down_pen',
        output='screen'
    )

    return LaunchDescription([
        display_launch,
        example_node,
    ])
