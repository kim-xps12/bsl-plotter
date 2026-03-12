#!/usr/bin/env python3
"""
Main launch file for BSL Plotter robot system.

Launches the complete system including:
- Robot description (URDF)
- Robot state publisher
- Test swing node (for demonstration)
- RViz2 visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directories
    description_pkg = get_package_share_directory('bsl_plotter_description')
    controller_pkg = get_package_share_directory('plotter_controller')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    use_hardware_arg = DeclareLaunchArgument(
        name='use_hardware',
        default_value='false',
        description='Whether to launch hardware driver (feetech_driver)'
    )

    # Include robot display launch
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'gui': 'false',
        }.items()
    )

    # Test swing node (only when not using hardware)
    test_swing_node = Node(
        package='plotter_controller',
        executable='test_swing.py',
        name='test_swing',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_hardware', default='false'))
    )

    # Feetech driver node (only when using hardware)
    feetech_driver_node = Node(
        package='plotter_controller',
        executable='feetech_driver.py',
        name='feetech_driver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_hardware'))
    )

    return LaunchDescription([
        use_rviz_arg,
        use_hardware_arg,
        display_launch,
        test_swing_node,
        feetech_driver_node,
    ])
