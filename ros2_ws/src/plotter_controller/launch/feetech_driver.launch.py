#!/usr/bin/env python3
"""
Launch file for BSL Plotter Feetech driver.

Launches the feetech_driver node for hardware servo control.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Feetech driver node
    feetech_driver_node = Node(
        package='plotter_controller',
        executable='feetech_driver.py',
        name='feetech_driver',
        output='screen'
    )

    return LaunchDescription([
        feetech_driver_node,
    ])
