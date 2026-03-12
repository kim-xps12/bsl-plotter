#!/usr/bin/env python3
"""
Launch file for displaying BSL Plotter robot model in RViz2.

This launch file loads the robot URDF, starts robot_state_publisher,
joint_state_publisher_gui, and RViz2 for visualization.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('bsl_plotter_description')

    # Paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'bsl_plotter.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )

    # Robot description from xacro
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
