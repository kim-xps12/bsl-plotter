#!/usr/bin/env python3
"""
Control arm module for BSL Plotter robot.

Provides inverse kinematics calculations and TF broadcasting for the 3-DOF arm.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
import tf2_ros
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState


class ControlArm:
    """
    Control class for the BSL Plotter robotic arm.

    Provides inverse kinematics and transform broadcasting functionality.
    """

    # Arm segment lengths [mm]
    l1 = 205.0
    l2 = 150.0
    l3 = 22.50
    l23 = l2 + l3

    theta1 = 0.0
    theta2 = 0.0
    theta3 = 0.0

    def __init__(self, node: Node, publisher):
        """
        Initialize the ControlArm.

        Args:
            node: ROS 2 node instance for TF broadcasting
            publisher: Publisher for JointState messages
        """
        self.node = node
        self.publisher_angles = publisher
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(node)

        self.js = JointState()
        self.js.name = ['1', '2', '3']
        self.js.position = [self.theta1, self.theta2, self.theta3]
        self.js.velocity = [0.0, 0.0, 0.0]
        self.js.effort = [0.0, 0.0, 0.0]

    def _inverse_kinematics(self, x: float, y: float) -> tuple:
        """
        Calculate inverse kinematics for 2-DOF planar arm.

        Args:
            x: Target x position [mm]
            y: Target y position [mm]

        Returns:
            Tuple of (theta1, theta2) in radians
        """
        theta2 = math.acos(
            (x**2 + y**2 - self.l1**2 - self.l23**2) / (2 * self.l1 * self.l23)
        )
        k1 = self.l1**2 + x**2 + y**2 - self.l23**2
        k2 = 2 * self.l1 * math.sqrt(x**2 + y**2)

        theta1 = math.atan2(y, x) - math.acos(k1 / k2)

        return theta1, theta2

    def solve_ik_deg(self, x: float, y: float) -> tuple:
        """
        Solve inverse kinematics and return angles in degrees.

        Args:
            x: Target x position [mm]
            y: Target y position [mm]

        Returns:
            Tuple of (theta1, theta2) in degrees
        """
        theta1, theta2 = self._inverse_kinematics(x, y)
        return np.rad2deg(theta1), np.rad2deg(theta2)

    def update_angles(self, th1: float, th2: float):
        """
        Update joint angles and broadcast TF transforms.

        Args:
            th1: Joint 1 angle [degrees]
            th2: Joint 2 angle [degrees]
        """
        th3 = self.theta3

        name_upper_arm = 'link_upper_arm_v7_1'
        name_fore_arm = 'link_fore_arm_v4_1'
        name_hand = 'link_hand_v9_1'
        name_finger = 'link_finger'

        self.js.header.stamp = self.node.get_clock().now().to_msg()
        self.js.position = [th1, th2, th3]
        self.publisher_angles.publish(self.js)

        now = self.node.get_clock().now().to_msg()

        # Transform: base_link -> upper_arm
        t0 = TransformStamped()
        t0.header.stamp = now
        t0.header.frame_id = 'base_link'
        t0.child_frame_id = name_upper_arm
        t0.transform.translation.x = 0.0
        t0.transform.translation.y = 0.0
        t0.transform.translation.z = 0.01
        q0 = quaternion_from_euler(0, 0, np.deg2rad(th1))
        t0.transform.rotation.x = q0[0]
        t0.transform.rotation.y = q0[1]
        t0.transform.rotation.z = q0[2]
        t0.transform.rotation.w = q0[3]

        # Transform: upper_arm -> fore_arm
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = name_upper_arm
        t1.child_frame_id = name_fore_arm
        t1.transform.translation.x = 0.2
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.038
        q1 = quaternion_from_euler(0, 0, np.deg2rad(th2))
        t1.transform.rotation.x = q1[0]
        t1.transform.rotation.y = q1[1]
        t1.transform.rotation.z = q1[2]
        t1.transform.rotation.w = q1[3]

        # Transform: fore_arm -> hand
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = name_fore_arm
        t2.child_frame_id = name_hand
        t2.transform.translation.x = 0.15
        t2.transform.translation.y = -0.01925
        t2.transform.translation.z = 0.015
        q2 = quaternion_from_euler(0, np.deg2rad(th3), 0)
        t2.transform.rotation.x = q2[0]
        t2.transform.rotation.y = q2[1]
        t2.transform.rotation.z = q2[2]
        t2.transform.rotation.w = q2[3]

        # Transform: hand -> finger
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = name_hand
        t3.child_frame_id = name_finger
        t3.transform.translation.x = 0.0225
        t3.transform.translation.y = 0.01925
        t3.transform.translation.z = 0.0
        t3.transform.rotation.x = q2[0]
        t3.transform.rotation.y = q2[1]
        t3.transform.rotation.z = q2[2]
        t3.transform.rotation.w = q2[3]

        self.tf_broadcaster.sendTransform(t0)
        self.tf_broadcaster.sendTransform(t1)
        self.tf_broadcaster.sendTransform(t2)
        self.tf_broadcaster.sendTransform(t3)

    def up_pen(self):
        """Raise the pen by setting theta3 to -20 degrees."""
        self.theta3 = -20
        self.update_angles(self.theta1, self.theta2)
