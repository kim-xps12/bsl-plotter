#!/usr/bin/env python3
"""
Test swing node for BSL Plotter robot.

Generates sinusoidal joint motion patterns for testing and demonstration.
Publishes to /joint_states topic which is consumed by robot_state_publisher.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TestSwingNode(Node):
    """Node that generates test swing motion for the robot arm."""

    def __init__(self):
        super().__init__('test_swing')

        # Publisher for joint states (standard ROS 2 topic name)
        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)

        # Timer for update loop (50 Hz)
        self.rate = 50.0  # Hz
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        # Time counter
        self.t = 0.0  # seconds

        self.get_logger().info('Test swing node started')

    def timer_callback(self):
        """Timer callback for generating swing motion."""
        # Calculate joint angles in degrees (matching ROS1 behavior)
        # ROS1: theta1 and theta2 both use +Z axis rotation in TF broadcast
        # URDF: rev1 axis = (0,0,-1), rev2 axis = (0,0,+1)
        theta1_deg = 45.0 + 20.0 * np.sin(1.0 * self.t)
        theta2_deg = -theta1_deg  # fore_arm rotates opposite to upper_arm
        theta3_deg = -15.0 + 15.0 * np.sin(1.0 * self.t)

        # Convert to radians for ROS 2
        # Axis direction mapping (ROS1 TF +Z broadcast vs URDF axis):
        # - rev1: URDF axis = (0, 0, -1), ROS1 used +Z -> negate
        # - rev2: URDF axis = (0, 0, +1), ROS1 used +Z -> keep same
        # - rev3: URDF axis = (0, -1, 0), ROS1 used +Y -> negate
        theta1_rad = np.deg2rad(-theta1_deg)  # negate for -Z axis
        theta2_rad = np.deg2rad(theta2_deg)   # keep for +Z axis
        theta3_rad = np.deg2rad(-theta3_deg)  # negate for -Y axis

        # Create and publish JointState message
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        # Joint names must match URDF joint names (3-DoF arm: rev1, rev2, rev3)
        js.name = ['rev1', 'rev2', 'rev3']
        js.position = [theta1_rad, theta2_rad, theta3_rad]
        js.velocity = []
        js.effort = []

        self.publisher_joint_states.publish(js)

        # Update time
        self.t += 1.0 / self.rate

        self.get_logger().info(
            f'Joint angles (deg): [{theta1_deg:.1f}, {theta2_deg:.1f}, {theta3_deg:.1f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TestSwingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
