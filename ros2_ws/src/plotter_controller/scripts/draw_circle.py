#!/usr/bin/env python3
"""
Circle drawing node for BSL Plotter robot.

Generates circular trajectory using inverse kinematics for the 2-DoF planar arm.
The arm draws a circle on the XY plane by computing joint angles from Cartesian positions.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DrawCircleNode(Node):
    """Node that generates circular trajectory for the robot arm."""

    # Robot link lengths (from URDF)
    L1 = 0.200015  # upper_arm length [m]
    L2 = 0.150000  # fore_arm length [m]
    L3 = 0.025000  # hand length [m]

    def __init__(self):
        super().__init__('draw_circle')

        # Publisher for joint states
        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)

        # Timer for update loop (50 Hz)
        self.rate = 50.0  # Hz
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        # Time counter
        self.t = 0.0  # seconds

        # Circle parameters
        self.circle_center_x = 0.30   # Circle center X [m]
        self.circle_center_y = 0.10   # Circle center Y [m] (shifted to +Y for 1st quadrant)
        self.circle_radius = 0.05     # Circle radius [m]
        self.circle_period = 5.0      # Time to complete one circle [s]

        # Fixed wrist angle (rev3) to keep pen perpendicular
        self.theta3_deg = 0.0

        self.get_logger().info('Draw circle node started')
        self.get_logger().info(
            f'Circle: center=({self.circle_center_x}, {self.circle_center_y}), '
            f'radius={self.circle_radius}, period={self.circle_period}s'
        )

    def inverse_kinematics_2dof(self, x: float, y: float) -> tuple:
        """
        Compute inverse kinematics for 2-DoF planar arm.

        Uses the combined length of fore_arm + hand as the second link.

        Args:
            x: Target X position [m]
            y: Target Y position [m]

        Returns:
            tuple: (theta1_deg, theta2_deg) joint angles in degrees,
                   or (None, None) if position is unreachable
        """
        # Effective second link length (fore_arm + hand)
        L2_eff = self.L2 + self.L3

        # Distance from origin to target
        d = np.sqrt(x**2 + y**2)

        # Check if target is reachable
        if d > (self.L1 + L2_eff) or d < abs(self.L1 - L2_eff):
            self.get_logger().warn(
                f'Target ({x:.3f}, {y:.3f}) is unreachable. d={d:.3f}, '
                f'range=[{abs(self.L1 - L2_eff):.3f}, {self.L1 + L2_eff:.3f}]'
            )
            return None, None

        # Cosine law for elbow angle
        cos_theta2 = (d**2 - self.L1**2 - L2_eff**2) / (2 * self.L1 * L2_eff)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  # Numerical safety

        # Elbow angle (choose elbow-down configuration: negative angle)
        theta2 = -np.arccos(cos_theta2)

        # Shoulder angle
        alpha = np.arctan2(y, x)
        beta = np.arctan2(L2_eff * np.sin(theta2), self.L1 + L2_eff * np.cos(theta2))
        theta1 = alpha - beta

        # Convert to degrees
        theta1_deg = np.rad2deg(theta1)
        theta2_deg = np.rad2deg(theta2)

        return theta1_deg, theta2_deg

    def timer_callback(self):
        """Timer callback for generating circular trajectory."""
        # Angular position on circle
        omega = 2.0 * np.pi / self.circle_period
        angle = omega * self.t

        # Target position on circle
        target_x = self.circle_center_x + self.circle_radius * np.cos(angle)
        target_y = self.circle_center_y + self.circle_radius * np.sin(angle)

        # Compute inverse kinematics
        theta1_deg, theta2_deg = self.inverse_kinematics_2dof(target_x, target_y)

        if theta1_deg is None or theta2_deg is None:
            self.get_logger().error('Failed to compute IK, skipping this step')
            self.t += 1.0 / self.rate
            return

        # theta3 keeps the hand aligned
        theta3_deg = self.theta3_deg

        # Convert to radians for ROS 2
        # Axis direction mapping (same as test_swing.py):
        # - rev1: URDF axis = (0, 0, -1) -> negate
        # - rev2: URDF axis = (0, 0, +1) -> keep same
        # - rev3: URDF axis = (0, -1, 0) -> negate
        theta1_rad = np.deg2rad(-theta1_deg)  # negate for -Z axis
        theta2_rad = np.deg2rad(theta2_deg)   # keep for +Z axis
        theta3_rad = np.deg2rad(-theta3_deg)  # negate for -Y axis

        # Create and publish JointState message
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['rev1', 'rev2', 'rev3']
        js.position = [theta1_rad, theta2_rad, theta3_rad]
        js.velocity = []
        js.effort = []

        self.publisher_joint_states.publish(js)

        # Update time
        self.t += 1.0 / self.rate

        # Log every second
        if int(self.t * self.rate) % int(self.rate) == 0:
            self.get_logger().info(
                f'Target: ({target_x:.3f}, {target_y:.3f}), '
                f'Joints (deg): [{theta1_deg:.1f}, {theta2_deg:.1f}, {theta3_deg:.1f}]'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
