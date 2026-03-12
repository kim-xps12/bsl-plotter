#!/usr/bin/env python3
"""
Lesson controller node for BSL Plotter robot.

Educational/training node demonstrating inverse kinematics control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from plotter_controller.control_arm import ControlArm


def target_line(t: float) -> tuple:
    """
    Calculate target position for a given time.

    Args:
        t: Time in seconds

    Returns:
        Tuple of (x, y) target position in mm
    """
    # TODO: Implement trajectory generation
    x = 200
    y = 200
    return x, y


class LessonControllerNode(Node):
    """Node for demonstrating inverse kinematics control."""

    def __init__(self):
        super().__init__('lesson_controller')

        # Publisher for joint states
        self.publisher_angles = self.create_publisher(JointState, 'joint_state', 10)

        # Control arm instance
        self.arm = ControlArm(self, self.publisher_angles)

        # Timer for control loop (100 Hz)
        self.rate = 100.0  # Hz
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        # Time counter
        self.t = 0.0

        self.get_logger().info('Lesson controller node started')

    def timer_callback(self):
        """Timer callback for control loop."""
        # Get target position
        x, y = target_line(self.t)
        self.get_logger().info(f'Target position: ({x}, {y})')

        # Calculate inverse kinematics
        theta1, theta2 = self.arm.solve_ik_deg(x, y)

        # Update arm angles
        self.arm.update_angles(theta1, theta2)

        # Increment time
        self.t += 1.0 / self.rate

        self.get_logger().info(f'x: {x}')


def main(args=None):
    rclpy.init(args=args)
    node = LessonControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
