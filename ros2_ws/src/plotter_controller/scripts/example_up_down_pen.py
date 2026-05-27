#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ExampleUpDownPenNode(Node):

    L1 = 0.200015
    L2 = 0.150000
    L3 = 0.025000

    PEN_DOWN_DEG = 0.0
    PEN_UP_DEG = -30.0

    def __init__(self):
        super().__init__('example_up_down_pen')

        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)

        self.rate = 50.0
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.t = 0.0

        self.circle_center_x = 0.30
        self.circle_center_y = 0.10
        self.circle_radius = 0.05
        self.circle_period = 5.0

        self.theta3_deg = self.PEN_UP_DEG

        self.get_logger().info('Example up/down pen node started')

    def inverse_kinematics_2dof(self, x: float, y: float):
        L2_eff = self.L2 + self.L3
        d = np.sqrt(x**2 + y**2)

        if d > (self.L1 + L2_eff) or d < abs(self.L1 - L2_eff):
            self.get_logger().warn(f'Target ({x:.3f}, {y:.3f}) is unreachable')
            return None, None

        cos_theta2 = (d**2 - self.L1**2 - L2_eff**2) / (2 * self.L1 * L2_eff)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
        theta2 = -np.arccos(cos_theta2)

        alpha = np.arctan2(y, x)
        beta = np.arctan2(L2_eff * np.sin(theta2), self.L1 + L2_eff * np.cos(theta2))
        theta1 = alpha - beta

        return np.rad2deg(theta1), np.rad2deg(theta2)

    def publish_joints(self, theta1_deg, theta2_deg, theta3_deg):
        theta1_rad = np.deg2rad(-theta1_deg)
        theta2_rad = np.deg2rad(theta2_deg)
        theta3_rad = np.deg2rad(-theta3_deg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['rev1', 'rev2', 'rev3', 'rev4']
        js.position = [theta1_rad, theta2_rad, theta3_rad, 0.0]
        js.velocity = []
        js.effort = []
        self.publisher_joint_states.publish(js)

    def circle_position(self, t):
        omega = 2.0 * np.pi / self.circle_period
        angle = omega * t
        x = self.circle_center_x + self.circle_radius * np.cos(angle)
        y = self.circle_center_y + self.circle_radius * np.sin(angle)
        return x, y

    def timer_callback(self):
        T_HOLD = 1.0
        T_LOWER = T_HOLD + 0.5
        T_DRAW_END = T_LOWER + self.circle_period
        T_LIFT = T_DRAW_END + 0.5
        T_CYCLE = T_LIFT + 1.0

        phase = self.t % T_CYCLE

        x, y = self.circle_position(0.0)
        theta1_deg, theta2_deg = self.inverse_kinematics_2dof(x, y)

        if theta1_deg is None:
            self.t += 1.0 / self.rate
            return

        if phase < T_HOLD:
            self.theta3_deg = self.PEN_UP_DEG
            self.get_logger().info('Pen up, holding at start', throttle_duration_sec=1.0)

        elif phase < T_LOWER:
            self.theta3_deg = self.PEN_DOWN_DEG
            self.get_logger().info('Lowering pen', throttle_duration_sec=1.0)

        elif phase < T_DRAW_END:
            draw_t = phase - T_LOWER
            x, y = self.circle_position(draw_t)
            result = self.inverse_kinematics_2dof(x, y)
            if result[0] is not None:
                theta1_deg, theta2_deg = result
            self.theta3_deg = self.PEN_DOWN_DEG
            self.get_logger().info(
                f'Drawing circle: ({x:.3f}, {y:.3f})', throttle_duration_sec=1.0)

        else:
            self.theta3_deg = self.PEN_UP_DEG
            self.get_logger().info('Lifting pen', throttle_duration_sec=1.0)

        self.publish_joints(theta1_deg, theta2_deg, self.theta3_deg)
        self.t += 1.0 / self.rate


def main(args=None):
    rclpy.init(args=args)
    node = ExampleUpDownPenNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
