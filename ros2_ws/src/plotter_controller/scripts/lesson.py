#!/usr/bin/env python3
"""
BSL Plotter ROS 2 レッスン

target_position(t) を実装して、ロボットアームに好きな軌道を描かせましょう。

実行方法:
  pixi run ros2 launch plotter_controller lesson.launch.py

答え合わせ:
  scripts/example_up_down_pen.py が模範解答です（円を描く実装）
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def target_position(t):
    """時刻 t [sec] における手先の目標位置 (x, y) [mm] を返す。

    ロボットアームの到達範囲はおよそ 25 ~ 375 mm です。
    この範囲内の座標を返すように実装してください。
    """
    # ここを書き換えてください
    x = 300
    y = 100

    return x, y


# ========================================================================
# 以下はノード本体（変更不要）
# ========================================================================

class LessonNode(Node):

    L1 = 205.0
    L2 = 150.0
    L3 = 22.5

    def __init__(self):
        super().__init__('lesson')

        self.publisher_joint_states = self.create_publisher(
            JointState, '/joint_states', 10)

        self.rate = 50.0
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.t = 0.0

        self.get_logger().info('Lesson node started')

    def inverse_kinematics_2dof(self, x, y):
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

    def publish_joints(self, theta1_deg, theta2_deg):
        theta1_rad = np.deg2rad(-theta1_deg)
        theta2_rad = np.deg2rad(theta2_deg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['rev1', 'rev2', 'rev3', 'rev4']
        js.position = [theta1_rad, theta2_rad, 0.0, 0.0]
        js.velocity = []
        js.effort = []
        self.publisher_joint_states.publish(js)

    def timer_callback(self):
        x, y = target_position(self.t)
        theta1_deg, theta2_deg = self.inverse_kinematics_2dof(x, y)

        if theta1_deg is not None:
            self.publish_joints(theta1_deg, theta2_deg)
            self.get_logger().info(
                f'Target: ({x:.3f}, {y:.3f})', throttle_duration_sec=1.0)

        self.t += 1.0 / self.rate


def main(args=None):
    rclpy.init(args=args)
    node = LessonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
