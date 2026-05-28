#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


class TfTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('tf_trajectory_publisher')

        self.declare_parameter('target_frame', 'link_finger')
        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('duration', 10.0)

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, '/tf_trajectory', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.path = Path()
        self.path.header.frame_id = self.reference_frame

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.reference_frame, self.target_frame, Time())
        except TransformException:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.reference_frame
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation

        self.path.poses.append(pose)

        now = self.get_clock().now()
        while self.path.poses:
            stamp = Time.from_msg(self.path.poses[0].header.stamp)
            if (now - stamp).nanoseconds / 1e9 > self.duration:
                self.path.poses.pop(0)
            else:
                break

        self.path.header.stamp = now.to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = TfTrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
