#!/usr/bin/env python3

import math
import numpy as np

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from sensor_msgs.msg import JointState


class ArmControl:
    # Written by ChatGPT

   
    l1 = 205.0 #[mm]
    l2 = 150.0 #[mm]
    l3 = 22.50 #[mm]
    l23 = l2 + l3
 
    theta1 = 0.0
    theta2 = 0.0
    theta3 = 0.0

    js = JointState()
    js.name = ["1", "2", "3"]
    js.position = [theta1, theta2, theta3]
    js.velocity = [0, 0, 0]
    js.effort = [0, 0, 0]

    def __init__(self, pub):
        self.publisher_angles = pub
 

    def __inverse_kinematics(self, x, y):
        theta2 = math.acos((x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2))
        k1 = self.l1**2 + x**2 + y**2 - self.l2**2
        k2 = 2*self.l1*math.sqrt(x**2 + y**2)
    
        theta1 = math.atan2(y, x) - math.acos(k1/k2)
    
        return theta1, theta2


    def solve_ik_deg(self, x, y):

        theta1, theta2 = self.__inverse_kinematics(x, y) 
        
        return np.rad2deg(theta1), np.rad2deg(theta2)


    def update_angles(self, th1, th2):

        th3 = self.theta3

        name_upper_arm = "link_upper_arm_v7_1"
        name_fore_arm = "link_fore_arm_v4_1"
        name_hand = "link_hand_v9_1"
        name_finger = "link_finger"

        self.js.position = [th1, th2, th3]
        self.publisher_angles.publish(self.js)

        br = tf2_ros.TransformBroadcaster()

        t0 = geometry_msgs.msg.TransformStamped()
        t0.header.stamp = rospy.Time.now()
        t0.header.frame_id = "base_link"
        t0.child_frame_id = name_upper_arm
        t0.transform.translation.x = 0
        t0.transform.translation.y = 0
        t0.transform.translation.z = 0.01
        q0 = tf_conversions.transformations.quaternion_from_euler(0, 0, np.deg2rad(th1))
        t0.transform.rotation.x = q0[0]
        t0.transform.rotation.y = q0[1]
        t0.transform.rotation.z = q0[2]
        t0.transform.rotation.w = q0[3]

        t1 = geometry_msgs.msg.TransformStamped()
        t1.header.stamp = rospy.Time.now()
        t1.header.frame_id = name_upper_arm
        t1.child_frame_id = name_fore_arm
        t1.transform.translation.x = 0.2
        t1.transform.translation.y = 0
        t1.transform.translation.z = 0.038
        q1 = tf_conversions.transformations.quaternion_from_euler(0, 0, np.deg2rad(th2))
        t1.transform.rotation.x = q1[0]
        t1.transform.rotation.y = q1[1]
        t1.transform.rotation.z = q1[2]
        t1.transform.rotation.w = q1[3]

        t2 = geometry_msgs.msg.TransformStamped()
        t2.header.stamp = rospy.Time.now()
        t2.header.frame_id = name_fore_arm
        t2.child_frame_id = name_hand
        t2.transform.translation.x = 0.15
        t2.transform.translation.y = -0.01925
        t2.transform.translation.z = 0.015
        q2 = tf_conversions.transformations.quaternion_from_euler(0, np.deg2rad(th3), 0)
        t2.transform.rotation.x = q2[0]
        t2.transform.rotation.y = q2[1]
        t2.transform.rotation.z = q2[2]
        t2.transform.rotation.w = q2[3]

        t3 = geometry_msgs.msg.TransformStamped()
        t3.header.stamp = rospy.Time.now()
        t3.header.frame_id = name_hand
        t3.child_frame_id = name_finger
        t3.transform.translation.x = 0.0225
        t3.transform.translation.y = 0.01925
        t3.transform.translation.z = 0.0
        q3 = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t3.transform.rotation.x = q2[0]
        t3.transform.rotation.y = q2[1]
        t3.transform.rotation.z = q2[2]
        t3.transform.rotation.w = q2[3]


        br.sendTransform(t0)
        br.sendTransform(t1)
        br.sendTransform(t2)
        br.sendTransform(t3)

    
    def up_pen(self):
        self.theta3 = -20
        self.update_angles(self.theta1, self.theta2)

