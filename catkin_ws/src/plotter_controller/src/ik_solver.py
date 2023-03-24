#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

import tf2_ros
import tf_conversions
import geometry_msgs.msg
import numpy as np


def solve_ik():

    theta1 = 10.0 #[deg]
    theta2 = 20.0 #[deg]

    return(theta1, theta2)


def update_tf(th1, th2, th3):

    br = tf2_ros.TransformBroadcaster()

    t0 = geometry_msgs.msg.TransformStamped()
    t0.header.stamp = rospy.Time.now()
    t0.header.frame_id = "base_link"
    t0.child_frame_id = "link_upper_arm_v5_1"
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
    t1.header.frame_id = "link_upper_arm_v5_1"
    t1.child_frame_id = "link_fore_arm_v3_1"
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
    t2.header.frame_id = "link_fore_arm_v3_1"
    t2.child_frame_id = "link_hand_v4_1"
    t2.transform.translation.x = 0.15
    t2.transform.translation.y = -0.01925
    t2.transform.translation.z = 0.015
    q2 = tf_conversions.transformations.quaternion_from_euler(0, np.deg2rad(th3), 0)
    t2.transform.rotation.x = q2[0]
    t2.transform.rotation.y = q2[1]
    t2.transform.rotation.z = q2[2]
    t2.transform.rotation.w = q2[3]

    br.sendTransform(t0)
    br.sendTransform(t1)
    br.sendTransform(t2)


def main():

    rospy.init_node('ik_solver')
    publisher_angles = rospy.Publisher('joint_state', JointState, queue_size=10)

    rate = 100 #[hz]

    r = rospy.Rate(rate) #[hz]
    
    theta1 = 0 #[rad]
    theta2 = 0  #[rad]
    theta3 = 0  #[rad]
    t = 0 #[sec]
    while not rospy.is_shutdown():

        #th1, th2 = solve_ik()
        
        js = JointState()
        js.name = ["1", "2", "3"]
        js.position = [theta1, theta2, theta3]
        js.velocity = [0, 0]
        js.effort = [0, 0]

        publisher_angles.publish(js)
       
        theta1 = 45 + 20*np.sin(1*t)
        theta2 = theta1*(-1)
        t = t + (1/rate) #[sec]

        update_tf(theta1, theta2, theta3)
        rospy.loginfo(js.name)
        rospy.loginfo(js.position)
        r.sleep()


if __name__=='__main__':
    main()
