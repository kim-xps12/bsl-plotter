#!/usr/bin/env python3

import math
import numpy as np

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import time
from sensor_msgs.msg import JointState

from control_arm import ControlArm


def target_line_circle(t):

    x = 30*np.sin((2*np.pi/5)*t) + 100
    y = 30*np.cos((2*np.pi/5)*t) + 100

    return x, y


def target_line_horizontal(t):
    
    x = 20*t -70
    y = 130

    return x, y



def main():

    rospy.init_node('lesson_controller')
    publisher_angles = rospy.Publisher('joint_state', JointState, queue_size=10)

    rate = 100 #[hz]

    r = rospy.Rate(rate) #[hz] 
    theta1 = 0 #[rad]
    theta2 = 0  #[rad]
    t = 0 #[sec]

    arm = ControlArm(publisher_angles)

    # Draw outside circle
    while (not rospy.is_shutdown()) and (t<=5):

        x, y = target_line_circle(t)
        theta1, theta2 = arm.solve_ik_deg(x, y)

        arm.update_angles(theta1, theta2)

        t = t + (1/rate) #[sec]

        rospy.loginfo(f"theta1: {theta1:.3f}, theta2: {theta2:.3f}")
        r.sleep()

    # lift the pen
    arm.lift_pen()

    # Wait while lifting the pen
    while (not rospy.is_shutdown()) and (t<=10):

        t = t + (1/rate) #[sec]

        rospy.loginfo("lifting, wait for 5 sec...")
        r.sleep()


    # Draw bridle line
    while (not rospy.is_shutdown()) and (t<=15):

        x, y = target_line_horizontal(t)
        theta1, theta2 = arm.solve_ik_deg(x, y)

        arm.update_angles(theta1, theta2)

        t = t + (1/rate) #[sec]

        rospy.loginfo(f"theta1: {theta1:.3f}, theta2: {theta2:.3f}")
        r.sleep()

    # lower the pen
    arm.lower_pen()

    # Wait while lowering the pen
    while (not rospy.is_shutdown()) and (t<=20):

        t = t + (1/rate) #[sec]

        rospy.loginfo("lowering, wait for 5 sec...")
        r.sleep()




if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

