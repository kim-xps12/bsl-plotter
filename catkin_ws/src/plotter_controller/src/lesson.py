#!/usr/bin/env python3

import math
import numpy as np

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import time
from sensor_msgs.msg import JointState

from arm_control import ArmControl


def target_line(t):
    """
    # Lesson1
    if(t<=7):
        x = 20*t + 200
    else:
        x = 340
    y = 50
    # Lesson2
    T = 5 #[sec]
    w = 2*np.pi/T #[rad/s]

    x = 100*np.sin(w*t)+200 #[mm]
    y = 50 #[mm]

    """
    #"""
    # Lesson3
    r = 50
    T = 5 #[sec]
    w = 2*np.pi/T #[rad/s]
    x = r*np.sin(w*t)+150 #[mm]
    y = r*np.cos(w*t)+150 #[mm]
    #"""

    """
    #Lesson
    T = 5
    a = 1/100
    w = 2*np.pi/T
    x = 100*np.sin(w*t) + 200
    y = a*((x - 200)**2) + 50
    """
    return x, y


def main():

    rospy.init_node('ik_solver')
    publisher_angles = rospy.Publisher('joint_state', JointState, queue_size=10)

    rate = 100 #[hz]

    r = rospy.Rate(rate) #[hz] 
    theta1 = 0 #[rad]
    theta2 = 0  #[rad]
    t = 0 #[sec]

    arm = ArmControl(publisher_angles)

    # up pen
    #arm.up_pen()

    while not rospy.is_shutdown():

        x, y = target_line(t)
        print(x, y)
        theta1, theta2 = arm.solve_ik_deg(x, y)

        #theta1 = 0.0
        #theta2 = 0.0
        arm.update_angles(theta1, theta2)

        t = t + (1/rate) #[sec]

        rospy.loginfo(x)
        r.sleep()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

