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


x, y = 100, 100


def callback():
    global x
    global y
    delta = 5
    rospy.loginfo(rospy.get_caller_id() + 'recieved "%s"', data.data)
    if data.data == "UP":
        y += delta
    if data.data == "DOWN":
        y -= delta
    if data.data == "LEFT":
        x -= delta
    if data.data == "RIGHT":
        x += delta

def main():
    global x
    global y

    rospy.init_node('ik_solver')
    publisher_angles = rospy.Publisher('joint_state', JointState, queue_size=10)

    rospy.Subscriber('chatter', String, callback)
    rate = 100 #[hz]

    r = rospy.Rate(rate) #[hz] 
    theta1 = 0 #[rad]
    theta2 = 0  #[rad]
    t = 0 #[sec]

    arm = ArmControl(publisher_angles)

    # up pen
    #arm.up_pen()

    while not rospy.is_shutdown():

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

