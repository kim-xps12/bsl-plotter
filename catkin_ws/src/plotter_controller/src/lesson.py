#!/usr/bin/env python3

import math
import numpy as np

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from arm_control import ArmControl


target_x = 200
target_y = 200

SPEED = 2

def callback(data):
    global target_x, target_y

    if data.data == 'a':
        if target_x > 100:
            target_x -= SPEED
    if data.data == 'd':
        if target_x < 300:
            target_x += SPEED
    if data.data == 's':
        if target_y > 100:
            target_y -= SPEED
    if data.data == 'w':
        if target_y < 300:
            target_y += SPEED

def target_line(t):

    # Lesson1
    """
    if(t<=7):
        x = 20*t + 200
    else:
        x = 340
    y = 50

    """

    # Lesson2
    """
    T = 5 #[sec]
    w = 2*np.pi/T #[rad/s]

    x = 100*np.sin(w*t)+250 #[mm]
    y = 50 #[mm]
    """

    # Lesson3
    r = 50
    T = 5 #[sec]
    w = 2*np.pi/T #[rad/s]
    x = r*np.sin(w*t)+150 #[mm]
    y = r*np.cos(w*t)+150 #[mm]
    return x, y


def main():

    rospy.init_node('ik_solver')
    publisher_angles = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.Subscriber('/keyboard_publisher/key', String, callback)

    rate = 100 #[hz]

    r = rospy.Rate(rate) #[hz]
    
    theta1 = 0 #[rad]
    theta2 = 0  #[rad]
    t = 0 #[sec]
    
    arm = ArmControl(publisher_angles)

    # up pen
    arm.up_pen()

    while not rospy.is_shutdown():

        # x, y = target_line(t)
        x = target_x
        y = target_y
        theta1, theta2 = arm.solve_ik_deg(x, y)

        arm.update_angles(theta1, theta2)

        t = t + (1/rate) #[sec]

        rospy.loginfo(x)
        r.sleep()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

