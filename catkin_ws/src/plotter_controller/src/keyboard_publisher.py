#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import readchar

def keyboard_publisher():
    pub = rospy.Publisher('/keyboard_publisher/key', String, queue_size=10)
    rospy.init_node('keyboard_publisher', anonymous=True)

    rospy.loginfo('------------------------------------------')
    rospy.loginfo('w: up, a: left, d: right, s: down, q: exit')

    while True:
        key_msg = readchar.readchar()
        if key_msg == 'q':
            break
        pub.publish(key_msg)

if __name__ == '__main__':
    try:
        keyboard_publisher()
    except rospy.ROSInterruptException:
        pass
