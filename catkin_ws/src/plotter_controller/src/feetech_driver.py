#!/usr/bin/env python3

import os
import math
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import sys, tty, termios
import numpy as np

from scservo_sdk import *
from sensor_msgs.msg import JointState


PI = math.pi
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

# Control table address
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_STS_GOAL_ACC = 41
ADDR_STS_GOAL_POSITION = 42
ADDR_STS_GOAL_SPEED = 46
ADDR_STS_PRESENT_POSITION = 56


# Default setting
BAUDRATE = 1000000           # SCServo default baudrate : 1000000
DEVICENAME = '/dev/ttyUSB0'
SCS_MINIMUM_POSITION_VALUE = 1024 #100    # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE = 3072 #4000   # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
SCS_MOVING_STATUS_THRESHOLD = 20                # SCServo moving status threshold
SCS_MOVING_SPEED = 0            # SCServo moving speed
SCS_MOVING_ACC = 0              # SCServo moving acc
protocol_end = 0                # SCServo bit end(STS/SMS=0, SCS=1)
index = 0


def conv_deg_cmd(angle):
    return int((angle+180)*11.37)


def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = PacketHandler(protocol_end)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_STS_GOAL_POSITION, 2)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

def set_scs_acc(scs_ids):
    
    for scs_id in scs_ids:
        
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, scs_id, ADDR_STS_GOAL_ACC, SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

def set_scs_speed(scs_ids):

    for scs_id in scs_ids:
        
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, scs_id, ADDR_STS_GOAL_SPEED, SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

def set_scs_torque_off(scs_ids):

    for scs_id in scs_ids:
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, scs_id, ADDR_SCS_TORQUE_ENABLE, 0)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))



def send_scs_params(scs_ids, params):

    assert len(scs_ids) == len(params)

    for i in range(len(scs_ids)):
    
        # Add SCServo#1 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(scs_ids[i], params[i])
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % scs_ids[i])
            quit()

    # Syncwrite goal position
    scs_comm_result = groupSyncWrite.txPacket()
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))


def callback(msg):
    js = msg
    scs_ids = [int(n) for n in js.name]
    directions = [-1, -1, 1]
    cmds = [conv_deg_cmd(p*d) for (p, d) in zip(js.position, directions)]

    # Allocate goal position value into byte array
    param_goal_positions = [ [SCS_LOBYTE(cmd), SCS_HIBYTE(cmd)] for cmd in cmds ]

    # Send
    send_scs_params(scs_ids, param_goal_positions)   

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    rospy.loginfo(js.position)


def subscriber():
    rospy.init_node('feetech_driver')
    rospy.Subscriber('joint_state', JointState, callback)
    rospy.spin()


if __name__=='__main__':
    subscriber()
