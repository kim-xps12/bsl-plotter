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

# for convert gcode-coorinate to robot-coorinate
offset_x = 80 #[mm]
offset_y = 80 #[mm]

def parse_move_command(line, mode):

    parts = line.split()
    command_type = parts[0]
    command = {'type': command_type}

    # Absolute mode
    if mode == "G90":
        for part in parts[1:]:
            if part.startswith('X'):
                command['x'] = float(part[1:]) + offset_x
            elif part.startswith('Y'):
                command['y'] = float(part[1:]) + offset_y
            elif part.startswith('F'):
                command['f'] = float(part[1:])
    
    # Write incremental mode (G91) here, if you wat it.
    # elif mode == "G91":
    # ...

    return command


def get_target_speed(cmds):

    speed_mm_per_min = [command['f'] for command in cmds if command['type'] == 'G1' and 'f' in command]
    
    if len(speed_mm_per_min)>0:
        speed_mm_per_sec = speed_mm_per_min[0] / 60
        return float(speed_mm_per_sec)
    else:
        return 10 #[mm/s], default


def load_gcode(file_path):

    commands = []
    mode = None  # 'G90' or 'G91'

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith(';') or not line:
                continue  # Ignore comment and empty line

            elif line.startswith('G21'):  # check millimetre setting
                continue  # Nothin to do for G21

            elif line.startswith('G90'):
                mode = 'G90'  # Absolute mode
            #elif line.startswith('G91'):
            #    mode = 'G91'  # Incremental mode (hint for feature)

            elif mode and line.startswith(('G0', 'G1')):
                command = parse_move_command(line, mode)
                commands.append(command)

    if not mode or mode not in ['G90']:
        raise ValueError("G-code file must specify coordinate mode G90.")
    
    return commands


def get_points_on_line(p0, p1, ds):

    x0, y0 = p0
    x1, y1 = p1
    
    dx = x1 - x0
    dy = y1 - y0
    
    length = (dx**2 + dy**2)**0.5
    
    # vertical line
    if dx == 0:
        step = ds if y1 > y0 else -ds
        points = [(x0, y0 + i * step) for i in range(int(length / ds) + 1)]
        return points
    
    # horizontal or ramp line
    m = dy / dx
    b = y0 - m * x0
    
    points = []
    num_points = int(length / ds) + 1
    for i in range(num_points):
        x = x0 + (dx / length) * ds * i
        y = m * x + b
        points.append((x, y))
    
    return points


def main():

    rospy.init_node('draw_gcode')
    publisher_angles = rospy.Publisher('joint_state', JointState, queue_size=10)

    rate = 100 #[hz]
    r = rospy.Rate(rate) #[hz] 
    theta1 = 0 #[rad]
    theta2 = 0  #[rad]
    t = 0 #[sec]

    path_gcode = "../data/star.gcode"

    arm = ControlArm(publisher_angles)

    # up pen
    #arm.up_pen()

    gcode_commands = load_gcode(path_gcode)
    speed = get_target_speed(gcode_commands) #[mm/sec]
    ds = speed / rate #[mm]

    while not rospy.is_shutdown():
 
        p0 = (0+offset_x, 0+offset_y)
    
        for command in gcode_commands:
            if command['type'] in ['G0', 'G1']:

                p1 = (command.get('x', p0[0]), command.get('y', p0[1]))
                points = get_points_on_line(p0, p1, ds)
            
                for [x, y] in points:

                    theta1, theta2 = arm.solve_ik_deg(x, y)
                    arm.update_angles(theta1, theta2)

                    t = t + (1/rate) #[sec]
                    rospy.loginfo([x, y])
                    r.sleep()

                p0 = p1
    

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

