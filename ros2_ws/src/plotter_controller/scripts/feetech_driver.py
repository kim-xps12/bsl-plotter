#!/usr/bin/env python3
"""
Feetech servo driver node for BSL Plotter robot.

Interfaces between ROS 2 and Feetech (SCS) servo motors via serial communication.
Subscribes to /joint_states and converts URDF joint angles to servo commands.
"""

import math
import sys
import termios
import tty

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from scservo_sdk import (
        PortHandler,
        PacketHandler,
        GroupSyncWrite,
        COMM_SUCCESS,
        SCS_LOBYTE,
        SCS_HIBYTE,
    )
    SCSERVO_AVAILABLE = True
except ImportError:
    SCSERVO_AVAILABLE = False


# Control table addresses
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_STS_GOAL_ACC = 41
ADDR_STS_GOAL_POSITION = 42
ADDR_STS_GOAL_SPEED = 46
ADDR_STS_PRESENT_POSITION = 56

# Default settings
BAUDRATE = 1000000  # SCServo default baudrate
DEVICENAME = '/dev/tty.usbserial-210'  # macOS serial device
SCS_MINIMUM_POSITION_VALUE = 1024
SCS_MAXIMUM_POSITION_VALUE = 3072
SCS_MOVING_STATUS_THRESHOLD = 20
SCS_MOVING_SPEED = 0
SCS_MOVING_ACC = 0
PROTOCOL_END = 0  # SCServo bit end (STS/SMS=0, SCS=1)

# Joint name to servo ID mapping (3-DoF arm)
JOINT_TO_SERVO_ID = {
    'rev1': 1,
    'rev2': 2,
    'rev3': 3,
}

# Servo direction corrections (to match physical servo orientation)
# These values convert from URDF coordinate frame to actual servo direction
SERVO_DIRECTIONS = {
    1: 1,   # rev1: URDF -Z axis, servo direction
    2: -1,  # rev2: URDF +Z axis, servo direction (inverted)
    3: -1,  # rev3: URDF -Y axis, servo direction (inverted)
}


def conv_deg_cmd(angle: float) -> int:
    """
    Convert angle in degrees to servo command value.

    Args:
        angle: Angle in degrees

    Returns:
        Servo command value
    """
    return int((angle + 180) * 11.37)


class FeetechDriverNode(Node):
    """ROS 2 node for Feetech servo motor control."""

    def __init__(self):
        super().__init__('feetech_driver')

        self.declare_parameter('device', DEVICENAME)
        device = self.get_parameter('device').get_parameter_value().string_value
        self.get_logger().info(f'Using device: {device}')

        if not SCSERVO_AVAILABLE:
            self.get_logger().error(
                'scservo_sdk not available. Install with: pip install scservo-sdk'
            )
            return

        # Initialize serial port
        self.port_handler = PortHandler(device)
        self.packet_handler = PacketHandler(PROTOCOL_END)
        self.group_sync_write = GroupSyncWrite(
            self.port_handler, self.packet_handler, ADDR_STS_GOAL_POSITION, 2
        )

        # Open port
        if self.port_handler.openPort():
            self.get_logger().info('Succeeded to open the port')
        else:
            self.get_logger().error('Failed to open the port')
            return

        # Set baudrate
        if self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().info('Succeeded to change the baudrate')
        else:
            self.get_logger().error('Failed to change the baudrate')
            return

        # Subscribe to joint states (standard ROS 2 topic name)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Feetech driver node started')

    def set_scs_acc(self, scs_ids: list):
        """Set acceleration for all servos."""
        for scs_id in scs_ids:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, scs_id, ADDR_STS_GOAL_ACC, SCS_MOVING_ACC
            )
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f'{self.packet_handler.getTxRxResult(result)}'
                )
            elif error != 0:
                self.get_logger().warn(
                    f'{self.packet_handler.getRxPacketError(error)}'
                )

    def set_scs_speed(self, scs_ids: list):
        """Set speed for all servos."""
        for scs_id in scs_ids:
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, scs_id, ADDR_STS_GOAL_SPEED, SCS_MOVING_SPEED
            )
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f'{self.packet_handler.getTxRxResult(result)}'
                )
            elif error != 0:
                self.get_logger().warn(
                    f'{self.packet_handler.getRxPacketError(error)}'
                )

    def set_scs_torque_off(self, scs_ids: list):
        """Disable torque for all servos."""
        for scs_id in scs_ids:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, scs_id, ADDR_SCS_TORQUE_ENABLE, 0
            )
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f'{self.packet_handler.getTxRxResult(result)}'
                )
            elif error != 0:
                self.get_logger().warn(
                    f'{self.packet_handler.getRxPacketError(error)}'
                )

    def send_scs_params(self, scs_ids: list, params: list):
        """Send goal positions to servos using SyncWrite."""
        assert len(scs_ids) == len(params)

        for i, scs_id in enumerate(scs_ids):
            result = self.group_sync_write.addParam(scs_id, params[i])
            if not result:
                self.get_logger().error(
                    f'[ID:{scs_id:03d}] groupSyncWrite addparam failed'
                )
                return

        # Execute SyncWrite
        result = self.group_sync_write.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'{self.packet_handler.getTxRxResult(result)}'
            )

    def joint_state_callback(self, msg: JointState):
        """
        Handle incoming joint state commands.

        Converts joint angles from URDF coordinate frame (radians) to
        servo commands (degrees with direction correction).

        Args:
            msg: JointState message with target positions in radians
        """
        # Map joint names to servo IDs and get corresponding positions
        scs_ids = []
        angles_deg = []

        for name, position_rad in zip(msg.name, msg.position):
            if name not in JOINT_TO_SERVO_ID:
                continue

            servo_id = JOINT_TO_SERVO_ID[name]
            direction = SERVO_DIRECTIONS.get(servo_id, 1)

            # Convert radians to degrees and apply direction correction
            angle_deg = np.rad2deg(position_rad) * direction

            scs_ids.append(servo_id)
            angles_deg.append(angle_deg)

        if not scs_ids:
            return

        # Convert to servo commands
        cmds = [conv_deg_cmd(angle) for angle in angles_deg]

        # Allocate goal position values into byte arrays
        param_goal_positions = [
            [SCS_LOBYTE(cmd), SCS_HIBYTE(cmd)] for cmd in cmds
        ]

        # Send commands
        self.send_scs_params(scs_ids, param_goal_positions)

        # Clear syncwrite parameter storage
        self.group_sync_write.clearParam()

        self.get_logger().info(f'Servo commands (deg): {angles_deg}')

    def destroy_node(self):
        """Clean up when node is destroyed."""
        if SCSERVO_AVAILABLE and hasattr(self, 'port_handler'):
            self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FeetechDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
