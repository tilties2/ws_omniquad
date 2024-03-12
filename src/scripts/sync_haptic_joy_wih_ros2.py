#!/usr/bin/env python

import rclpy
import numpy as np
from std_msgs.msg import Int16
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
#from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint, VehicleOdometry
#from px4_msgs.msg import OffboardControlMode
from scservo_sdk import *                    # Uses SCServo SDK library
from time import sleep
import os
import signal
import sys

class HapticRC(Node):

    def __init__(self):
        super().__init__('haptic_rc')

        if os.name == 'nt':
            import msvcrt
            def getch():
                return msvcrt.getch().decode()
        else:
            import sys, tty, termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            def getch():
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch

        # Define publishers and subscribers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        #self.odom_sub = self.create_subscription(
        #    VehicleOdometry,
        #    '/fmu/out/vehicle_odometry',
        #    self.odometry_callback,
        #    qos_profile)


        self.haptic_rc_ref_pub = self.create_publisher(TwistStamped, '/haptic_rc/ref', qos_profile)

        self.haptic_rc_mode_pub = self.create_publisher(Int16, '/haptic_rc/mode', qos_profile)

        # Torque control parameters
        self.KP_TORQUE = np.array([50, 50, 50, 50])
        self.KI_TORQUE = np.array([1, 1, 1, 1])    # Integral gain
        self.KE = 0.40           # Elastic coefficient

        # Create timer for periodic execution
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def odometry_callback(self, msg):
        # TODO: take odomoetry in B the body frame
        # ie. Set velocity_frame variable to VELOCITY_FRAME_BODY_FRD = 3 # FRD body-fixed frame
        # Assuming velocity is in body frame
        self.lin_vel_B = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        print(self.lin_vel_B[0])
        self.ang_vel_B = np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]])
        #print(self.ang_vel_B)
        print(msg.velocity_frame)

    def thrust_callback(self, msg):
        self.cmd_thrust = np.array([msg.xyz[0], msg.xyz[1], msg.xyz[2]])
        #print(self.cmd_thrust)

    def torque_callback(self, msg):
        self.cmd_torque = np.array([msg.xyz[0], msg.xyz[1], msg.xyz[2]])
        #print(self.cmd_torque)


    def calculate_reference_trajectory(self):
        # Load wrench command
        

        # Publish for debug
        haptic_rc_msg = TwistStamped()
        haptic_rc_msg.header.stamp = self.get_clock().now().to_msg()
        haptic_rc_msg.twist.linear.x = 0.0
        haptic_rc_msg.twist.linear.y = 1.0
        haptic_rc_msg.twist.linear.z = 2.0
        haptic_rc_msg.twist.angular.x = 3.0
        haptic_rc_msg.twist.angular.y = 4.0
        haptic_rc_msg.twist.angular.z = 5.0
        
        self.haptic_rc_ref_pub.publish(haptic_rc_msg)

        haptic_rc_mode_msg = Int16()
        haptic_rc_mode_msg.data = 2

        self.haptic_rc_mode_pub.publish(haptic_rc_mode_msg)

    def timer_callback(self):
        self.calculate_reference_trajectory()


def main(args=None):
    rclpy.init(args=args)
    haptic_rc = HapticRC()
    rclpy.spin(haptic_rc)
    haptic_rc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
