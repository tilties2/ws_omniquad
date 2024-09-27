#!/usr/bin/env/python
"""
File: haptic-rc-ros2.py
Authors: Julien Mellet, and Simon Le berre
Date: 2024-07-29
Review: 2024-09-26
Description: A Python script to use the haptic rc device to send some command through ROS2.
"""

import sys
import serial
import threading
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import WrenchStamped, TwistStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TiltingMcDesiredAngles
from px4_msgs.msg import VehicleStatus

import time

class HapticRC(Node):
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200, timeout=0.01):
        super().__init__('haptic_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.data_prev = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.dead_zone = 30
        self.f_max = 350
        self.k_b = 15 # 100
        self.data_lock = threading.Lock()  # Lock for synchronizing access to shared data

        # Subscriber
        self.wrench_sub = self.create_subscription(
            WrenchStamped, '/wrench_estimation',
            self.wrench_callback, qos_profile)

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        # Publisher
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_tilting = self.create_publisher(TiltingMcDesiredAngles, '/fmu/in/tilting_mc_desired_angles', qos_profile)

        # Change publisher type to TwistStamped
        self.publisher_tilting_twist = self.create_publisher(TwistStamped, '/fmu/in/tilting_mc_desired_angles_twist', qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.trajectory_msg = TrajectorySetpoint()
        self.trajectory_msg.position[2] = -5
        self.trajectory_msg.yaw = 1.7

        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_data)

        # Connect to the serial port
        self.connect()

        # Start a separate thread for reading from the serial port
        self.reading_thread = threading.Thread(target=self.read_data)
        self.reading_thread.daemon = True
        self.reading_thread.start()

    def connect(self):
        try:
            # Open serial port with minimal buffering
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout,
                inter_byte_timeout=self.timeout
            )
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud rate.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port {self.serial_port}: {e}")
            self.ser = None

    def read_data(self):
        while rclpy.ok():  # Run this loop as long as ROS 2 is running
            if not self.ser:
                continue
            
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read_until(b'o')
                    if data and b'i' in data:
                        data_frame = data.decode('utf-8').strip()
                        parsed_data = self.classification(data_frame)
                        if parsed_data:
                            # Lock the data before updating it
                            with self.data_lock:
                                self.data = parsed_data
            except serial.SerialTimeoutException:
                self.get_logger().warning("Serial read timeout occurred.")
            except Exception as e:
                self.get_logger().error(f"Error reading from serial port: {e}")

    def write(self, x):
        if self.ser:
            try:
                self.ser.write(bytes(x, 'utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")

    def extraction_value(self, chain, start, end):
        start_index = chain.find(start)
        end_index = chain.find(end)
        if start_index != -1 and end_index != -1:
            return chain[start_index + len(start):end_index].strip()
        return None

    def classification(self, data_frame):
        if data_frame.startswith('ia'):
            try:
                # Extract values and ensure they are not None
                data_a = self.extraction_value(data_frame, 'a', 'b')
                data_b = self.extraction_value(data_frame, 'b', 'c')
                data_c = self.extraction_value(data_frame, 'c', 'd')
                data_d = self.extraction_value(data_frame, 'd', 'p')
                pitch = self.extraction_value(data_frame, 'p', 'r')
                roll = self.extraction_value(data_frame, 'r', 'o')
                
                # Convert values to float and handle None cases
                data_a = float(data_a) if data_a is not None else self.data_prev[0]
                data_b = float(data_b) if data_b is not None else self.data_prev[1]
                data_c = float(data_c) if data_c is not None else self.data_prev[2]
                data_d = float(data_d) if data_d is not None else self.data_prev[3]
                pitch = float(pitch) if pitch is not None else self.data_prev[4]
                roll = float(roll) if roll is not None else self.data_prev[5]
                
                self.data_prev[0] = data_a
                self.data_prev[1] = data_b
                self.data_prev[2] = data_c
                self.data_prev[3] = data_d
                self.data_prev[4] = pitch
                self.data_prev[5] = roll

                return [data_a, data_b, data_c, data_d, pitch, roll]
                
            except ValueError as e:
                self.get_logger().error(f"Error parsing data frame: {e}")
        else:
            pass
            #self.get_logger().warning("Invalid data frame received")
        return None

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
    
    def publish_data(self):
        with self.data_lock:  # Lock the data before accessing it
            if self.data:
                # Publish offboard control modes
                offboard_msg = OffboardControlMode()
                offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                offboard_msg.position = True
                offboard_msg.velocity = False
                offboard_msg.acceleration = False
                self.publisher_offboard_mode.publish(offboard_msg)
                
                trajectory_msg = TrajectorySetpoint()
                self.trajectory_msg.position[0] +=  0.6 * self.remap(self.data[3] - 1000, [-300, 300]) 
                self.trajectory_msg.position[1] +=  0.6 * self.remap(self.data[2] - 1000, [-300, 300]) 
                self.trajectory_msg.position[2] +=  0.4 * self.remap(self.data[0] - 1000, [-300, 300])
                self.trajectory_msg.yaw         -=  0.5 * self.remap(self.data[1] - 1000, [-300, 300]) 

                self.trajectory_pub.publish(self.trajectory_msg)
                
                tilting_msg = TiltingMcDesiredAngles()
                tilting_msg.roll_body  = -self.remap_ang(self.data[4] - 1000, [-500, 500])
                tilting_msg.pitch_body =  self.remap_ang(self.data[5] - 1000, [-500, 500])
                self.publisher_tilting.publish(tilting_msg)

                # Create TwistStamped message
                twist_stamped_msg = TwistStamped()
                twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
                twist_stamped_msg.header.frame_id = 'base_link'
                
                # Explicitly cast values to float
                twist_stamped_msg.twist.linear.x  = float(self.trajectory_msg.position[0])
                twist_stamped_msg.twist.linear.y  = float(self.trajectory_msg.position[1])
                twist_stamped_msg.twist.linear.z  = float(self.trajectory_msg.position[2])
                twist_stamped_msg.twist.angular.x = float(tilting_msg.roll_body)
                twist_stamped_msg.twist.angular.y = float(tilting_msg.pitch_body)
                twist_stamped_msg.twist.angular.z = float(self.trajectory_msg.yaw)
                
                self.publisher_tilting_twist.publish(twist_stamped_msg)
                self.get_logger().info(f"Published twist: {twist_stamped_msg.twist.linear.x}, {twist_stamped_msg.twist.linear.y}, {twist_stamped_msg.twist.linear.z}, {twist_stamped_msg.twist.angular.x}, {twist_stamped_msg.twist.angular.y}, {twist_stamped_msg.twist.angular.z}")


    def remap(self, value, input_range):
        min_in, max_in = input_range
        min_out, max_out = -1, 1
        
        # Clamp the input value to be within the input range
        value = max(min(value, max_in), min_in)
        
        # Remap the value to the output range
        remapped_value = float((value - min_in) * (max_out - min_out) / (max_in - min_in) + min_out)

        if -0.2 < remapped_value < 0.2:
            return 0.0
        
        return remapped_value * 0.03

    def remap_ang(self, value, input_range):
        min_in, max_in = input_range
        min_out, max_out = -0.4, 0.4
        
        # Clamp the input value to be within the input range
        value = max(min(value, max_in), min_in)
        
        # Remap the value to the output range
        remapped_value = float((value - min_in) * (max_out - min_out) / (max_in - min_in) + min_out)

        if -0.01 < remapped_value < 0.01:
            return 0.0
        
        return remapped_value * 0.7854

    def wrench_callback(self, msg):
        # Send 
        # ros2 topic pub /wrench_estimation geometry_msgs/msg/WrenchStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, wrench: {force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}" --once
        # Define saturation limits
        force_limit = self.f_max 

        # Extract force and torque from the Wrench message
        data_joy_a = -msg.wrench.force.z
        data_joy_b = -msg.wrench.torque.z
        data_joy_c =  msg.wrench.force.x
        data_joy_d =  msg.wrench.force.y

        data_joy_a *= self.k_b
        data_joy_b *= self.k_b
        data_joy_c *= self.k_b
        data_joy_d *= self.k_b

        # Apply saturation limits
        data_joy_a = max(-force_limit, min(force_limit, data_joy_a)) + 1000
        data_joy_b = max(-force_limit, min(force_limit, data_joy_b)) + 1000
        data_joy_c = max(-force_limit, min(force_limit, data_joy_c)) + 1000
        data_joy_d = max(-force_limit, min(force_limit, data_joy_d)) + 1000

        # Construct the data string to send to the joystick
        # Example: ia1250b950c1050d900o
        data_string = f"ia{int(data_joy_a)}b{int(data_joy_b)}c{int(data_joy_c)}d{int(data_joy_d)}o"

        print(data_string)

        self.write(data_string)

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    haptic_rc = HapticRC()
    rclpy.spin(haptic_rc)
    haptic_rc.disconnect()
    haptic_rc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
