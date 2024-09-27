#!/usr/bin/env python

import rclpy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint, VehicleOdometry
from px4_msgs.msg import OffboardControlMode

class WrenchEstimation(Node):

    def __init__(self):
        super().__init__('wrench_estimation')

        # Define publishers and subscribers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile)

        self.thrust_sub = self.create_subscription(
            VehicleThrustSetpoint,
            '/fmu/out/vehicle_thrust_setpoint',
            self.thrust_callback,
            qos_profile)
        
        self.torque_sub = self.create_subscription(
            VehicleTorqueSetpoint,
            '/fmu/out/vehicle_torque_setpoint',
            self.torque_callback,
            qos_profile)

        self.wrench_estimation_pub = self.create_publisher(WrenchStamped, '/wrench_estimation', qos_profile)

        #self.velocity = VehicleOdometry
        #self.angular_velocity = VehicleOdometry
        #self.cmd_thrust = VehicleThrustSetpoint
        #self.cmd_torque = VehicleTorqueSetpoint
        self.vehicle_mass = 2.5
        self.vehicle_inertia = np.array([[0.15, 0, 0],
                                         [0, 0.15, 0],
                                         [0, 0, 0.1]])
        self.ext_lin_acc = np.zeros(3)
        self.ext_ang_acc = np.zeros(3)
        self.int_vec_lin = np.zeros(3)
        self.int_vec_ang = np.zeros(3)
        self.gravity = np.array([0, 0, 9.81])  # Assuming gravity is along the z-axis

        # Create timer for periodic execution
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.k_I_lin = np.array([0.9, 0.9, 0.9])
        self.k_I_ang = np.array([0.9, 0.9, 0.9])

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


    def calculate_wrench_estimate(self):
        # Load wrench command
        cmd_lin_acc = 1.0 / self.vehicle_mass * self.cmd_thrust
        cmd_ang_acc = np.linalg.inv(self.vehicle_inertia).dot(self.cmd_torque)

        # TODO: rotate gravity vector with drone attitude
        self.grav_B = self.gravity

        # Compute nonlinear terms
        n_ang = np.linalg.inv(self.vehicle_inertia).dot(-np.cross(self.vehicle_inertia.dot(self.ang_vel_B), self.ang_vel_B))
        n_lin = np.cross(self.ang_vel_B, self.lin_vel_B) + self.grav_B

        # Compute the integral term using forward Euler
        self.int_vec_lin += (cmd_lin_acc + self.ext_lin_acc - n_lin) * self.timer_period
        self.int_vec_ang += (cmd_ang_acc + self.ext_ang_acc - n_ang) * self.timer_period

        # Update estimate
        self.ext_lin_acc = (self.lin_vel_B - self.int_vec_lin) * self.k_I_lin
        self.ext_ang_acc = (self.ang_vel_B - self.int_vec_ang) * self.k_I_ang

        # Compute thrust and torque
        thrust = self.vehicle_mass * self.ext_lin_acc
        torque = np.matmul(self.vehicle_inertia, self.ext_ang_acc)

        # Publish for debug
        wrench_estimation_msg = WrenchStamped()
        wrench_estimation_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_estimation_msg.wrench.force.x = thrust[0]
        wrench_estimation_msg.wrench.force.y = thrust[1]
        wrench_estimation_msg.wrench.force.z = thrust[2] - 24.9
        wrench_estimation_msg.wrench.torque.x = torque[0]
        wrench_estimation_msg.wrench.torque.y = torque[1]
        wrench_estimation_msg.wrench.torque.z = torque[2]
        self.wrench_estimation_pub.publish(wrench_estimation_msg)

    def timer_callback(self):
        self.calculate_wrench_estimate()


def main(args=None):
    rclpy.init(args=args)
    wrench_estimation = WrenchEstimation()
    rclpy.spin(wrench_estimation)
    wrench_estimation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
