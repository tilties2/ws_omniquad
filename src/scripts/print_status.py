#!/usr/bin/env python

import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import TiltingMcDesiredAngles


class WrenchEstimation(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback,
            qos_profile)
        
        self.thrust_sub = self.create_subscription(
            VehicleThrustSetpoint,
            '/fmu/out/vehicle_thrust_setpoint',
            self.vehicle_thrust_setpoint_callback,
            qos_profile)
        
        self.torque_sub = self.create_subscription(
            VehicleTorqueSetpoint,
            '/fmu/out/vehicle_torque_setpoint',
            self.vehicle_torque_setpoint_callback,
            qos_profile)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/out/wrench_estimated', qos_profile)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.velocity = VehicleOdometry
        self.cmd_thrust = VehicleThrustSetpoint
        self.cmd_torque = VehicleTorqueSetpoint
        self.dt = timer_period
        self.vehicle_mass = 2.5 #0.387
        self.vehicle_inertia = np.array([[0.15, 0, 0],
                                         [0, 0.15, 0],
                                         [0, 0, 0.1]])
        self.cmd_lin_acc = 1.0 / self.vehicle_mass * np.array([0.0, 0.0, -0.387])
        self.cmd_ang_acc = np.linalg.inv(self.vehicle_inertia).dot(np.array([0.0, 0.0, 0.0]))
 
    def vehicle_odometry_callback(self, msg):
        # TODO: handle NED->ENU transformation
        #print("odometry velocity: ", msg.velocity)
        #print("  - offboard status: ", VehicleOdometry)
        self.velocity = msg.velocity

    def vehicle_thrust_setpoint_callback(self, msg):
        # TODO: handle NED->ENU transformation
        #print("command thrust: ", msg.xyz[2])
        print(self.cmd_lin_acc)
        #print("  - offboard status: ", VehicleOdometry)
        self.cmd_thrust = msg.xyz

        self.cmd_lin_acc = 1.0 / self.vehicle_mass * self.cmd_thrust
        #self.cmd_ang_acc = np.linalg.inv(self.vehicle_inertia).dot(self.cmd_torque)

    def vehicle_torque_setpoint_callback(self, msg):
        # TODO: handle NED->ENU transformation
        #print("command torque: ", msg.xyz)
        #print("  - offboard status: ", VehicleOdometry)
        self.cmd_torque = msg.xyz

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        trajectory_msg = TrajectorySetpoint()

        #self.publisher_trajectory.publish(trajectory_msg)



def main(args=None):
    rclpy.init(args=args)

    wrench_estimation = WrenchEstimation()

    rclpy.spin(wrench_estimation)

    wrench_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
