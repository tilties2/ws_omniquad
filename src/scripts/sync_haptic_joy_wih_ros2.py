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

        # Control table address
        self.ADDR_SCS_MODE               = 33
        self.ADDR_SCS_TORQUE_ENABLE      = 40
        self.ADDR_SCS_GOAL_ACC           = 41
        self.ADDR_SCS_GOAL_POSITION      = 42
        self.ADDR_SCS_GOAL_SPEED         = 46
        self.ADDR_SCS_PRESENT_POSITION   = 56
        self.ADDR_SCS_PRESENT_TORQUE     = 60

        # Control modes
        self.MODE_POSITION               = 0
        self.MODE_WHEEL                  = 1

        # Default setting
        self.SCS_ID_2                    = 2                 # SCServo#1 ID : 2
        self.SCS_ID_3                    = 3
        self.SCS_ID_4                    = 4
        self.SCS_ID_5                    = 5
        BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
        DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        # TODO: add min and max position safety
        self.SCS_2_MIN_POS               = 2100
        self.SCS_2_MAX_POS               = 2700
        self.SCS_2_INIT_POS              = 2400
        self.SCS_2_OFFSET_ORIGIN_POS     = 2400
        self.SCS_3_MIN_POS               = 2700
        self.SCS_3_MAX_POS               = 3400
        self.SCS_3_INIT_POS              = 3000
        self.SCS_3_OFFSET_ORIGIN_POS     = 3000
        self.SCS_4_MIN_POS               = 2700
        self.SCS_4_MAX_POS               = 3600
        self.SCS_4_INIT_POS              = 3200
        self.SCS_4_OFFSET_ORIGIN_POS     = 3200
        self.SCS_5_MIN_POS               = 2300
        self.SCS_5_MAX_POS               = 2900
        self.SCS_5_INIT_POS              = 2600
        self.SCS_5_OFFSET_ORIGIN_POS     = 2600

        SCS_MOVING_ACC              = 0                 # SCServo moving acc
        protocol_end                = 0                 # SCServo bit end(STS/SMS=0, SCS=1)
        
        # Torque control parameters
        self.KP_TORQUE = np.array([50, 50, 50, 50])
        self.KI_TORQUE = np.array([1, 1, 1, 1])    # Integral gain
        self.KE = 0.40           # Elastic coefficient

        # Create timer for periodic execution
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Control loop variables
        self.torque_setpoint = np.zeros(4)    # Set your desired torque value
        self.torque_feedback = np.zeros(4)
        self.integral_term = np.zeros(4)
        self.INTEGRAL_LIMIT = 2000
        self.torque_error = np.zeros(4)
        self.speed_control_output = np.zeros(4)
        #external_wrench = Wrench()
        #external_wrench.torque.x = 0
        self.external_wrench_torque_x = 0
        #external_wrench.torque.y = 0
        #external_wrench.torque.z = 0

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Get methods and members of Protocol
        self.packetHandler = PacketHandler(protocol_end)

        # Initialize GroupSyncWrite instance
        #groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_SCS_GOAL_POSITION, 2)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_SCS_GOAL_SPEED, 2)

        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_SCS_PRESENT_POSITION, 4)
        self.groupSyncReadTorque = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_SCS_PRESENT_TORQUE, 4)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # Add parameter storage for SCServos present position value
        scs_addparam_result = self.groupSyncReadPosition.addParam(self.SCS_ID_2)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncReadPosition addparam failed" % self.SCS_ID_2)
            quit()
        scs_addparam_result = self.groupSyncReadPosition.addParam(self.SCS_ID_3)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncReadPosition addparam failed" % self.SCS_ID_3)
            quit()
        scs_addparam_result = self.groupSyncReadPosition.addParam(self.SCS_ID_4)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncReadPosition addparam failed" % self.SCS_ID_4)
            quit()
        scs_addparam_result = self.groupSyncReadPosition.addParam(self.SCS_ID_5)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncReadPosition addparam failed" % self.SCS_ID_5)
            quit()

        # Add parameter storage for SCServos present torque value
        scs_addparam_result = self.groupSyncReadTorque.addParam(self.SCS_ID_2)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncReadTorque addparam failed" % self.SCS_ID_2)
            quit()
        scs_addparam_result = self.groupSyncReadTorque.addParam(self.SCS_ID_3)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncReadTorque addparam failed" % self.SCS_ID_3)
            quit()
        scs_addparam_result = self.groupSyncReadTorque.addParam(self.SCS_ID_4)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncReadTorque addparam failed" % self.SCS_ID_4)
            quit()
        scs_addparam_result = self.groupSyncReadTorque.addParam(self.SCS_ID_5)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncReadTorque addparam failed" % self.SCS_ID_5)
            quit()

        # Initialize motors position
        # Write SCServos mode position 
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_2, self.ADDR_SCS_MODE, self.MODE_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_3, self.ADDR_SCS_MODE, self.MODE_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_4, self.ADDR_SCS_MODE, self.MODE_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_5, self.ADDR_SCS_MODE, self.MODE_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        # Initialize servos position
        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.SCS_ID_2, self.ADDR_SCS_GOAL_POSITION, self.SCS_2_INIT_POS)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.SCS_ID_3, self.ADDR_SCS_GOAL_POSITION, self.SCS_3_INIT_POS)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.SCS_ID_4, self.ADDR_SCS_GOAL_POSITION, self.SCS_4_INIT_POS)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.SCS_ID_5, self.ADDR_SCS_GOAL_POSITION, self.SCS_5_INIT_POS)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

    def signal_handler(self, sig, frame):
        # #### WRITE ####
        # Allocate goal speed value into byte array
        param_goal_speed = [SCS_LOBYTE(SCS_TOSCS(int(0), 15)), SCS_HIBYTE(SCS_TOSCS(int(0), 15))]

        # Add SCServo#2 goal speed value to the Syncwrite parameter storage
        scs_addparam_result = self.groupSyncWrite.addParam(self.SCS_ID_2, param_goal_speed)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS_ID_2)
            quit()

        # Add SCServo#3 goal speed value to the Syncwrite parameter storage
        scs_addparam_result = self.groupSyncWrite.addParam(self.SCS_ID_3, param_goal_speed)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS_ID_3)
            quit()

        # Add SCServo#4 goal speed value to the Syncwrite parameter storage
        scs_addparam_result = self.groupSyncWrite.addParam(self.SCS_ID_4, param_goal_speed)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS_ID_4)
            quit()

        # Add SCServo#5 goal speed value to the Syncwrite parameter storage
        scs_addparam_result = self.groupSyncWrite.addParam(self.SCS_ID_5, param_goal_speed)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS_ID_5)
            quit()

        # Syncwrite goal speed
        scs_comm_result = self.groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        # Clear syncread parameter storage
        self.groupSyncReadPosition.clearParam()
        self.groupSyncReadTorque.clearParam()

        # Write SCServos mode position
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_2, self.ADDR_SCS_MODE, self.MODE_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_3, self.ADDR_SCS_MODE, self.MODE_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_4, self.ADDR_SCS_MODE, self.MODE_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_5, self.ADDR_SCS_MODE, self.MODE_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        # Initialize servos position
        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.SCS_ID_2, self.ADDR_SCS_GOAL_POSITION, self.SCS_2_INIT_POS)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.SCS_ID_3, self.ADDR_SCS_GOAL_POSITION, self.SCS_3_INIT_POS)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))
        
        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.SCS_ID_4, self.ADDR_SCS_GOAL_POSITION, self.SCS_4_INIT_POS)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.SCS_ID_5, self.ADDR_SCS_GOAL_POSITION, self.SCS_5_INIT_POS)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        sleep(1)

        # SCServo#2 torque
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_2, self.ADDR_SCS_TORQUE_ENABLE, 0)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        # SCServo#3 torque
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_3, self.ADDR_SCS_TORQUE_ENABLE, 0)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        # SCServo#4 torque
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_4, self.ADDR_SCS_TORQUE_ENABLE, 0)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        # SCServo#5 torque
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.SCS_ID_5, self.ADDR_SCS_TORQUE_ENABLE, 0)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        # Close port
        self.portHandler.closePort()
        print('You pressed Ctrl+C!')
        sys.exit(0)
    
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
        signal.signal(signal.SIGINT, self.signal_handler)
        self.calculate_reference_trajectory()


def main(args=None):
    rclpy.init(args=args)
    haptic_rc = HapticRC()
    rclpy.spin(haptic_rc)
    haptic_rc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
