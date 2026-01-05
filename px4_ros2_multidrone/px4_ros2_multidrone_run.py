#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Create subscriptions for drone 1
        self.status_sub = self.create_subscription(VehicleStatus,'/px4_1/fmu/out/vehicle_status',self.vehicle_status_callback,qos_profile)

        #Create subscriptions for drone 2
        self.status_sub_two = self.create_subscription(VehicleStatus,'/px4_2/fmu/out/vehicle_status',self.vehicle_status_callback_two,qos_profile)

        #Create publishers for drone 1
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/px4_1/fmu/in/vehicle_command", 10)

        #Create publishers for drone 2
        self.publisher_offboard_mode_two = self.create_publisher(OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory_two = self.create_publisher(TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_two = self.create_publisher(VehicleCommand, "/px4_2/fmu/in/vehicle_command", 10)
        
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback_one) # for drone one
        self.arm_timer_two = self.create_timer(arm_timer_period, self.arm_timer_callback_two)   # for drone two

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.publish_position_drone_one)  # for drone one
        self.timer_two = self.create_timer(timer_period, self.publish_position_drone_two)  # for drone two

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = True
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state

        self.nav_state_two = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state_two = VehicleStatus.ARMING_STATE_ARMED
        self.velocity_two = Vector3()
        self.offboardMode_two = False
        self.flightCheck_two = False
        self.myCnt_two = 0
        self.arm_message_two = True
        self.failsafe_two = False
        self.current_state_two = "IDLE"
        self.last_state_two = self.current_state_two


    def arm_timer_callback_one(self):

        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    #self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    #self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    #self.get_logger().info(f"Arming, Takeoff")
                self.arm_one() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    #self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    #self.get_logger().info(f"Takeoff, Loiter")
                self.arm_one() #send arm command
                self.take_off_one() #send takeoff command

            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    #self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    #self.get_logger().info(f"Loiter, Offboard")
                self.arm_one()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    #self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard_one()

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info("for drone one ")
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def arm_timer_callback_two(self):

        match self.current_state_two:
            case "IDLE":
                if(self.flightCheck_two and self.arm_message_two == True):
                    self.current_state_two = "ARMING"
                    #self.get_logger().info(f"Arming drone two")

            case "ARMING":
                if(not(self.flightCheck_two)):
                    self.current_state_two = "IDLE"
                    #self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state_two == VehicleStatus.ARMING_STATE_ARMED and self.myCnt_two > 10):
                    self.current_state_two = "TAKEOFF"
                    #self.get_logger().info(f"Arming, Takeoff")
                self.arm_two() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck_two)):
                    self.current_state_two = "IDLE"
                    #self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state_two == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state_two = "LOITER"
                    #self.get_logger().info(f"Takeoff, Loiter")
                self.arm_two() #send arm command
                self.take_off_two() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck_two)):
                    self.current_state_two = "IDLE"
                    #self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state_two == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state_two = "OFFBOARD"
                    #self.get_logger().info(f"Loiter, Offboard")
                self.arm_two()

            case "OFFBOARD":
                if(not(self.flightCheck_two) or self.arm_state_two != VehicleStatus.ARMING_STATE_ARMED or self.failsafe_two == True):
                    self.current_state_two = "IDLE"
                    #self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard_two()

        # if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
        #     self.arm_message = False

        if (self.last_state_two != self.current_state_two):
            self.last_state_two = self.current_state_two
            self.get_logger().info("for drone two below ")
            self.get_logger().info(self.current_state_two)

        self.myCnt_two += 1

    def state_offboard_one(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True   

    def state_offboard_two(self):
        self.myCnt_two = 0
        self.publish_vehicle_command_two(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode_two = True  

    # Arms the vehicle
    def arm_one(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        #self.get_logger().info("Arm command send")

    def arm_two(self):
        self.publish_vehicle_command_two(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        #self.get_logger().info("Arm command send")

    # Takes off the vehicle to a user specified altitude (meters)
    def take_off_one(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        #self.get_logger().info("Takeoff command send")

    def take_off_two(self):
        self.publish_vehicle_command_two(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        #self.get_logger().info("Takeoff command send")

    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 2  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)
    
    def publish_vehicle_command_two(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 3  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_two.publish(msg)

 
    def vehicle_status_callback(self, msg):

        # if (msg.nav_state != self.nav_state):
        #     self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        # if (msg.arming_state != self.arm_state):
        #     self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        # if (msg.failsafe != self.failsafe):
        #     self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        # if (msg.pre_flight_checks_pass != self.flightCheck):
        #     self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def vehicle_status_callback_two(self, msg):

        # if (msg.nav_state != self.nav_state_two):
        #     self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        # if (msg.arming_state != self.arm_state_two):
        #     self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        # if (msg.failsafe != self.failsafe_two):
        #     self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        # if (msg.pre_flight_checks_pass != self.flightCheck_two):
        #     self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state_two = msg.nav_state
        self.arm_state_two = msg.arming_state
        self.failsafe_two = msg.failsafe
        self.flightCheck_two = msg.pre_flight_checks_pass

        
    #publishes offboard control modes and velocity as trajectory setpoints
    def publish_position_drone_one(self):
        if(self.offboardMode == True):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)            


            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = float('nan')
            trajectory_msg.velocity[1] = float('nan')
            trajectory_msg.velocity[2] = float('nan')
            trajectory_msg.position[0] = 4
            trajectory_msg.position[1] = 4
            trajectory_msg.position[2] =-10
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = float('nan')

            self.publisher_trajectory.publish(trajectory_msg)

    def publish_position_drone_two(self):
        if(self.offboardMode_two == True):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            self.publisher_offboard_mode_two.publish(offboard_msg)            


            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = float('nan')
            trajectory_msg.velocity[1] = float('nan')
            trajectory_msg.velocity[2] = float('nan')
            trajectory_msg.position[0] = 8
            trajectory_msg.position[1] = 8
            trajectory_msg.position[2] =-10
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = float('nan')

            self.publisher_trajectory_two.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()