#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
import math

from tier4_external_api_msgs.srv import Engage
from autoware_auto_vehicle_msgs.msg import VelocityReport

from autoware_auto_control_msgs.msg import AckermannControlCommand
from tier4_control_msgs.msg import GateMode
from autoware_auto_vehicle_msgs.msg import GearCommand

from rclpy.qos import qos_profile_services_default
from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *
import time

ROTATION = 0
BTN_NAME = ['A', 'X', 'B', 'Y']
A_BTN = 0
X_BTN = 1
B_BTN = 2
Y_BTN = 3
SL_BTN = 4
SR_BTN = 5
PLUS_BTN = 9
HOME_BTN = 12
R_BTN = 14
ZR_BTN = 15

# Press button to change speed.
SPEED_DIF = 0.1
# When you add 0.1, machine adds 0.1000000001.
TOLERANCE = 0.01

class joycon_teleop(Node):

    def __init__(self):
        super().__init__('joycon_teleop')
        self.twist_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 100)

        self.create_subscription(VelocityReport ,"/vehicle/status/velocity_status", self.GetVelocity, 1)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.target_vel = 0.0
        self.target_angle = 0.0
        self.vel_add = 0.0
        self.angle_add = 0.0
        
        self.MAX_VEL = 30
        self.MIN_VEL = 0
        
        # Initializing Default Count and Speed
        self.button_cnt = 0

        # Getvelocity and accel
        self.previous_velocity_ = 0
        self.prev_stamp_ = None
        self.current_acceleration_ = 0.01
        self.current_velocity_ = 0.0

        # Publish gate mode
        self.pub_mannual_cmd_ = self.create_publisher(AckermannControlCommand, "/external/selected/control_cmd", 1)
        self.pub_gate_mode_ = self.create_publisher(GateMode, "/control/gate_mode_cmd", 1)
        self.pub_gear_cmd_ = self.create_publisher(GearCommand, "/external/selected/gear_cmd", 1)

        #Send engage survice
        self.srv_client_engage_ = self.create_client(Engage, "/api/autoware/set/engage")


        # Set GateMode as External
        self.gate_cmd = GateMode()
        self.gate_cmd.data = GateMode.EXTERNAL
        self.pub_gate_mode_.publish(self.gate_cmd)

        # Request Engage
        req = Engage.Request()
        req.engage = True
        self.future = self.srv_client_engage_.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)

        self.ackermann = AckermannControlCommand()
        self.scale_factor = -0.25

        self.gear_cmd = GearCommand()
        
        # JoyStick Initialization
        pygame.joystick.init()
        self.joystick0 = pygame.joystick.Joystick(0)
        self.joystick0.init()
        self.joystickx = 0
        self.joysticky = 0
        pygame.init()
        print("Initlialization was succesed!!")
        

    def timer_callback(self):
        eventlist = pygame.event.get()
        # Event Processing
        for e in eventlist:
            if e.type == QUIT:
                return
            # Stick Processing
            if e.type == pygame.locals.JOYHATMOTION:
                self.button_cnt += 1
                self.joystickx, self.joysticky = self.joystick0.get_hat(0)

                if self.joystickx > 0 and self.target_vel < self.MAX_VEL:
                     print("Joystick: Forward")   
                     self.vel_add = 0.5
                elif self.joystickx < 0 and self.target_vel > 0:
                    print("Joystick: Backward")  
                    self.vel_add = -0.5
                else:
                    self.vel_add = 0.0

                if self.joysticky > 0:
                    print("Joystick: Left")  
                    self.target_angle = -45
                elif self.joysticky < 0:
                    print("Joystick: Right")  
                    self.target_angle = 45
                else:
                    self.target_angle = 0.0

            # Button Processing
            if e.type == pygame.locals.JOYBUTTONDOWN:
                self.button_cnt += 1
                # turn
                if e.button == A_BTN:
                    print("A Bttuon: Turn right")      
                    self.target_vel = 2.5
                    self.target_angle = 65
                # Accel
                elif e.button == X_BTN:
                    print("X Bttuon: Accel")               
                    self.target_vel += 2.5
                # Break
                elif e.button == B_BTN:
                    print("B Bttuon: Break")
                    self.vel_add = 0.0
                    self.target_vel = 0.0
                elif e.button == Y_BTN:
                    print("Y Bttuon: Turn left")     
                    self.target_vel = 2.5
                    self.target_angle = -65
                else:
                    print("This button has no action set!")

        self.target_vel += self.vel_add
        self.target_vel = min(self.target_vel, self.MAX_VEL)
        self.target_vel = max(self.target_vel, 0)

        #print("Set Vel {:.2f}".format(self.target_vel))
        self.SetMannualCmd(self.target_vel, self.target_angle)
        self.pub_gear_cmd_.publish(self.gear_cmd)
        self.pub_mannual_cmd_.publish(self.ackermann)

    def GetVelocity(self, msg: VelocityReport):
        self.current_velocity_ = msg.longitudinal_velocity
        #print("current_velocity: {}".format(self.current_velocity_))
        cutoff = 0.0
        dt = 0.0
        acc = 0.01
        if (self.previous_velocity_):
            cutoff = 10.0
            dt = 1.0 / 10.0
            acc = (self.current_velocity_ - self.previous_velocity_) / dt
        if (self.current_acceleration_):
            self.current_acceleration_ = acc
            
        else:
            self.current_acceleration_ = self.lowpassFilter(acc, self.current_acceleration_, cutoff, dt)

        self.previous_velocity_ = msg.longitudinal_velocity
        self.prev_stamp_ = msg.header.stamp
    
    # a_t = k * a_{t-1} + (1-k) * a_{t}
    def lowpassFilter(self, current_value: float, prev_value: float, cutoff: float, dt: float) -> float:
        
        tau = 1.0 / (2.0 * math.pi * cutoff + 1.0e-5)
        k = tau / (dt + tau)
        return k * prev_value + (1.0 - k) * current_value

    def SetMannualCmd(self, target_vel:float, target_angle :float):
        self.ackermann.stamp = self.get_clock().now().to_msg()
        self.ackermann.lateral.steering_tire_angle =  self.scale_factor * math.pi * (target_angle/ 180.0)
        self.ackermann.longitudinal.speed = target_vel / 3.6

        if (self.current_acceleration_):
            k = -0.5
            v = self.current_velocity_
            v_des = self.ackermann.longitudinal.speed
            a_des = k * (v - v_des) + self.current_acceleration_

            # clamp(a_des, -1.0, 1.0)
            low = -1.0
            high = 1.0
            if a_des < low:
                a_des = low
            elif a_des > high:
                a_des = high
            self.ackermann.longitudinal.acceleration = a_des
    
            eps = 0.001
            if (self.ackermann.longitudinal.speed > eps):
                self.gear_cmd.command = GearCommand.DRIVE
            elif (self.ackermann.longitudinal.speed < -eps and self.current_velocity_ < eps) :
                self.gear_cmd.command = GearCommand.REVERSE
                self.ackermann.longitudinal.acceleration *= -1.0
            else:
                self.gear_cmd.command = GearCommand.PARK

def main(args=None):
    print('Hi from awsim_switch_controller.')
    rclpy.init(args=args)
    
    joycon_teleop_class = joycon_teleop()
    rclpy.spin(joycon_teleop_class)

    joycon_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
