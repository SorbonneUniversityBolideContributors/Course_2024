#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__date__ = "2023-01"


#%% IMPORTS
from vehicle import Driver

import numpy as np
import rospy
from std_msgs.msg import Bool
from control_bolide.msg import SpeedDirection


#%% CLASS
class SimuController():

    def __init__(self, driver:Driver, bot_name:str="simu_bot") -> None:
        
        self.driver = driver

        # Subscribers
        self.sub_param = rospy.Subscriber("/param_change_alert", Bool, self.get_max_params)
        self.sub_cmd_vel = rospy.Subscriber("/" + bot_name + "/cmd_vel", SpeedDirection, self.set_command)

        self.get_max_params()
        self.set_command(SpeedDirection(0., 0.))

    def get_max_params(self, value = True) :
        self.maxSpeed = rospy.get_param('/simulation_max_speed', default = 28) / 3.6
        self.maxSteeringAngle = rospy.get_param('/simulation_max_angle', default = 20) * np.pi/180

    def cmd_vel_callback(self, msg:SpeedDirection):
        """Callback for the cmd_vel topic"""
        self.set_command(msg.speed, msg.direction)

    def set_command(self, msg:SpeedDirection):
        """Set the speed and steering angle of the vehicle"""
        speed_command = msg.speed
        steeringAngle_command = msg.direction
        # make sure the commands are between -1 and 1
        assert speed_command >= -1 and speed_command <= 1, "Speed command must be between -1 and 1"
        assert steeringAngle_command >= -1 and steeringAngle_command <= 1, "Steering angle command must be between -1 and 1"
        
        # apply the commands
        self.driver.setCruisingSpeed(speed_command * self.maxSpeed)
        self.driver.setSteeringAngle(steeringAngle_command * self.maxSteeringAngle)
