#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ackerman_controller_bot controller."""

import numpy as np 
import message_filters  

from vehicle import Driver
from controller import Lidar, Camera, DistanceSensor

class BotController : 

    def __init__(self, driver, max_speed=0.5, max_angle=0.35) :
    
        # Robot initialisation 
        self.driver  = driver 
        s_left  = self.driver.getDevice("ds_left")
        s_right = self.driver.getDevice("ds_right")
        s_max_left  = self.driver.getDevice("ds_max_left")
        s_max_right = self.driver.getDevice("ds_max_right")
        s_center_left  = self.driver.getDevice("ds_center_left")
        s_center_right = self.driver.getDevice("ds_center_right")
        s_center = self.driver.getDevice("ds_center")
        self.time_step = 64
        
        self.sensors = [s_max_left, s_left, s_center_left, s_center, s_center_right, s_right, s_max_right]
        self.angles = np.linspace(-max_angle, max_angle, len(self.sensors))
        for s in self.sensors : s.enable(self.time_step)
        # Movement parameters 
        self.max_speed, self.max_angle = max_speed, max_angle
        self.current_speed, self.current_angle = 0.0, 0.0    
    
    
    def compute_trajectory(self, datas, seuil_low) : 
        maxi = np.argmax(datas)
        if(datas[3] >= seuil_low) : new_angle = 0 
        else : new_angle = self.angles[maxi]
        driver.setCruisingSpeed(self.max_speed)
        driver.setSteeringAngle(new_angle)
    
    def apply_command(self, msg_speed, msg_angle) : 
        """ Applying speed to vehicle motors """
         
        self.current_speed = msg_speed.data * self.max_speed
        self.current_angle = msg_angle.data * self.max_angle 
        # Clipping to max value
        if self.current_speed > self.max_speed : self.current_speed = self.max_speed     
        elif self.current_speed < -1*self.max_speed : self.current_speed = -1*self.max_speed        
        if self.current_angle > self.max_angle : self.current_angle = self.max_angle
        elif self.current_angle < -1*self.max_angle : self.current_angle = -1*self.max_angle 
        # Applying speed 
        driver.setCruisingSpeed(self.current_speed)
        driver.setSteeringAngle(self.current_angle)
   
driver = Driver()      
bc = BotController(driver) 
seuil_low = 4
while driver.step() != -1: 
    sensors_scan= [s.getValue() for s in bc.sensors]
    bc.compute_trajectory(sensors_scan, seuil_low)
