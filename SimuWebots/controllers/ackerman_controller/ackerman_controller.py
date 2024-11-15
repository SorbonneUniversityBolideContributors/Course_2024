#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__version__ = "2.0.0"

"""ackerman_controller controller."""

import rospy 
import numpy as np 
from threading import Lock
from vehicle import Driver
from controller import Lidar, Camera, DistanceSensor
from control_bolide.msg import SpeedDirection
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from sensor_msgs.msg import LaserScan

class RosController : 
    
    def __init__(self, driver, max_speed=2.0, max_angle=0.35) :
        
        # Robot initialisation 
        self.driver  = driver 
        self.lidar   = self.driver.getDevice("RPlidar A2")
        self.camera  = self.driver.getDevice("camera")
        s_left  = self.driver.getDevice("ds_left")
        s_right = self.driver.getDevice("ds_right")
        s_back_left  = self.driver.getDevice("ds_back_left")
        s_back_right = self.driver.getDevice("ds_back_right")

        self.sensors = [s_left, s_right, s_back_left, s_back_right]
        self.time_step = 64
        self.camera.enable(self.time_step)
        for s in self.sensors : s.enable(self.time_step)
        
        # Movement parameters 
        self.max_speed, self.max_angle = max_speed, max_angle
        self.current_speed, self.current_angle = 0.0, 0.0

        # Add a lock for thread safety
        self.lock = Lock()
        
        # Sensor publishers 
        self.lidar_scan, self.image_scan, self.sensors_scan = LaserScan(), Int16MultiArray(), Float32MultiArray() 
        self.pub_scan = rospy.Publisher("/raw_lidar_data", LaserScan, queue_size=10)
        self.pub_img  = rospy.Publisher("/raw_camera_data", Int16MultiArray, queue_size=10)
        self.pub_sens = rospy.Publisher("/SensorsScan", Float32MultiArray, queue_size=10)

        self.lidar_scan.header.frame_id = "lidar_frame"
        self.lidar_scan.angle_min = 0 # in radians
        self.lidar_scan.angle_max = 2*np.pi # in radians
        self.lidar_scan.angle_increment = 2*np.pi/360 # in radians (should be 1 degree)
        self.lidar_scan.time_increment = self.lidar.getSamplingPeriod() # in seconds
        self.lidar_scan.scan_time = self.lidar_scan.time_increment * 360 # in seconds
        self.lidar_scan.range_min = 0.0
        self.lidar_scan.range_max = 10000.0
    

    def apply_command(self, msg_cmd_vel) :
        """ Applying speed to vehicle motors """
        
        # Getting speed and angle from message
        msg_speed, msg_angle = msg_cmd_vel.speed, msg_cmd_vel.direction

        self.current_speed = msg_speed * self.max_speed
        self.current_angle = msg_angle * self.max_angle 
        
        # Clipping to max value
        if self.current_speed > self.max_speed : self.current_speed = self.max_speed     
        elif self.current_speed < -1*self.max_speed : self.current_speed = -1*self.max_speed        
        if self.current_angle > self.max_angle : self.current_angle = self.max_angle
        elif self.current_angle < -1*self.max_angle : self.current_angle = -1*self.max_angle 

        # Applying speed 
        with self.lock :
            driver.setCruisingSpeed(self.current_speed)
            driver.setSteeringAngle(self.current_angle)
   

    def rescale_to_360(self, data) :
        """ Rescaling data from 800 to 360 points """
        # the lidar has 800 points, we want to rescale it to 360 points
        """ num_degrees = 360
        num_bin = 800
        bin_size = num_bin / num_degrees

        # Create an array to store the resampled data
        resampled_data = np.zeros(num_degrees)

        # Resample the data
        for i in range(num_degrees):
            # Find the corresponding bin index
            bin_index = int(i / bin_size)
            
            # Calculate the average distance for the bin
            distance = data[bin_index]
            resampled_data[i] = distance if (distance not in [np.nan, np.inf]) else 0. """

        return data[:360]

    def publish_scan(self, event) :
        try : 
            self.lidar.enable(self.time_step)


            self.lidar_scan.header.stamp = rospy.Time.now()

            self.lidar_scan.ranges = self.lidar.getRangeImage()[:360]
            self.pub_scan.publish(self.lidar_scan)

            self.lidar.disable()
        except Exception as e :
            print("Error while publishing scan", e)
        
  
    def publish_image(self, event) : 
        self.image_scan.data = self.camera.getImage()  
        self.pub_img.publish(self.image_scan)
      
    def publish_sensors(self, event) : 
        self.sensors_scan.data = [s.getValue() for s in self.sensors]
        self.pub_sens.publish(self.sensors_scan)
   
   
print("ROS Controller initialization") 
driver = Driver()      
rospy.init_node('ackerman_controller', anonymous=True)

TIME_SCAN = 2.0/10.0
TIME_IMG  = 5.0/10.0
TIME_SENS = 1.0/10.0

rc = RosController(driver) 
rospy.Timer(rospy.Duration(TIME_SCAN), rc.publish_scan)
rospy.Timer(rospy.Duration(TIME_IMG),  rc.publish_image)
rospy.Timer(rospy.Duration(TIME_SENS), rc.publish_sensors)
print("Controller initialized: publishing in ROS topics:\n-/LidarScan: Lidar data\n-/ImageScan: Camera data\n-/SensorsScan: TOFs data")

# Create subscriber outside the loop
cmd_vel_sub = rospy.Subscriber("/cmd_vel", SpeedDirection, rc.apply_command, queue_size=1)


rate = rospy.Rate(30)
while not rospy.is_shutdown():
    # Your control logic here
    if driver.step() == -1:
        break
    rate.sleep()