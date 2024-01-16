#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from vehicle import Driver
from controller import Lidar

import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from control_bolide.msg import SpeedDirection



class YellowController():

    def __init__(self, driver:Driver) -> None:
        
        self.driver = driver

        self.pub_scan = rospy.Publisher("/yellow_bot/raw_lidar_data", LaserScan, queue_size=10)
        self.sub_param = rospy.Subscriber("/param_change_alert", Bool, self.get_max_params)

        self.sub_cmd_vel = rospy.Subscriber("cmd_vel", SpeedDirection, self.cmd_vel_callback)

        self.basicTimeStep = int(self.driver.getBasicTimeStep())
        self.sensorTimeStep = 4 * self.basicTimeStep

        # Lidar
        self.lidar = Lidar("RpLidarA2")
        self.lidar.enable(self.sensorTimeStep)
        self.lidar.enablePointCloud()

        self.lidarScan = LaserScan()
        self.lidarScan.header.stamp = rospy.Time.now()
        self.lidarScan.header.frame_id = "lidar_frame"
        self.lidarScan.angle_min = 0 # in radians
        self.lidarScan.angle_max = 2*np.pi
        self.lidarScan.angle_increment = 2*np.pi/360
        self.lidarScan.time_increment = self.lidar.getSamplingPeriod()
        self.lidarScan.scan_time = self.lidarScan.time_increment * 360
        self.lidarScan.range_min = 0.17
        self.lidarScan.range_max = 12.0

        self.lidarScan.ranges = [0.] * 360

        self.get_max_params()
        # Speed
        self.speed = 0

        # Steering
        self.steeringAngle = 0

        self.setCommand()

    def get_max_params(self, value = True) :
        self.maxSpeed = rospy.get_param('/simulation_max_speed', default = 28) / 3.6
        self.maxSteeringAngle = rospy.get_param('/simulation_max_angle', default = 25) * np.pi/180


    def command_to_units(self, speed, steeringAngle):
        speed = speed * self.maxSpeed
        steeringAngle = steeringAngle * self.maxSteeringAngle
        return speed, steeringAngle
    
    def applyCommand(self):
        self.driver.setCruisingSpeed(self.speed)
        self.driver.setSteeringAngle(self.steeringAngle)

    def setCommand(self, speed_command=0, steeringAngle_command=0):

        assert speed_command >= -1 and speed_command <= 1, "Speed command must be between -1 and 1"
        assert steeringAngle_command >= -1 and steeringAngle_command <= 1, "Steering angle command must be between -1 and 1"

        self.speed, self.steeringAngle = self.command_to_units(speed_command, steeringAngle_command)
    
    def treat_lidar_data(self, lidar_data) -> list:
        """outputs 360 values, no inf values"""
        num_degrees = 360
        num_bin = len(lidar_data)
        bin_size = num_bin / num_degrees

        # Create an array to store the resampled data
        resampled_data = np.zeros(num_degrees)

        # Resample the data
        for i in range(num_degrees):
            # Find the corresponding bin index
            bin_index = int(i / bin_size)
            
            # Calculate the average distance for the bin
            distance = lidar_data[bin_index]
            resampled_data[i] = distance if (distance not in [np.nan, np.inf]) else 0.
        
        return resampled_data


    def simulationStep(self, event):
        
        # lidar
        self.lidarScan.ranges = self.treat_lidar_data( self.lidar.getRangeImage() )
        self.lidarScan.header.stamp = rospy.Time.now()
        self.pub_scan.publish(self.lidarScan)

        self.applyCommand()


    def cmd_vel_callback(self, msg):
        self.setCommand(msg.speed, msg.direction)



print("ROS Controller initialization") 
driver = Driver()      
rospy.init_node('Yellow_controller', anonymous=True)

robot_controller = YellowController(driver)

TIME_SIMU = 1.0/12.0
rospy.Timer(rospy.Duration(TIME_SIMU), robot_controller.simulationStep)
print("Controller initialized: publishing in ROS topics:\n-raw_lidar_data")


rate = rospy.Rate(30)
while not rospy.is_shutdown():
    # Your control logic here
    if driver.step() == -1:
        break
    rate.sleep()