#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__date__ = "2023-01"


#%% IMPORTS
from vehicle import Driver
from controller import Accelerometer, Gyro

from sensor_msgs.msg import Imu as SensorImu
import rospy


#%% CLASS
class SimuImuPublisher():

    def __init__(self, driver:Driver, bot_name:str="simu_bot") -> None:
        
        self.driver = driver
        
        # Publisher
        topic_name = "/" + bot_name + "/raw_imu_data"
        rospy.loginfo("Initializing imu publisher on topic: " + topic_name)
        self.pub_imu = rospy.Publisher(topic_name, SensorImu, queue_size=10)

        self.basicTimeStep = int(self.driver.getBasicTimeStep())
        self.sensorTimeStep = 4 * self.basicTimeStep

        # Lidar initialization
        self.imu_init()
    

    def imu_init(self):
        """Initialize the imu settings and the imuFrame message"""

        self.accelerometer = Accelerometer("TT02_accelerometer")
        self.accelerometer.enable(self.sensorTimeStep)

        self.gyro = Gyro("TT02_gyro")
        self.gyro.enable(self.sensorTimeStep)

        self.imuFrame = SensorImu()
        self.imuFrame.header.stamp = rospy.Time.now()
        self.imuFrame.header.frame_id = "imu_frame"

        self.imuFrame.orientation.x = 0
        self.imuFrame.orientation.y = 0
        self.imuFrame.orientation.z = 0

        self.imuFrame.angular_velocity.x = 0
        self.imuFrame.angular_velocity.y = 0
        self.imuFrame.angular_velocity.z = 0

        self.imuFrame.linear_acceleration.x = 0
        self.imuFrame.linear_acceleration.y = 0
        self.imuFrame.linear_acceleration.z = 0


    def publish_imu_data(self, *args):
        """Publishes the accelerometer data in the publisher topic"""

        # accelerometer
        self.imuFrame.linear_acceleration.x = self.accelerometer.getValues()[0]
        self.imuFrame.linear_acceleration.y = self.accelerometer.getValues()[1]
        self.imuFrame.linear_acceleration.z = self.accelerometer.getValues()[2]

        # gyro
        self.imuFrame.angular_velocity.x = self.gyro.getValues()[0]
        self.imuFrame.angular_velocity.y = self.gyro.getValues()[1]
        self.imuFrame.angular_velocity.z = self.gyro.getValues()[2]

        self.imuFrame.header.stamp = rospy.Time.now()
        self.pub_imu.publish(self.imuFrame)
