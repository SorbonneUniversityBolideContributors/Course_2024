#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__date__ = "2023-01"


#%% IMPORTS
from vehicle import Driver
from controller import Accelerometer, Gyro, InertialUnit

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

        self.inertial_unit = InertialUnit("TT02_inertial_unit")
        self.inertial_unit.enable(self.sensorTimeStep)

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

        # inertial unit
        roll, pitch, yaw = self.inertial_unit.getRollPitchYaw()
        self.imuFrame.orientation.x = roll
        self.imuFrame.orientation.y = pitch
        self.imuFrame.orientation.z = yaw

        # accelerometer
        linear_acceleration = self.accelerometer.getValues()
        self.imuFrame.linear_acceleration.x = linear_acceleration[0]
        self.imuFrame.linear_acceleration.y = linear_acceleration[1]
        self.imuFrame.linear_acceleration.z = linear_acceleration[2]

        # gyro
        angular_velocity = self.gyro.getValues()
        self.imuFrame.angular_velocity.x = angular_velocity[0]
        self.imuFrame.angular_velocity.y = angular_velocity[1]
        self.imuFrame.angular_velocity.z = angular_velocity[2]

        self.imuFrame.header.stamp = rospy.Time.now()
        self.pub_imu.publish(self.imuFrame)
