#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__date__ = "2023-01"


#%% IMPORTS
from simu_controller import SimuController
from perception_bolide.msg import ForkSpeed
import rospy


#%% CLASS
class SimuForkPublisher():

    def __init__(self, simuController:SimuController, bot_name:str="simu_bot") -> None:
        
        self.simu_controller = simuController

        # Publisher
        topic_name = "/" + bot_name + "/raw_fork_data"
        rospy.loginfo("Initializing fork sensor publisher on topic: " + topic_name)
        self.pub_fork_speed = rospy.Publisher(topic_name, ForkSpeed, queue_size=10)

        self.basicTimeStep = int(self.simu_controller.driver.getBasicTimeStep())
        self.sensorTimeStep = 4 * self.basicTimeStep

        # sensor initialization
        self.sensor_init()

    def sensor_init(self):
        """Initialize the fork settings and the ForkSpeed message"""
        self.forkSpeed = ForkSpeed()
        self.forkSpeed.header.frame_id = "fork_frame"
        self.forkSpeed.header.stamp = rospy.Time.now()
        self.forkSpeed.speed = 0.

    def publish_fork_data(self, *args):
        """Publishes the fork data in the publisher topic"""
        
        # Update the ForkSpeed message
        self.forkSpeed.header.stamp = rospy.Time.now()
        self.forkSpeed.speed = self.simu_controller.speed
        # the fork sensor can't be implemented in the simulation so we just set it to the speed of the vehicle

        # Publish the MultiRange message
        self.pub_fork_speed.publish(self.forkSpeed)
