#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__date__ = "2023-01"


#%% IMPORTS
from vehicle import Driver
from controller import DistanceSensor

from sensor_msgs.msg import Range as SensorRange
from perception_bolide.msg import MultipleRange
import rospy


#%% CLASS
class SimuRearRangesPublisher():

    def __init__(self, driver:Driver, bot_name:str="simu_bot") -> None:
        
        self.driver = driver
        
        # Publisher
        topic_name = "/" + bot_name + "/raw_rear_range_data"
        rospy.loginfo("Initializing rear range sensors publisher on topic: " + topic_name)
        self.pub_img = rospy.Publisher(topic_name, MultipleRange, queue_size=10)

        self.basicTimeStep = int(self.driver.getBasicTimeStep())
        self.sensorTimeStep = 4 * self.basicTimeStep

        # Sensors bounds
        self.ir_min_range = 0.06
        self.ir_max_range = 0.3
        self.sonar_min_range = 0.07
        self.sonar_max_range = 0.5

        # sensors initialization
        self.sensors_init()
    

    def sensors_init(self):
        """Initialize the sensors settings and the MultiRange message"""

        self.ir_rear_left = DistanceSensor("TT02_IR_rear_left")
        self.ir_rear_left.enable(self.sensorTimeStep)

        self.ir_rear_right = DistanceSensor("TT02_IR_rear_right")
        self.ir_rear_right.enable(self.sensorTimeStep)

        self.sonar_rear = DistanceSensor("TT02_sonar_rear")
        self.sonar_rear.enable(self.sensorTimeStep)

        self.multiRangeFrame = MultipleRange()

        # Rear IR sensors ===============================================================
        for sensorFrame in [self.multiRangeFrame.IR_rear_left, self.multiRangeFrame.IR_rear_right]:
            sensorFrame = SensorRange()
            sensorFrame.header.frame_id = "rear_ir_range_frame"
            sensorFrame.radiation_type = SensorRange.INFRARED
            sensorFrame.min_range = self.ir_min_range
            sensorFrame.max_range = self.ir_max_range
        
        # Rear sonar sensor =============================================================
        self.multiRangeFrame.Sonar_rear = SensorRange()
        self.multiRangeFrame.Sonar_rear.header.frame_id = "rear_sonar_range_frame"
        self.multiRangeFrame.Sonar_rear.radiation_type = SensorRange.ULTRASOUND
        self.multiRangeFrame.Sonar_rear.min_range = self.sonar_min_range
        self.multiRangeFrame.Sonar_rear.max_range = self.sonar_max_range

    def crop_range(self, range:float, min_range:float, max_range:float) -> float:
        """Crops the range value between min_range and max_range"""
        if range < min_range:
            range = min_range
        elif range > max_range:
            range = max_range
        return range

    def publish_rear_range_data(self, *args):
        """Publishes the rear range data in the publisher topic"""
        # each time, the data is cropped between min and max range values

        # Rear IR sensors ===============================================================
        # Left
        self.multiRangeFrame.IR_rear_left.range = self.crop_range(self.ir_rear_left.getValue(), self.ir_min_range, self.ir_max_range)
        self.multiRangeFrame.IR_rear_left.header.stamp = rospy.Time.now()
        # Right
        self.multiRangeFrame.IR_rear_right.range = self.crop_range(self.ir_rear_right.getValue(), self.ir_min_range, self.ir_max_range)
        self.multiRangeFrame.IR_rear_right.header.stamp = rospy.Time.now()

        # Rear sonar sensor =============================================================
        self.multiRangeFrame.Sonar_rear.range = self.crop_range(self.sonar_rear.getValue(), self.sonar_min_range, self.sonar_max_range)
        self.multiRangeFrame.Sonar_rear.header.stamp = rospy.Time.now()

        # Publish the MultiRange message
        self.pub_img.publish(self.multiRangeFrame)
