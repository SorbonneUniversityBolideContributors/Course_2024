#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__date__ = "2023-01"



#%% WARNING

"""
TO CHANGE THE CAMERA RESOLUTION, YOU NEED TO CHANGE THE RESOLUTION IN THE .proto FILE AND HERE
"""

CAMERA_WIDTH = 160
CAMERA_HEIGHT = 128



#%% IMPORTS
from vehicle import Driver
from controller import Camera

from sensor_msgs.msg import Image as SensorImage
import rospy


#%% CLASS
class SimuCameraPublisher():

    def __init__(self, driver:Driver, bot_name:str="simu_bot") -> None:
        
        self.driver = driver
        
        # Publisher
        topic_name = "/" + bot_name + "/raw_image_data"
        rospy.loginfo("Initializing camera publisher on topic: " + topic_name)
        self.pub_img = rospy.Publisher(topic_name, SensorImage, queue_size=10)

        self.basicTimeStep = int(self.driver.getBasicTimeStep())
        self.sensorTimeStep = 4 * self.basicTimeStep

        # Lidar initialization
        self.camera_init()
    

    def camera_init(self):
        """Initialize the camera settings and the lidarScan message"""

        self.camera = Camera("TT02_camera")
        self.camera.enable(self.sensorTimeStep)

        self.imageFrame = SensorImage()
        self.imageFrame.header.stamp = rospy.Time.now()
        self.imageFrame.header.frame_id = "camera_frame"
        self.imageFrame.encoding = "rgb8"
        self.imageFrame.is_bigendian = False
        self.imageFrame.height, self.imageFrame.width = CAMERA_HEIGHT, CAMERA_WIDTH
        self.imageFrame.step = 3 * CAMERA_WIDTH # This is the full row length in bytes. It's 3 times the width because each pixel has three color channels (RGB).
        

    def publish_camera_data(self, *args):
        """Publishes the camera data in the publisher topic"""
        # camera
        self.imageFrame.data = self.camera.getImage()
        self.imageFrame.header.stamp = rospy.Time.now()
        self.pub_img.publish(self.imageFrame)
