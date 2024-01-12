#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys 
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation 
from std_msgs.msg import Int16MultiArray

class ImagePlot : 

	def __init__(self, w, h) : 
	
		self.fig, self.ax = plt.subplots(figsize=(7, 4))
		self.w, self.h = w, h
		self.image = np.zeros((self.h, self.w))
		self.ln = plt.imshow(self.image)
		
	def initPlot(self)  :
		self.ax.set_title("Camera video")
		self.fig.set_facecolor((207/255, 106/255, 4/255))
		return self.ln
		
	def callback(self, msg) : 
		scan = msg.data			
		self.image = np.array(scan).reshape((self.h, self.w, 4))
		
	def updatePlot(self, frame) :
		self.ln.set_data(self.image)
		return self.ln 	
		

def listener(p) : 
	topic = rospy.get_param("image_datas", default="/ImageScan")
	rospy.Subscriber(topic, Int16MultiArray, p.callback)
	plt.show(block = True)
	

if __name__ == '__main__' :

	rospy.init_node('plot_image', anonymous = True)
	
	w, h = 640, 480
	p = ImagePlot(w, h) 
	
	ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)

	try : listener(p)
	except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit() 
		

