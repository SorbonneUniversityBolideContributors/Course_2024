#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys 
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation 
from std_msgs.msg import Float32MultiArray

class Plot : 

	def __init__(self, xlim, ylim) : 
		
		self.fig, self.ax = plt.subplots(figsize=(6, 6))
		self.ln, = plt.plot([], [], '.b')
		self.x_data, self.y_data = [], [] 
		
		self.xlim, self.ylim = [-xlim, xlim], [-ylim, ylim]
		self.angles = None
	
	def initPlot(self)  :
		self.ax.set_xlim(self.xlim) ; self.ax.set_ylim(self.ylim)
		self.ax.set_xlabel("distance (m)") ; self.ax.set_ylabel("distance (m)")
		self.ax.invert_yaxis() ; self.ax.invert_xaxis()
		self.ax.set_title("Lidar Plot")
		self.ax.grid(color="gray")
		self.ax.set_facecolor((0.0, 0.0, 0.0))
		self.fig.set_facecolor((207/255, 106/255, 4/255))
		return self.ln
		
	def callback(self, msg) : 
		
		scan = msg.data
		if self.angles is None : self.angles = np.linspace(0, 2*np.pi, len(scan))
		
		self.x_data = [0] ; self.y_data = [0]
		for i in range(len(scan)) :
			self.x_data.append(msg.data[i] * np.cos(self.angles[i]))
			self.y_data.append(msg.data[i] * np.sin(self.angles[i]))
			
	def updatePlot(self, frame) :
		self.ln.set_data(self.y_data, self.x_data)
		return self.ln 	
			
			
def listener(p) : 
	topic = rospy.get_param("lidar_datas", default="/LidarScan")
	rospy.Subscriber(topic, Float32MultiArray, p.callback)
	plt.show(block = True)
	
		
if __name__ == '__main__' :

	rospy.init_node('plot_lidar', anonymous = True)
	
	xlim, ylim = 5, 5
	p = Plot(xlim, ylim) 
	
	ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)

	try : listener(p)
	except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit() 
		

		 
