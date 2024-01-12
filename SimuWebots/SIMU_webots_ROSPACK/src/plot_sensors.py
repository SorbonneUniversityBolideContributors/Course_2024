#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from matplotlib.animation import FuncAnimation 
from std_msgs.msg import Float32MultiArray

class Plot : 

	def __init__(self, xlim, ylim) : 
		self.fig, self.ax = plt.subplots(figsize=(6, 6))
		self.ln, = plt.plot([], [], '.b')
		self.x_data, self.y_data = [-1, 1, 0, -1, 1, -1, 1, 0, -1, 1], [1, 1, -1, 0, 0] 
		self.angles = [45*np.pi/180, 315*np.pi/180]
		self.xlim, self.ylim = [-xlim, xlim], [-ylim, ylim]
		
	
	def initPlot(self)  :
		self.ax.set_xlim(self.xlim) ; self.ax.set_ylim(self.ylim)
		self.ax.set_xlabel("distance (m)") ; self.ax.set_ylabel("distance (m)")
		self.ax.set_title("Sensors plot")
		self.ax.grid(color="gray")
		self.ax.set_facecolor((0.0, 0.0, 0.0))
		self.fig.set_facecolor((207/255, 106/255, 4/255))
		return self.ln
		
	def callback(self, msg) : 
		scan, cpt_angles = msg.data, 0
		self.x_data, self.y_data = [-1, 1, 0, -1, 1, 0], [1, 1, -1] 
		for i in range(len(scan)) : 
			value = scan[i]
			if i == 2 : value = -1*value # Back sensor value is negative for more natural plotting
			if i <= 2 : self.y_data.append(value + self.y_data[i])
			else : 
				if i == 3 : 
					self.x_data.append(-1 + -1*value * np.cos(self.angles[cpt_angles]))
					self.y_data.append(1*value * np.sin(self.angles[cpt_angles]))
				elif i == 4 : 
					self.x_data.append(1  + value * np.cos(self.angles[cpt_angles]))
					self.y_data.append(-1 * value * np.sin(self.angles[cpt_angles]))
				cpt_angles += 1
				
		assert len(self.y_data) == len(self.x_data)
		
	def updatePlot(self, frame) :
		
		try : self.ln.set_data(self.x_data, self.y_data)
		except : print("[WRN] - No data got to be plotted")
		return self.ln 	
			
			
def listener(p) : 
	topic = rospy.get_param("sensors_datas", default="/SensorsScan")
	rospy.Subscriber(topic, Float32MultiArray, p.callback)
	plt.show(block = True)
	
		
if __name__ == '__main__' :

	rospy.init_node('plot_sensors', anonymous = True)
	
	xlim, ylim = 5, 5
	p = Plot(xlim, ylim) 
	
	ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)
	sens_x, sens_y = [-1, 1, 0, -1, 1], [1, 1, -1, 0, 0]
	p.ax.plot(sens_x, sens_y, ".r")
	p.ax.add_patch(Rectangle((-1, -1), 2, 2))
	try : listener(p)
	except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit() 
		

