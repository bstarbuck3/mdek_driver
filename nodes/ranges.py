#! /usr/bin/env python

import rospy
import serial, time, os
from serial import SerialException
import numpy as np
import time
from mdek_driver.msg import UWB

class Ranges:
	def __init__(self):
		rospy.init_node('mdek_driver')
		self.rangePub = rospy.Publisher("/Crawler/ranges", UWB, queue_size=10)
		# initialize serial port
		dwPort = rospy.get_param('~port','/dev/ttyACM0')
		dwRate = rospy.get_param('~baud',115200)
		self.ser = serial.Serial(port = dwPort,timeout = 10,baudrate = dwRate)
		# ranges :         rt1,    rt2,    rt3,    rt4,    ra12,   ra13,   ra23,   ra24,   ra34
		self.Z =  np.mat([[15.0], [15.0], [15.0], [15.0], [15.0], [15.0], [15.0], [15.0], [15.0]])

	def run(self):
		# connect to robot serial port tag
		print("serial")
		try:
			self.ser.close()
			self.ser.open()
			time.sleep(1)
			# configure as tag and publish ranges			
			self.ser.write("nmt\r")
			time.sleep(1)
			self.ser.write("\r")
			self.ser.write("\r")
			time.sleep(1)
			self.ser.write("les\r")
			time.sleep(1)
			msg = UWB()
			# collect data and publish				
			while not rospy.is_shutdown():
				raw_data = self.ser.readline()
				data = raw_data.split()
				for i in range(0,len(data)):					
					if data[i][0:4] =='821D':
						self.Z[0] = data[i][21:]
					if data[i][0:4] =='1D08':
						self.Z[1] = data[i][21:]
					if data[i][0:4] =='1A3B':
						self.Z[2] = data[i][21:]
					if data[i][0:4] =='053B':
						self.Z[3] = data[i][21:]
				print(self.Z.T)
				msg.rt1 = self.Z[0]
				msg.rt2 = self.Z[1]
				msg.rt3 = self.Z[2]
				msg.rt4 = self.Z[3]
				msg.ra12 = self.Z[4]
				msg.ra13 = self.Z[5]
				msg.ra23 = self.Z[6]
				msg.ra24 = self.Z[7]
				msg.ra34 = self.Z[8]
				msg.header.frame_id = "UWB_frame"			
				msg.header.stamp = rospy.get_rostime()
				self.rangePub.publish(msg)
			self.ser.close()
		except SerialException:
			print("Could not connect to the serial port")	
	
if __name__ == "__main__":
    demo = Ranges()
    demo.run()
