#! /usr/bin/env python

# Bryan Starbuck - Georgia Institute of Technology - bstarbuck3@gatech.edu

import rospy
import serial, time, os
from serial import SerialException
from decimal import Decimal
import numpy as np
import time
from sensor_msgs.msg import Imu

class Ranges:
	def __init__(self):
		rospy.init_node('mdek_driver')
		self.accelPub = rospy.Publisher("/Crawler/accels", Imu, queue_size=10)
		# initialize serial port
		dwPort = rospy.get_param('~port','/dev/ttyACM0')
		dwRate = rospy.get_param('~baud',115200)
		self.ser = serial.Serial(port = dwPort,timeout = 10,baudrate = dwRate)
		self.accel = np.mat([[0.0], [0.0], [0.0]])
		self.tag_started = 0
		
	def run(self):
		# connect tag to serial port
		try:
			self.ser.close()
			self.ser.open()
			time.sleep(1)
			# configure mdek as a tag			
			self.ser.write("nmt\r")
			time.sleep(1)
			self.ser.write("\r")
			self.ser.write("\r")
			time.sleep(1)
			self.ser.write("av\r")
			time.sleep(1)
			self.ser.write("\r")
			imumsg = UWB()
			print("serial connection established")
			# read and publish data			
			while not rospy.is_shutdown():
				raw_data = self.ser.readline()
				data = raw_data.split()
				# converts acceleration values to float
				self.accel[0] = float(str(data[3]).replace(',',''))
				self.accel[1] = float(str(data[6]).replace(',',''))
				self.accel[2] = float(str(data[9]).replace(',',''))
				print(self.accel.T)
				# publish ros message
				imumsg.linear_acceleration.x = self.accel[0]
				imumsg.linear_acceleration.y = self.accel[1]
				imumsg.linear_acceleration.z = self.accel[2]
				imumsg.header.frame_id = "IMU_frame"			
				imumsg.header.stamp = rospy.get_rostime()
				self.accelPub.publish(imumsg)
				self.ser.write("les\r")
				time.sleep(1)
			self.ser.close()
		except SerialException:
			print("Could not connect to the serial port")		
	
if __name__ == "__main__":
    demo = Ranges()
    demo.run()
