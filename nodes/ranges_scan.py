#! /usr/bin/env python

# Bryan Starbuck - Georgia Institute of Technology - bstarbuck3@gatech.edu

import rospy
import serial, time, os
from serial import SerialException
from decimal import Decimal
import numpy as np
import os
import sys
import pygatt
import time
import logging
import binascii
from binascii import hexlify
import struct
import subprocess
from mdek_driver.msg import UWB

class Ranges:
	def __init__(self):
		rospy.init_node('mdek_driver')
		self.rangePub = rospy.Publisher("/Crawler/ranges", UWB, queue_size=10)
		# initialize serial port
		dwPort = rospy.get_param('~port','/dev/ttyACM0')
		dwRate = rospy.get_param('~baud',115200)
		self.ser = serial.Serial(port = dwPort,timeout = 10,baudrate = dwRate)
		# initialize bluetooth adapter - sudo permission is required for this code
		subprocess.call('sudo hciconfig hci0 up',shell=True)
		self.adapter = pygatt.GATTToolBackend()
		# ranges :         rt1,    rt2,    rt3,    rt4,    ra12,   ra13,   ra23,   ra24,   ra34
		self.Z =  np.mat([[15.0], [15.0], [15.0], [15.0], [15.0], [15.0], [15.0], [15.0], [15.0]])
		self.range_matrix = np.mat([[0.0], [0.0], [0.0]])
		self.range_init = 0
		self.idb = {}
		self.idr = {}
		
	def scan(self):
		# scan for all blueetooth devices		
		all_dev = self.adapter.scan(run_as_root=True)
		for dev in all_dev:
			# DW is the start of all mdek labels
			if dev['name']!=None and dev['name'][:2]=='DW' and dev['name'][2:] not in self.idb:
				self.idb[dev['name'][2:]] = dev['address']
				self.range_matrix = np.mat([[0.0], [0.0], [0.0]])
				self.range_init = 0
				self.idr = {}
				# connect to the new anchor using bluetooth address of mdek
				self.blue(self.idb[dev['name'][2:]])
				# take an average for a good anchor to anchor estimate
				ranges = self.range_averages()
				# sort the data using labels
				for m,n in enumerate(self.idr):
					if self.idr[n] =='3b1a' or self.idr[n] =='081d':
						if self.Z[6] != 15.0:
							self.Z[6] = (self.Z[6]+ranges[n])/2
						else:
							self.Z[6] = ranges[n]
					else:
						if dev['name'][2:]=='1D08':
							if self.idr[n] =='1d82':
								self.Z[4] = ranges[n]
							if self.idr[n] =='3b05':
								self.Z[7] = ranges[n]
						else:
							if self.idr[n] =='1d82':
								self.Z[5] = ranges[n]
							if self.idr[n] =='3b05':
								self.Z[8] = ranges[n]
				print(self.Z.T)

	def blue(self,j):
		# connect to mdek using bluetooth address - need to be within adapter range
		print(j)
		self.adapter.start()
		cdev = self.adapter.connect(j)
		time.sleep(5)
		# configure anchor as a tag
		ldm = cdev.char_write("3f0afd88-7770-46b0-b5e7-9fc099598964",bytearray.fromhex("5d20"))
		self.adapter.stop()
		self.adapter.start()
		# has to reconnect to use as a tag
		cdev = self.adapter.connect(j)
		time.sleep(1)
		# mtu relates to the amount of information that will come through
		mtu = cdev.exchange_mtu(64)
		# receive 10 ranges from neighboring anchors to be averaged
		while len(self.range_matrix.T) < 10:
			cdev.subscribe('003bbdf2-c634-4b3d-ab56-7ec889b89a37',callback = self.handle_data)
		# reconfigure tag as anchor
		ldm = cdev.char_write("3f0afd88-7770-46b0-b5e7-9fc099598964",bytearray.fromhex("dd20"))
		self.adapter.stop()
		
	def handle_data(self,handle,value):
		# convert data into usable float format
		hex_data = hexlify(value)
		n = (len(hex_data)-4)/14
		ranges = np.mat([[0.0],[0.0],[0.0]])		
		for i in range(0,n):
			range_data = int(hex_data[(8+6)*i+8:(8+6)*i+16],16)
			ranges[i] = long(hex(struct.unpack('<L',struct.pack('>L',range_data))[0]),16)/1000.0
			id_data = hex_data[(8+6)*i+4:(8+6)*i+8]
			if id_data not in self.idr:
				self.idr[i] = id_data
		if self.range_init == 0:
			self.range_matrix = ranges
			self.range_init = 1
		else:
			# collect ranges to be averaged
			self.range_matrix = np.concatenate([self.range_matrix,ranges],1)
		
	def range_averages(self):
		# remove outliers and average
		for row in self.range_matrix:
			for i in range(len(row)):
				if(row.item(i)>row.mean()+2*row.std() or row.item(i)<row.mean()-2*row.std()):
					self.range_matrix = delete(self.range_matrix,i,1)
		return self.range_matrix.mean(1)

	def run(self):
		# bluetooth scan to get all anchor to anchor ranges
		self.scan()
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
			# print ranges through serial port
			self.ser.write("les\r")
			time.sleep(1)
			msg = UWB()
			print("serial connection established")
			# read and publish data	
			while not rospy.is_shutdown():
				raw_data = self.ser.readline()
				data = raw_data.split()
				# len(data) is the number of tag to anchor ranges
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
				# publish ros message
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
	# resetting the bluetooth adapter avoids errors
    subprocess.call('sudo hciconfig hci0 reset',shell=True)
