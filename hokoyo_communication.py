#! /usr/bin/env python
# SLAM UGV ISP
# Serial Class -- Handles Communication to Hokoyo
# by Katie Gandomi

import serial
import math
import plotter
import matplotlib.pyplot as plt
from time import sleep

'This class handles serial communication with laser scanner'
class serial_comms():

	'Constructor for serial class, sets up serial comms'
	def __init__(self, port, baud):
		# Port and Baud Rate to be used
		self.port = port
		self.baud = baud
		self.ser = ''

	'Method for calling process to be tested'
	def request_scan_points(self):
		# Connect to serial port
		self.connect()

		# Request data from sensor
		request = self.request_data()
		data_dist = self.decode_dist_data(request)

		# Convert to X-Y coordinates
		points = self.convert_dist_to_points(data_dist)

		# Disconnect from port
		self.disconnect()

		# Return scan points
		return points, data_dist

	'Connects to Hokuyo over serial'
	def connect(self):
		 # Connect to serial port
		self.ser = serial.Serial(self.port, self.baud, timeout=1)

		# Open Serial Port if possible
		if self.ser.isOpen() : 
			print(self.ser.name + ' is open...')

	'Closes serial port to Hokuyo'
	def disconnect(self):
		self.ser.close()
		# exit()

	'Calculates the average of a set of requests'
	def average_requests(self, requests):
		avg = []
		for i in range(0, len(requests[0])) : 
			total = 0.0
			for j in range(0,len(requests)) :
				total = total + requests[j][i]

			total = total / len(requests)
			avg.append(total)

		print "Average.."
		print avg

		return avg


	'This method requests data from the Laser Scanner'
	def request_data(self):
		# Command for requesting data from sensor
		# cmd = 'MD0010075001001'
		cmd = 'MS0044072501001'

		# Write Command to sensor
		self.ser.write(cmd + '\n')

		# Sleep to allow sensor time to return request
		sleep(0.3)
		# self.ser.readline()

		# Collect output response
		response = []
		while (self.ser.inWaiting()):
			output = self.ser.readline().strip('\n')
			response.append(output[:-1])

		# print "Response"
		# print response
		# print ''

		return response

	'Decodes distance data from request'
	def decode_dist_data(self, response):
		# Collect distance terms from the output
		dist = []
		flag_index = 1000000
		for index in range(0, len(response)):
			if response[index] == '99' :
				flag_index = index + 1

			if(index > flag_index):
				dist.append(response[index])

		distancedata = ''
		distancedata = distancedata.join(dist)

		# print "Distance Data"
		# print distancedata
		# print ''

		# Shift bits from distance data, to get something sensible
		distances = [ ]

		for i in range(1, len(distancedata), 2):
			v = ((ord(distancedata[i-1])-48)<<6) + ((ord(distancedata[i])-48))
			distances.append(v)

		print "Distance..."
		print distances
		print ''

		return distances

	'Convert distance data to coordinate points'
	def convert_dist_to_points(self, distances):
		# Convert distances to a set of points
		start = 44
		curve = [ ]
		for i in range(0, len(distances)):
		    v = distances[i]
		    a = 0
		    if (v == 0) : 
		    	v = 4100
		    	a = -1
		    th = math.radians((44 + i)*0.36)
		    curve.append((-v * math.sin(th), v * math.cos(th), a))

		print "Plot Points..."
		print curve
		print ''

		return curve

	'Plot points'
	def plot(self, points):
		x = []
		y = []
		for coords in points : 
			x.append(coords[0])
			y.append(coords[1])

		plt.plot(x, y, 'ro')
		plt.ylabel('Plot of serial points')
		plt.show()