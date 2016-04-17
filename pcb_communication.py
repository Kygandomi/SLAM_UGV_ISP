#! /usr/bin/env python
# SLAM UGV ISP
# Serial Class -- Handles Communication to PCB
# by Katie Gandomi

import serial

'This class handles serial communication with pcb'
class serial_comms():

	'Constructor for serial class, sets up serial comms'
	def __init__(self, port, baud):
		# Port and Baud Rate to be used
		self.port = port
		self.baud = baud
		self.ser = ''

	'Connects to PCB over serial'
	def connect(self):
		 # Connect to serial port
		self.ser = serial.Serial(self.port, self.baud, timeout=1)

		# Open Serial Port if possible
		if self.ser.isOpen() : 
			print(self.ser.name + ' is open...')

	'Closes serial port to PCB'
	def disconnect(self):
		self.ser.close()

	'Write data to the PCB'
	def send_packet(self, m1_l1, m1_l2, m2_l1, m2_l2, m3_l1, m3_l2):
		# Write Command to PCB
		self.ser.write(b'\xfe')
		self.ser.write(b'\xfe')
		self.ser.write(chr(m1_l1))
		self.ser.write(chr(m1_l2))
		self.ser.write(chr(m2_l1))
		self.ser.write(chr(m2_l2))
		self.ser.write(chr(m3_l1))
		self.ser.write(chr(m3_l2))

	'Clears data written to encoders'
	def clear_encoders(self):
		self.ser.write(b'\xfe')
		self.ser.write(b'\xef')

	'Read data from the PCB'
	def recieve_packet(self):
		# Collect output response
		response = []
		while (self.ser.inWaiting()):
			output = ord(self.ser.read())
			response.append(output)

		return response

	'Parse recieved data'
	def parse_packet(self, response):
		# Declare variables
		m1_enc, m2_enc, m3_enc = -3,-3,-3

		# Check to make sure the packet is atleast 13 bytes long
		if (len(response) < 13):
			return -1, -1, -1

		# Find the start bytes and the execute action byte
		enc_pos = -1
		for byte_pos in range(0, len(response)):
			if byte_pos + 3 < len(response):
				if response[byte_pos] == 254 and response[byte_pos+1] == 254 and response[byte_pos+2] == 0:
					enc_pos = byte_pos + 3

		# If we found something valid, check the remaining bytes
		if enc_pos != -1 and enc_pos + 11 < len(response):
			m1_enc = (response[enc_pos] << 24) + (response[enc_pos+1] << 16) + (response[enc_pos+2] << 8) + (response[enc_pos+3])
			m2_enc = (response[enc_pos+4] << 24) + (response[enc_pos+5] << 16) + (response[enc_pos+6] << 8) + (response[enc_pos+7])
			m3_enc = (response[enc_pos+8] << 24) + (response[enc_pos+9] << 16) + (response[enc_pos+10] << 8) + (response[enc_pos+11])
		else :
			return -2, -2, -2

		return m1_enc, m2_enc, m3_enc





