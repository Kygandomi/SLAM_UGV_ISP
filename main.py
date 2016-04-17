#! /usr/bin/env python
# SLAM UGV ISP
# Main Class -- Handles Mapping Performance
# by Katie Gandomi

import hokoyo_communication as hokoyo_comm
import pcb_communication as pcb_comm
import plotter
from time import sleep
import numpy

'This is the main class for performing fast slam'
class Fast_Slam():

	'Constructor'
	def __init__(self):
		# Set up components needed for robot
		self.initializer()

		# Main loop for executing program
		self.execute_main()

	'Initialize important components of program'
	def initializer(self):
		# Initalize particles
		self.initialize_particle_set()

		# Initialize plotter
		self.laser_scan_plotter = plotter.scan_plotter()

		# Connect to PCB over serial
		pcb_port = '/dev/tty.usbserial-A902U9B9'
		pcb_baud = 115200
		self.pcb_ser = pcb_comm.serial_comms(pcb_port, pcb_baud)
		self.pcb_ser.connect()

		# Connect to Hokoyo over serial on the RPI
		hokoyo_port = '/dev/tty.usbmodem1421'
		hokoyo_baud = 115200
		self.hokoyo_ser = hokoyo_comm.serial_comms(hokoyo_port, hokoyo_baud)
		self.hokoyo_ser.connect()

	'Main loop for executing the program'
	def execute_main(self): 
		# Take a laser scan of the environment
		points, distances = self.hokoyo_ser.request_scan_points()

		# Perform robot movement routine
		self.robot_movement_routine(distances)

		# Perform Fast Slam and Display best Map
		self.perform_fast_slam(distances)

		# Call Execute Main Method
		self.execute_main()

	'Perform Robot Movement Routine'
	def robot_movement_routine(self, distances):
		# Check Hokoyo scan to confirm the path is safe
		if(self.obstacle_in_path(distances)):
			# Perform rotation
			self.rotate()

		# If the path is safe adventure around! 
		else :
			# Move Robot Forward
			self.move_forward()

		# Stop Robot after action
		self.stop_robot()


	'Perform Grid-Based Fast Slam'
	def perform_fast_slam(self, distances):
		# For every particle in particle list :
		for particle in self.particles :
			# Update particle pose strictly with odometry
			self.update_particle_pose_with_odometry()

			# Correct particle pose with scan matching
			self.update_particle_pose_with_scan_matching()

			# Update particle's map of the world
			self.update_particle_map_with_scan()

			# Calculate particle's weight
			self.calculate_particle_weight()

		# Pick best map from particles
		best_map = self.pick_best_particle_map()

		# Resample from particles
		self.resample_routine()

	'Update Particles Pose based on odometry'
	def update_particle_pose_with_odometry(self):
		pass

	'Update Particles Pose based on Scan Matching'
	def update_particle_pose_with_scan_matching(self):
		pass

	'Update Particle Maps based on laser scan data'
	def update_particle_map_with_scan(self):
		pass

	'Calculate Particle weight based on how close scan data matches particle map'
	def calculate_particle_weight(self):
		pass

	'Resample particles based on their probabilities'
	def resample_routine(self):
		pass

	'Initialize a set of particles'
	def initialize_particle_set(self):
		# Number of particles in system
		self.num_particles = 10

		# Variable for storing particles
		self.particles = []

		# Set up particles
		for i in range(0, self.num_particles):
			# Every particle consists of :
			# An X,Y,Theta State
			x_pos = 0
			x_pos = 0
			x_pos = 'EAST'

			# A weight
			x_pos = 0

			# And a grid map
			grid_map = numpy.empty((500,500))
			grid_map.fill(0.5)

			# 
			r = [x_pos, y_pos, theta, weight, grid_map]
			self.particles.append(r)

	'Picks best particle from set and returns its map'
	def pick_best_particle_map(self):
		best_weight = -1
		best_index = -1
		for p in range(0, len(self.particles)) :
			if self.particles[p][3] > best_weight :
				best_weight = self.particles[p][3]
				best_index = p

		return self.particles[p][4]


	'Check laser scan readings for obstacles'
	def obstacle_in_path(self, distances):
		# Position of distance directly in front of myself
		center_index = 340

		if(len(distances) > 1) :
			# Range in front of robot
			path = []
			for i in range(0, 5) : 
				if (i == 0):
					path.append(distances[center_index])
				else :
					path.append(distances[center_index+1])
					path.append(distances[center_index-1])

			# For every distance directly in front of the robot
			for dist in path : 
				# Check to see if the distances is too small
				if dist > 0 and dist < 700 : 
					return True

		return False

	'Rotates Robot a certain number of encoder ticks'
	def rotate(self):
		# Get current encoder reading
		response = self.pcb_ser.recieve_packet()
		m1_enc_init, m2_enc_init, m3_enc_init = self.pcb_ser.parse_packet(response)
		m1_enc, m2_enc, m3_enc = m1_enc_init, m2_enc_init, m3_enc_init

		# Set thresholds for traveling forward
		m1_threshold = 50;
		m3_threshold = 50;

		# Set pwm signal for traveling forward
		m1_l1, m1_l2, m2_l1, m2_l2, m3_l1, m3_l2 = 0, 120, 0, 120, 0, 120

		# Move the robot forward for a certain number of encoder ticks
		print "Rotate..."

		counter = 0;
		while((abs(m1_enc - m1_enc_init) < m1_threshold) and (abs(m3_enc - m3_enc_init) < m3_threshold) and counter < 200):
			# Send move command
			self.pcb_ser.send_packet(m1_l1, m1_l2, m2_l1, m2_l2, m3_l1, m3_l2)
			sleep(1)

			# Debug Statement
			print "---"
			print response
			print m1_enc
			print m2_enc
			print m3_enc
			print abs(m1_enc - m1_enc_init)
			print abs(m2_enc - m2_enc_init)
			print abs(m3_enc - m3_enc_init)

			# Check incoming readings
			response = self.pcb_ser.recieve_packet()
			m1_enc, m2_enc, m3_enc = self.pcb_ser.parse_packet(response)
			sleep(1)

			counter = counter + 1

		# Clear Encoders for next time
		self.pcb_ser.clear_encoders

		# Update the zero position on the plot to the robot's new pos
		self.laser_scan_plotter.update_heading()

	'Move Robot Forward a certain number of encoder ticks'
	def move_forward(self):
		# Get current encoder reading
		response = self.pcb_ser.recieve_packet()
		m1_enc_init, m2_enc_init, m3_enc_init = self.pcb_ser.parse_packet(response)
		m1_enc, m2_enc, m3_enc = m1_enc_init, m2_enc_init, m3_enc_init

		# Set thresholds for traveling forward
		m1_threshold = 200;
		m3_threshold = 200;

		# Set pwm signal for traveling forward
		m1_l1, m1_l2, m2_l1, m2_l2, m3_l1, m3_l2 = 120, 0, 0, 0, 0, 120

		print "Moving Forward"

		# Move the robot forward for a certain number of encoder ticks
		counter = 0;
		while((abs(m1_enc - m1_enc_init) < m1_threshold) and (abs(m3_enc - m3_enc_init) < m3_threshold) and counter < 200):
			# Send move command
			self.pcb_ser.send_packet(m1_l1, m1_l2, m2_l1, m2_l2, m3_l1, m3_l2)
			sleep(1)

			# Check incoming readings
			response = self.pcb_ser.recieve_packet()
			m1_enc, m2_enc, m3_enc = self.pcb_ser.parse_packet(response)

			print "---"
			print response
			print m1_enc
			print m2_enc
			print m3_enc
			print abs(m1_enc - m1_enc_init)
			print abs(m2_enc - m2_enc_init)
			print abs(m3_enc - m3_enc_init)

			counter = counter + 1

		# Clear Encoders for next time
		self.pcb_ser.clear_encoders

		# Update the zero position on the plot to the robot's new pos
		avg = ((m1_enc_init - m1_enc) + (m2_enc_init - m2_enc))/2.5
		self.laser_scan_plotter.update_zero(avg)

	def stop_robot(self):
		self.pcb_ser.send_packet(0,0,0,0,0,0)

##############################
#    Main for Running Code   #
##############################
fs = Fast_Slam()
fs.laser_scan_plotter.mainloop()




