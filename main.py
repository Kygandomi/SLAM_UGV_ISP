#! /usr/bin/env python
# SLAM UGV ISP
# Main Class -- Handles Mapping Performance
# by Katie Gandomi

import hokoyo_communication as hokoyo_comm
import pcb_communication as pcb_comm
import scipy.stats
import plotter
import random
import numpy
import math
from time import sleep

'This is the main class for performing fast slam'
class Fast_Slam():

	'Constructor'
	def __init__(self):
		# Set up components needed for robot
		self.initializer()

		# Main loop for executing program
		# self.execute_main()

	'Initialize important components of program'
	def initializer(self):
		# Number of particles in system
		self.num_particles = 5

		# Factor by which to scale pixels to represent true distances
		self.pixel_scaling_factor = 25

		# Robot Initial Heading and Robot Position
		self.robot_heading = 'EAST'
		self.robot_pos = [200,200]

		# Set up grid map vairables
		self.gridx = 1000
		self.gridy = 800

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

		# Clear Encoders ? 
		self.pcb_ser.clear_encoders

		# Perform robot movement routine
		self.robot_movement_routine(distances)

		# Perform Fast Slam and Display best Map
		self.perform_fast_slam(points, distances)

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
		self.stop_robot()
		self.stop_robot()

	'Perform Grid-Based Fast Slam'
	def perform_fast_slam(self, points, distances):
		print 'Fast Slam--'

		# For every particle in particle list :
		for index in range(0, len(self.particles)) :
			# Get particle we're working with
			particle = self.particles[index]

			# Get particle variables
			x_pos = particle[0]
			y_pos = particle[1]
			theta = particle[2]
			g_map = particle[4]

			# Update particle pose strictly with odometry
			pose = self.update_particle_pose_with_odometry()

			# Correct particle pose with scan matching
			corrected_pose = self.update_particle_pose_with_scan_matching(points, pose, g_map)

			# Update particle's map of the world
			grid_map, weight = self.update_particle_map_with_scan(points, corrected_pose, g_map, distances)

			# Calculate particle's weight
			# weight = self.calculate_particle_weight(grid_map, distances)

			# Pose values to update particle location
			vx = corrected_pose[0]
			vy = corrected_pose[1]
			vz = corrected_pose[2]

			# Update this particle in the list
			self.particles[index] = [vx, vy, vz, weight, grid_map]

		# Pick best map from particles
		best_map, best_pose = self.pick_best_particle_map()

		# Print best map
		self.laser_scan_plotter.plot_matrix(best_map)
		self.laser_scan_plotter.plot_robot(best_pose)
		self.laser_scan_plotter.update() 

		# Normalize Particle Weights
		self.normalize_weights()

		# Resample from particles
		self.resample_routine()

	'Update Particles Pose based on odometry'
	def update_particle_pose_with_odometry(self):
		# Gaussian centered about the robot pos ?? 
		mu_x, sigma_x = self.robot_pos[0], self.robot_pos[0]
		mu_y, sigma_y = self.robot_pos[1], self.robot_pos[1]

		# Given a gaussian distribution centered at true value
		gaussian_x = random.gauss(mu_x, 5.0)
		gaussian_y = random.gauss(mu_y, 5.0)

		return [gaussian_x, gaussian_y, self.robot_heading]

	'Update Particles Pose based on Scan Matching'
	def update_particle_pose_with_scan_matching(self, points, pose, grid):
		# Current pose
		px = pose[0]
		py = pose[1]
		ptheta = pose[2]

		# Current best pose
		best = 0.5 * len(points)
		best_x = px
		best_y = py
		best_theta = ptheta

		# Check for angle differences from -3 to +3
		for angle in range(-3,3):
			# Check over x range from -3 to +3
			for p_x in range(px-3, px+3):
				# Check over y range from -3 to +3
				for p_y in range(py-3, py+3):
					total = 0.0
					for point in points : 
						# Shift particle
						x_pos = (points[0] - p_x)/self.pixel_scaling_factor
						y_pos = (points[1] - p_y)/self.pixel_scaling_factor

						# Get Angle we're considering
						theta = -1
						if(particle_pose[2] == 'EAST'):
							theta = 135
						elif(particle_pose[2] == 'WEST') :
							theta = 315
						elif(particle_pose[2] == 'SOUTH'):
							theta = 225
						elif(particle_pose[2] == 'NORTH'):
							theta = 0

						# Add Angle
						theta = theta + angle; 

						# Perform Rotation
						rx = (x_pos * math.cos(math.radians(theta))) - (y_pos * math.sin(math.radians(theta)))
						ry = (x_pos * math.sin(math.radians(theta))) + (y_pos * math.cos(math.radians(theta))) 

						# Now add translation 
						tx = int(rx + p_x)
						ty = int(ry + p_y)

						# Add this to running total
						total = total + grid[tx][ty]

					# If this points total is greater than the previous best total
					if(total > best){
						# Save this points information
						best = total
						best_x = p_x
						best_y = p_y
						best_theta = theta
					}

		# Return corrected pose
		return [best_x, best_y, best_theta]


	'Update Particle Maps based on laser scan data'
	def update_particle_map_with_scan(self, scan_points, particle_pose, g_map, distances):
		# Distance Index and Weight of particle
		index = -1
		weight = 0

		# First we need to transform points from laser scan 
		for points in scan_points :
			# Increment Index for obtaining distance associated to the given point
			index = index + 1
			dist = distances[index]

			# Get current varaibles
			x_pos = (points[0] - particle_pose[0])/self.pixel_scaling_factor
			y_pos = (points[1] - particle_pose[1])/self.pixel_scaling_factor
			theta = particle_pose[2]

			# Rotate First
			rx = (x_pos * math.cos(math.radians(theta))) - (y_pos * math.sin(math.radians(theta)))
			ry = (x_pos * math.sin(math.radians(theta))) + (y_pos * math.cos(math.radians(theta))) 

			# Then Translate
			tx = int(rx + particle_pose[0])
			ty = int(ry + particle_pose[1])

			# If a wall was detected
			if(points[2] == 0){
				g_map[tx][ty] = (g_map[tx][ty] + 1.0) / 2.0
			}

			# Angle and Distance from center to this new point
			delta_x = tx - int(particle_pose[0]) 
			delta_y = ty - int(particle_pose[1]) 
			delta_r = abs(math.sqrt(math.pow((delta_x), 2) + math.pow((delta_y), 2)))
			angle = math.atan2(delta_y, delta_x) 

			# Particle weight is a comparison between actual recorded distances and estimated distance
			# Summed over all particles
			weight = weight + abs(delta_r - dist) 

			# Then the matrix needs to be updated from particles pos to this coord
			for r in range(1, int(delta_r)) :
				pixel_x = int(r * math.cos(angle) + particle_pose[0])
				pixel_y = int(r * math.sin(angle) + particle_pose[1])

				if pixel_x < self.gridx and pixel_y < self.gridy: 
					g_map[pixel_x][pixel_y] = (g_map[pixel_x][pixel_y] + 0.0) /2.0

		# Return grid map (and weight for particle ?)
		return g_map, weight


	'Calculate Particle weight based on how close scan data matches particle map? o_o'
	def calculate_particle_weight(self, grid):
		total = 0.0
		for i in range(0, self.gridx):
			for j in range(0, self.gridy) : 
				total = total + grid[i][j]

		weight = total / (self.gridx * self.gridy)
		return weight

	'Normalizes the particle weights'
	def normalize_weights(self):
		total = 0
		for particle in self.particles:
			total = total + particle[3]

		for particle in self.particles:
			particle[3] = particle[3] / total

	'Resample particles based on their probabilities'
	def resample_routine(self):
		# Roulette wheel of particles 
		# Where each particles chance of being selected is given by its weight
		# So higher weighted particles have a higher chance of being selected
		roullette_wheel = []
		# Set up roullette wheel
		for p in range(0, len(self.particles)) : 
			if self.particles[p][3] < 0.1:
				for i in range(0, 1):
					roullette_wheel.append( self.particles[p] )
			elif self.particles[p][3] < 0.2:
				for i in range(0, 2):
					roullette_wheel.append( self.particles[p] )
			elif self.particles[p][3] < 0.3:
				for i in range(0, 3):
					roullette_wheel.append( self.particles[p] )
			elif self.particles[p][3] < 0.4:
				for i in range(0, 4):
					roullette_wheel.append( self.particles[p] )
			elif self.particles[p][3] < 0.5:
				for i in range(0, 5):
					roullette_wheel.append( self.particles[p] )
			elif self.particles[p][3] < 0.6:
				for i in range(0, 6):
					roullette_wheel.append( self.particles[p] )
			elif self.particles[p][3] < 0.7:
				for i in range(0, 7):
					roullette_wheel.append( self.particles[p] )
			elif self.particles[p][3] < 0.8:
				for i in range(0, 8):
					roullette_wheel.append( self.particles[p] )
			elif self.particles[p][3] < 0.9:
				for i in range(0, 9):
					roullette_wheel.append( self.particles[p] )
			elif self.particles[p][3] < 1.0:
				for i in range(0, 10):
					roullette_wheel.append( self.particles[p] )

		# Resample Particles with replacement
		for p in range(0, len(self.particles)) :
			if self.particles[p][3] < 0.4 :
				r = random.randint(0, len(roullette_wheel)-1)
				self.particles[p] = roullette_wheel[r]

	'Initialize a set of particles'
	def initialize_particle_set(self):
		# Variable for storing particles
		self.particles = []

		# Set up particles
		for i in range(0, self.num_particles):
			# Every particle consists of :
			# An X,Y,Theta State
			x_pos = 200
			y_pos = 200
			theta = 'EAST'

			# A weight
			weight = 0

			# And a grid map
			grid_map = numpy.empty((self.gridx,self.gridy))
			grid_map.fill(0.5)

			# Place particle in list
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

		return self.particles[p][4], [self.particles[p][0], self.particles[p][1]]


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
		print "Rotate"

		counter = 0;
		while((abs(m1_enc - m1_enc_init) < m1_threshold) and (abs(m3_enc - m3_enc_init) < m3_threshold) and counter < 200):
			# Send move command
			self.pcb_ser.send_packet(m1_l1, m1_l2, m2_l1, m2_l2, m3_l1, m3_l2)
			sleep(1)

			# # Debug Statement
			# print "---"
			# print response
			# print m1_enc
			# print m2_enc
			# print m3_enc
			# print abs(m1_enc - m1_enc_init)
			# print abs(m2_enc - m2_enc_init)
			# print abs(m3_enc - m3_enc_init)

			# Check incoming readings
			response = self.pcb_ser.recieve_packet()
			m1_enc, m2_enc, m3_enc = self.pcb_ser.parse_packet(response)
			sleep(1)

			counter = counter + 1

		# Clear Encoders for next time
		self.pcb_ser.clear_encoders

		# Update the zero position on the plot to the robot's new pos
		self.update_robot_heading()

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

			# print "---"
			# print response
			# print m1_enc
			# print m2_enc
			# print m3_enc
			# print abs(m1_enc - m1_enc_init)
			# print abs(m2_enc - m2_enc_init)
			# print abs(m3_enc - m3_enc_init)

			counter = counter + 1

		# Clear Encoders for next time
		self.pcb_ser.clear_encoders

		# Update the zero position on the plot to the robot's new pos
		v1 = ((abs(m1_enc_init - m1_enc) / 64) * 82) /25.0
		v2 = ((abs(m2_enc_init - m2_enc) / 64) * 82) /25.0
		avg = (v1 + v2) / 2.0
		self.update_robot_pos(avg)

	'Command for stopping robot movement'
	def stop_robot(self):
		self.pcb_ser.send_packet(0,0,0,0,0,0)

	'Updates the Robots Position'
	def update_robot_pos(self, dist):
		dist = dist / 100

		if(self.robot_heading == 'EAST') :
			self.robot_pos[0] = self.robot_pos[0] + dist
			self.robot_pos[1] = self.robot_pos[1]
		elif (self.robot_heading == 'SOUTH') :
			self.robot_pos[0] = self.robot_pos[0]
			self.robot_pos[1] = self.robot_pos[1] + dist
		elif (self.robot_heading == 'NORTH') :
			self.robot_pos[0] = self.robot_pos[0]
			self.robot_pos[1] = self.robot_pos[1] - dist
		elif (self.robot_heading == 'WEST') :
			self.robot_pos[0] = self.robot_pos[0] - dist
			self.robot_pos[1] = self.robot_pos[1]

	'Update the Robot Heading'
	def update_robot_heading(self):
		if self.robot_heading == 'EAST' :
			self.robot_heading = 'SOUTH'
		elif self.robot_heading == 'SOUTH' :
			self.robot_heading = 'WEST'
		elif self.robot_heading == 'WEST' :
			self.robot_heading = 'NORTH'
		elif self.robot_heading == 'NORTH' :
			self.robot_heading = 'EAST'

##############################
#    Main for Running Code   #
##############################
fs = Fast_Slam()
fs.execute_main()
fs.laser_scan_plotter.mainloop()




