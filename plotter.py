#! /usr/bin/env python
# SLAM UGV ISP
# Plotter Class -- Plots Map Data
# by Katie Gandomi

import Tkinter as tk
import math


'This class plots results from laser scanner'
class scan_plotter(tk.Tk):

	'Constructor for plotter class, sets up drawing canvas'
	def __init__(self, *args, **kwargs):
		# Set up drawing environment 
		tk.Tk.__init__(self, *args, **kwargs)
		self.title("Plot Laser Scan - Gandomi")

		# Set up canvas variables 
		canvas_width = 1000
		canvas_height = 800

		self.canvas_center = [200, 200]

		# Create a Canvas to draw on 
		self.canvas = tk.Canvas(self, width=canvas_width, height=canvas_height, borderwidth=0, highlightthickness=0)
		self.canvas.pack(fill=tk.BOTH, expand=1) 

		# Create image object for handeling pixels
		self.img = tk.PhotoImage(width=canvas_width, height=canvas_height)
		self.canvas.create_image((canvas_width/2, canvas_height/2), image=self.img, state="normal")

		# Points for plotting
		self.points = {}
		self.rays = {}
		self.robot_oval = {}

		# Robot Initial Heading
		self.robot_heading = 'EAST'

	'Plots a matrix of coordinate points'
	def plot_matrix(self, map_matrix):
		for i in range(0, len(map_matrix)):
			for j in range(0, len(map_matrix[0])):
				val = map_matrix[i][j]
				if val > 0.5 :
					# Place a blue pixel
					self.img.put("#3232FF", (i,j))
				elif val < 0.5:
					# Place a white pixel
					self.img.put("#FFFFFF", (i,j))
				elif val == 0.5:
					# Place a grey pixel
					self.img.put("#B9B9B9", (i,j))

	'Plots robot in world'
	def plot_robot(self, pose):
		if len(self.robot_oval) != 0 :
			for oval in self.robot_oval.values():
				self.canvas.delete(oval)

		self.robot_oval[pose[0], pose[1]] = self.canvas.create_oval(pose[0] - 3, pose[1] - 3, pose[0] + 3, pose[1] + 3, fill="red", tags="oval")


	'Plots points recieved from serial class'
	def plot_points(self, points):
		for coords in points :
			x_pos = (coords[0] - self.canvas_center[0])/20.0
			y_pos = (coords[1] - self.canvas_center[1])/20.0

			theta = -1
			if(self.robot_heading == 'EAST'):
				theta = 135
			elif(self.robot_heading == 'WEST') :
				theta = 315
			elif(self.robot_heading == 'SOUTH'):
				theta = 225
			elif(self.robot_heading == 'NORTH'):
				theta = 0

			# Rotate First
			rx = (x_pos * math.cos(math.radians(theta))) - (y_pos * math.sin(math.radians(theta)))
			ry = (x_pos * math.sin(math.radians(theta))) + (y_pos * math.cos(math.radians(theta))) 

			# Then Translate
			tx = int(rx + self.canvas_center[0])
			ty = int(ry + self.canvas_center[1])

			# Plot result
			if(coords[2] == 0) : 
				self.points[tx, ty] = self.canvas.create_line(tx, ty, tx+1, ty+1, width=2,fill="blue")
			self.rays[tx, ty, self.canvas_center[0], self.canvas_center[1]] = self.canvas.create_line(self.canvas_center[0], self.canvas_center[1], tx+1, ty+1, width=2,fill="white")

		self.canvas.update()

	'Updates the zero position for the plot'
	def update_zero(self, dist):
		dist = dist / 100

		if(self.robot_heading == 'EAST') :
			self.canvas_center[0] = self.canvas_center[0] + dist
			self.canvas_center[1] = self.canvas_center[1]
		elif (self.robot_heading == 'SOUTH') :
			self.canvas_center[0] = self.canvas_center[0]
			self.canvas_center[1] = self.canvas_center[1] + dist
		elif (self.robot_heading == 'NORTH') :
			self.canvas_center[0] = self.canvas_center[0]
			self.canvas_center[1] = self.canvas_center[1] - dist
		elif (self.robot_heading == 'WEST') :
			self.canvas_center[0] = self.canvas_center[0] - dist
			self.canvas_center[1] = self.canvas_center[1]

	'Update Robot Heading after rotation'
	def update_heading(self):
		if self.robot_heading == 'EAST' :
			self.robot_heading = 'SOUTH'
		elif self.robot_heading == 'SOUTH' :
			self.robot_heading = 'WEST'
		elif self.robot_heading == 'WEST' :
			self.robot_heading = 'NORTH'
		elif self.robot_heading == 'NORTH' :
			self.robot_heading = 'EAST'

	'This method deletes all points from the canvas'
	def clear_points(self):
		pass

	'Perform the hough transform!'
	def hough_transform(self):
		pass






