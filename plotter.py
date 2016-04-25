#! /usr/bin/env python
# SLAM UGV ISP
# Plotter Class -- Plots Map Data
# by Katie Gandomi

import Tkinter as tk
import math
from PIL import Image, ImageTk, ImageDraw
import numpy


'This class plots results from laser scanner'
class scan_plotter(tk.Tk):

	'Constructor for plotter class, sets up drawing canvas'
	def __init__(self, *args, **kwargs):
		# Set up drawing environment 
		tk.Tk.__init__(self, *args, **kwargs)
		self.title("Plot Laser Scan - Gandomi")
		self.index = 0

		# Set up canvas variables 
		self.canvas_width = 800
		self.canvas_height = 500

		self.canvas_center = [200, 200]

		# Data for storing RGB color information
		self.data = numpy.zeros( (self.canvas_height,self.canvas_width, 3), dtype=numpy.uint8 )
		# self.data.fill([50, 50, 200])

		# Create a frame
		self.frame = tk.Frame(self, width=self.canvas_width, height=self.canvas_height)
		self.frame.pack()

		# Create a canvas to draw on 
		self.canvas = tk.Canvas(self.frame, width=self.canvas_width,height=self.canvas_height)
		self.canvas.place(x=-1,y=-1)

		# Create image object for handeling pixels
		self.img = tk.PhotoImage(width=self.canvas_width, height=self.canvas_height)
		self.canvas.create_image((self.canvas_width/2, self.canvas_height/2), image=self.img, state="normal")

		# Points for plotting
		self.points = {}
		self.rays = {}
		self.robot_oval = {}

		# Robot Initial Heading
		self.robot_heading = 'EAST'

	'Plots a matrix of coordinate points'
	def plot_matrix(self, map_matrix):
		for i in range(0, len(map_matrix)):
			for j in range(0, len(map_matrix[i])):
				val = map_matrix[i][j]
				if val > 0.5 :
					# Place a blue pixel #3232FF
					self.data[j][i] = [50, 50, 200]
				elif val < 0.5:
					# Place a white pixel #FFFFFF
					self.data[j][i] = [255, 255, 255]
				elif val == 0.5:
					# Place a grey pixel #B9B9B9
					self.data[j][i] = [	185, 185, 185]

		self.im=Image.frombytes('RGB', (self.data.shape[1], self.data.shape[0]), self.data.astype('b').tostring())
		self.photo = ImageTk.PhotoImage(image=self.im)
		self.canvas.create_image(0,0,image=self.photo,anchor=tk.NW)

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

	def hough_transform_simple(self):
		hough_accum = numpy.zeros( (len(self.data), len(self.data[0])), dtype=numpy.uint8 )
		for x in range(0, len(self.data)):
			for y in range(0, len(self.data[x])):
				if self.data[x][y][0] == 50 and self.data[x][y][1] == 50 and self.data[x][y][2] == 200 :
					# Equation of line in param space
					for k in range(0, len(self.data)):
						d = abs(-x * k + y)
						print d
						print k
						if d > 0 and d < len(self.data) and k > 0 and k < len(self.data):
							hough_accum[d][k] = hough_accum[d][k] + 1
						else: 
							break

		# Plot Lines in new PIL image
		this_im = Image.new('L', (self.data.shape[1], self.data.shape[0]), color=255)
		draw = ImageDraw.Draw(this_im) 

		for d in range(0, len(self.data)):
			for k in range(0, len(self.data[0])):
				# If atleast ten points run through here
				if hough_accum[d][k] > 2 :
					print 'Greater than 2 ~~~~~~~~~'
					start_line = [-1,-1]
					end_line = [-1,-1]
					for x in range(0, len(self.data)):
						y = k * x + d
						if y < len(self.data[x]) and y > 0:
							if self.data[x][y][0] == 50 and self.data[x][y][1] == 50 and self.data[x][y][2] == 200 :
								if start_line[0] == -1 and start_line[1] == -1 :
									start_line[0] = x
									start_line[1] = y
								else:
									end_line[0] = x
									end_line[1] = y

					# Draw line 
					if start_line[0] != -1 and start_line[1] != -1 and end_line[0] != -1 and end_line[1] != -1:
						print 'DOING THE HT!'
						print start_line
						print end_line
						draw.line((start_line[0], start_line[1], end_line[0],end_line[1]), fill=150, width=2)

	'Perform the hough transform for line detection!'
	def hough_transform(self):
		print 'HT'

		# Get Image Center for Hough Transform
		imageCenterX = self.canvas_width / 2
		imageCenterY = self.canvas_height / 2

		# Variables needed to perform Hough Transform
		nAng = 256
		nRad = 256
		rMax = math.sqrt(math.pow(imageCenterX, 2) + math.pow(imageCenterY, 2))
		dAng = math.pi / nAng
		cRad = nRad / 2
		dRad = (2*rMax) / nRad

		# Fill Hough Accumulator
		hough_accum = numpy.zeros( (nAng,nRad), dtype=numpy.uint8 )
		for x in range(0, len(self.data)):
			for y in range(0, len(self.data[x])):
				# If the pixel is an edge pixel
				if self.data[x][y][0] == 50 and self.data[x][y][1] == 50 and self.data[x][y][2] == 200 :
					# Then get location of this pixel relative to center of image
					posX = x - imageCenterX
					posY = y - imageCenterY

					# Then...
					for angle in range(0, nAng):
						theta = dAng * angle
						r = cRad + (x * math.cos(math.radians(theta)) + y * math.sin(math.radians(theta)))/dRad

						if r >= 0 and r < nRad :
							hough_accum[angle][r] = hough_accum[angle][r] + 1

		# Get max lines from hough accum
		lines = []
		for x in range(0, nAng):
			for y in range(0, nRad):
				if hough_accum[x][y] > 70 :
					print '....'
					print hough_accum[x][y]
					lines.append([x,y])

		print lines

		# Plot Lines in new PIL image
		this_im = Image.new('L', (self.data.shape[1], self.data.shape[0]), color=255)
		draw = ImageDraw.Draw(this_im) 

		# Get Equation of Line in Hessian Normal Form
		for line in lines:
			theta = line[0]
			r = line[1]
			start_line = [-1,-1]
			end_line = [-1,-1]
			# Hmmm there has to be a better way to do this...
			if math.cos(math.radians(theta)) == 0 :
				# Y = Constant, X = Changing
				for y in range(0, len(self.data[0])):
					x = r
					if self.data[x][y][0] == 50 and self.data[x][y][1] == 50 and self.data[x][y][2] == 200 :
						if start_line[0] == -1 and start_line[1] == -1 :
							start_line[0] = x
							start_line[1] = y
						else:
							end_line[0] = x
							end_line[1] = y

			elif math.sin(math.radians(theta)) == 0:
				# X = Constant, Y = Changing
				for x in range(0, len(self.data)):
					y = r
					if self.data[x][y][0] == 50 and self.data[x][y][1] == 50 and self.data[x][y][2] == 200 :
						if start_line[0] == -1 and start_line[1] == -1 :
							start_line[0] = x
							start_line[1] = y
						else:
							end_line[0] = x
							end_line[1] = y
			else :
				# Both are changing
				for x in range(0, len(self.data)):
					y = (r - x*math.cos(math.radians(theta)))/math.sin(math.radians(theta))
					if y < len(self.data[x]) and y > 0:
						# Equation of line in Hessian Normal Form
						if self.data[x][y][0] == 50 and self.data[x][y][1] == 50 and self.data[x][y][2] == 200 :
							if start_line[0] == -1 and start_line[1] == -1 :
								start_line[0] = x
								start_line[1] = y
							else:
								end_line[0] = x
								end_line[1] = y
			
			if start_line[0] != -1 and start_line[1] != -1 and end_line[0] != -1 and end_line[1] != -1:
				print 'DOING THE HT!'
				print start_line
				print end_line
				draw.line((start_line[0], start_line[1], end_line[0],end_line[1]), fill=150, width=2)

		# Display Result
		this_im.show('image' + str(self.index))









