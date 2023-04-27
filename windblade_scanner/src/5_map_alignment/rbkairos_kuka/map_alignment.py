#!/usr/bin/python
'''
The octomap merge package expects the "Neighbor" Octomap to use the custom
msg type defined by the package... so, even if merging only 2 maps, need to 
convert msg types to make this possible. 
'''
# ROS general imports
import rospy
import tf

# Msg imports
from visualization_msgs.msg import MarkerArray, Marker
# To Enable Dynamic Reconfigure (WIP) 
from dynamic_reconfigure.server import Server
from windblade_scanner.cfg import rbkairos_alignmentConfig

# For Contour Plotting 
import numpy as np
import matplotlib.pyplot as plt
from colour import Color

# For Alignment 
import math

# For path planning comparisons 
from nav_msgs.msg import Odometry


class MapAlignment():

	def __init__(self, loop_rate=5.0, plot_visualize = False):

		# Safety 
		self.rate = rospy.Rate(loop_rate)

		# Time stamp for both
		self.time_stamp = rospy.Time.now()

        # Define topics 
		self.node_name = "map_alignment/"
		self.map_turtle = str(rospy.get_param(self.node_name + "loaded_map_topic_turtlebot"))
		self.topic_rbkairos = str(rospy.get_param(self.node_name + "map_topic_rbkairos"))
		self.odom = str(rospy.get_param(self.node_name + "odom_topic"))

		# Waypoint Check Data 
		self.waypoint_x = str(rospy.get_param(self.node_name + "waypoint_x"))
		self.waypoint_y = str(rospy.get_param(self.node_name + "waypoint_y"))
		self.waypoint_tol = str(rospy.get_param(self.node_name + "waypoint_tol"))

		# Turtlemap filter limits
		self.xmin_turtle = float(rospy.get_param(self.node_name + "xmin_turtle"))
		self.xmax_turtle = float(rospy.get_param(self.node_name + "xmax_turtle"))
		self.ymin_turtle = float(rospy.get_param(self.node_name + "ymin_turtle"))
		self.ymax_turtle = float(rospy.get_param(self.node_name + "ymax_turtle"))
		self.zmin_turtle = float(rospy.get_param(self.node_name + "zmin_turtle"))
		self.zmax_turtle = float(rospy.get_param(self.node_name + "zmax_turtle"))

		# Rbkairos filter limits
		self.xmin_rbkairos = float(rospy.get_param(self.node_name + "xmin_rbkairos"))
		self.xmax_rbkairos = float(rospy.get_param(self.node_name + "xmax_rbkairos"))
		self.ymin_rbkairos = float(rospy.get_param(self.node_name + "ymin_rbkairos"))
		self.ymax_rbkairos = float(rospy.get_param(self.node_name + "ymax_rbkairos"))
		self.zmin_rbkairos = float(rospy.get_param(self.node_name + "zmin_rbkairos"))
		self.zmax_rbkairos = float(rospy.get_param(self.node_name + "zmax_rbkairos"))

		# Subscribers, publishers 																								
		self.align_sub_turtle=rospy.Subscriber(self.map_turtle, MarkerArray, self.turtle_scrubber)
		# Loaded Map 
		self.align_sub_rbkairos=rospy.Subscriber(self.topic_rbkairos, MarkerArray, self.rbkairos_scrubber)
		# Autonomous 
		self.odom_sub=rospy.Subscriber(self.odom, Odometry,self.waypoint_checker)


		self.align_pub1 = rospy.Publisher("/occupied_cells_loaded_map_scrubbed_turtle",MarkerArray, queue_size=1)
		self.align_pub2 = rospy.Publisher("/occupied_cells_map_scrubbed_rbkairos",MarkerArray, queue_size=1)
		self.align_pub = rospy.Publisher("/occupied_cells_map_scrubbed_merged",MarkerArray, queue_size=1)

		# Msgs
		self.loaded_map_info_turtle = MarkerArray()
		self.final_map_info_turtle = MarkerArray()
		self.loaded_map_info_rbkairos = MarkerArray()
		self.final_map_info_rbkairos = MarkerArray()
		# Fullmerge
		self.fullmerge=MarkerArray()

	## Waypoints 
		self.at_waypoint = False

	## Visualization
		# Only want ONE data set. 
		self.data_recieved_robot = False 
		self.data_recieved_map = False 
		# Countour line publisher. 
		self.contour_pub = rospy.Publisher("/contours",Marker, queue_size=1)
		# Containers for contour plots. 
		self.contour_data=[[],[]]
		self.plot_visualize = plot_visualize
		self.plotted = False
		self.plot = 0
		self.features_found = False
	
	## Alignment
		self.features = []
		self.map_z_levels = []
		self.aligned = False
		self.map_rescaled = False

	## Merging 
		self.merged = False

# Helper functions
	def quat_to_euler(self, x, y, z, w):
		quat = [x,y,z,w]
		roll,pitch,yaw = tf.transformations.euler_from_quaternion(quat)
		return [roll,pitch,yaw]

	def euler_to_quat(self, r, p, y):
		x,y,z,w = tf.transformations.quaternion_from_euler(r,p,y)
		return [x,y,z,w]

##########################
### Waypoint Checker
##########################
	def waypoint_checker(self, msg):
		if not self.at_waypoint:
			# Check if in tolerance of desired waypoint.
			# IF within tolerance, will perform the map alignment and merge. 
			x_pos = msg.pose.pose.position.x
			y_pos = msg.pose.pose.position.y
			if x_pos >= self.waypoint_x-self.waypoint_tol/2 and x_pos <= self.waypoint_x+self.waypoint_tol/2\
			and y_pos >= self.waypoint_y-self.waypoint_tol/2 and y_pos <= self.waypoint_y+self.waypoint_tol/2:
				self.at_waypoint = True

##########################
### Turtle scrubber
##########################
	def turtle_scrubber(self, msg):
		# ONLY want 1 MSG. 
		if not self.data_recieved_map:
			self.data_recieved_map = True
			loaded_info_array1 = msg.markers

			######################
			# Partition Data
			######################
			'''
			info_size_list1 = []
			for i in range(0,len(loaded_info_array1)):
				item = (i, len(loaded_info_array1[i].points), len(loaded_info_array1[i].colors))
				info_size_list1.append(item)
				#rospy.loginfo("Turtle %s",str(i))
			#rospy.loginfo("Rbkairos List %s",str(info_size_list1))
			'''

			# Save Raw Data
			self.loaded_map_info_turtle.markers = [loaded_info_array1[15],loaded_info_array1[16]]
			
			#rospy.loginfo("BASE MAP LENGTH %s",str(len(self.loaded_map_info_turtle.markers[1].points)))

			############################
			# RANGE FILTER
			############################

			for array in range(0,len(self.loaded_map_info_turtle.markers)):
				#print(array)
				#print("Original Length Array", array,len(self.loaded_map_info_turtle.markers[array].points))

				anomoly_count_range1 = 0
				anomoly_list_range1 = []

				list1 = []

				# Range Filter
				for i in range(0,len(self.loaded_map_info_turtle.markers[array].points)):
					z = self.loaded_map_info_turtle.markers[array].points[i].z
					y = self.loaded_map_info_turtle.markers[array].points[i].y
					x = self.loaded_map_info_turtle.markers[array].points[i].x
					if x < (self.xmin_turtle) or x > (self.xmax_turtle) or y < (self.ymin_turtle) or y > (self.ymax_turtle) or z< (self.zmin_turtle) or z > (self.zmax_turtle):
						'''
						if x < (self.xmin_turtle) or x > (self.xmax_turtle):
							list1.append((x,y,z,"x viol", "xmin:",self.xmin_turtle, "xmax:", self.xmax_turtle))
						elif y < (self.ymin_turtle) or y > (self.ymax_turtle):
							list1.append((x,y,z, "y viol", "ymin:",self.ymin_turtle, "ymax:", self.ymax_turtle))
						else:
							list1.append((x,y,z, "z viol","zmin:",self.zmin_turtle, "zmax:", self.zmax_turtle))
						'''
						anomoly_count_range1+=1
						item = ("index", i, x,y,z)
						anomoly_list_range1.append(item)

				for i in range(0,len(list1)):
					rospy.loginfo("z list, %s: %s", str(i),str(list1[i]))
				#######################
				# Remove Anomolies
				#######################
				for i in range(0,len(anomoly_list_range1)):
					index = anomoly_list_range1[i][1]-i
					del self.loaded_map_info_turtle.markers[array].points[index]
					del self.loaded_map_info_turtle.markers[array].colors[index]

				# Save Data
				#print("SCRUBBED 1!")
				rospy.logdebug("Scrubbed Length Array %s %s", str(array),str(len(self.loaded_map_info_turtle.markers[array].points)))

			if len(self.final_map_info_turtle.markers) == 0:
				self.final_map_info_turtle.markers = self.loaded_map_info_turtle.markers

		##############################################
		# Map Align Part I: Elevation Data Reorganization 
		##############################################
			### 1. Get elevation data. How many curves working with. 
			# Voxel list 
			voxel_list1 = []
			# Elevation data 
			z_counter1 = 0
			z_level1 = []
			z_data1 = []
			for i in range(0,len(self.loaded_map_info_turtle.markers[1].points)):
				x = round(self.loaded_map_info_turtle.markers[1].points[i].x,3)
				y = round(self.loaded_map_info_turtle.markers[1].points[i].y,3)
				z = round(self.loaded_map_info_turtle.markers[1].points[i].z,3)
				# Save unique elevation data, least to greatest. 
				if z not in z_data1:
					z_counter1+=1
					z_level1.append(z)
					z_level1.sort()
				# Save Data
				z_data1.append(z)
				voxel_list1.append((x,y,z))

		###### Store the elevation data for reference to scanned map. 
			self.map_z_levels = z_data1

			# Make List to store values for contour fitting.  
			contour_data1 = []
			for i in range(0,z_counter1):
				contour_data1.append((z_level1[i],[[],[]]))


			rospy.logdebug("Z_count Turtle %s", str(z_counter1))
			rospy.logdebug("Z_level Turtle %s", str(z_level1))

			### 2. Only the topmost curve matters, but will curvefit all curves for visualization. 
			# Resort Data for Curve Fitting
			for i in range(0,len(voxel_list1)):
				# Check for elevation match. Save x,y if found. 
				for level in range(0,len(contour_data1)):
					if voxel_list1[i][2] == contour_data1[level][0]:
						contour_data1[level][1][0].append(voxel_list1[i][0])
						contour_data1[level][1][1].append(voxel_list1[i][1])
						break

			####self.feature_finder(data=contour_data1,map_name="Turtle")
			# Save Data for plotting 
			self.contour_data[0] = contour_data1
			#rospy.loginfo(contour_data1)

###########################
### Rbkairos scrubber
###########################
	def rbkairos_scrubber(self, msg):
		# ONLY want 1 MSG from the robot once its AT the waypoint.
		if not self.data_recieved_robot and self.at_waypoint:
			self.data_recieved_robot = True
			loaded_info_array2 = msg.markers

			######################
			# Partition Data
			######################
			'''
			info_size_list2 = []
			for i in range(0,len(loaded_info_array2)):
				item = (i, len(loaded_info_array2[i].points), len(loaded_info_array2[i].colors))
				info_size_list2.append(item)
				#rospy.loginfo("Rbkairos %s",str(i))
			#rospy.loginfo("Rbkairos List %s",str(info_size_list2))
			'''
			# Save Raw Data
			self.loaded_map_info_rbkairos.markers = [loaded_info_array2[15],loaded_info_array2[16]]

			############################
			# RANGE FILTER
			############################

			for array in range(0,len(self.loaded_map_info_rbkairos.markers)):
				#print(array)
				#print("Original Length 1", len(self.loaded_map_info_rbkairos.markers[array].points))

				anomoly_count_range2 = 0
				anomoly_list_range2 = []

				# Range Filter
				for i in range(0,len(self.loaded_map_info_rbkairos.markers[array].points)):
					z = self.loaded_map_info_rbkairos.markers[array].points[i].z
					y = self.loaded_map_info_rbkairos.markers[array].points[i].y
					x = self.loaded_map_info_rbkairos.markers[array].points[i].x
					if x < (self.xmin_rbkairos) or x > (self.xmax_rbkairos) or y < (self.ymin_rbkairos) or y > (self.ymax_rbkairos) or z< (self.zmin_rbkairos) or z > (self.zmax_rbkairos):
						anomoly_count_range2+=1
						item = ("index", i, x,y,z)
						anomoly_list_range2.append(item)

				#######################
				# Remove Anomolies
				#######################
				for i in range(0,len(anomoly_list_range2)):
					index = anomoly_list_range2[i][1]-i
					del self.loaded_map_info_rbkairos.markers[array].points[index]
					del self.loaded_map_info_rbkairos.markers[array].colors[index]

				# Save Data
				#print("SCRUBBED 1!")
				#rospy.logdebug("Scrubbed Length %s %s", str(array),str(len(self.loaded_map_info_rbkairos.markers[array].points)))

			if len(self.final_map_info_rbkairos.markers) == 0:
				self.final_map_info_rbkairos.markers = self.loaded_map_info_rbkairos.markers


		##############################################
		# Map Align Part I: Elevation Data Reorganization 
		##############################################
			### 1. Get elevation data. How many curves working with. 
			# Voxel list 
			voxel_list2 = []
			# Elevation data 
			z_counter2 = 0
			z_level2 = []
			z_data2 = []
			for i in range(0,len(self.loaded_map_info_rbkairos.markers[1].points)):
				x = round(self.loaded_map_info_rbkairos.markers[1].points[i].x,3)
				y = round(self.loaded_map_info_rbkairos.markers[1].points[i].y,3)
				z = round(self.loaded_map_info_rbkairos.markers[1].points[i].z,3)
				# Save unique elevation data, least to greatest. 
				if z not in z_data2:
					z_counter2+=1
					z_level2.append(z)
					z_level2.sort()
				# Save Data
				z_data2.append(z)
				voxel_list2.append((x,y,z))

			# Make List to store values for contour fitting.  
			contour_data2 = []
			for i in range(0,z_counter2):
				contour_data2.append((z_level2[i],[[],[]]))

			rospy.logdebug("Z_count rbkairos %s", str(z_counter2))
			rospy.logdebug("Z_level rbkairos %s", str(z_level2))

			### 2. Only the topmost curve matters, but will curvefit all curves for visualization. 
			# Resort Data for Curve Fitting
			for i in range(0,len(voxel_list2)):
				# Check for elevation match. Save x,y if found. 
				for level in range(0,len(contour_data2)):
					if voxel_list2[i][2] == contour_data2[level][0]:
						contour_data2[level][1][0].append(voxel_list2[i][0])
						contour_data2[level][1][1].append(voxel_list2[i][1])
						break

			####self.feature_finder(data=contour_data2,map_name="Rbkairos")

			# Save Data for plotting 
			self.contour_data[1] = contour_data2

##############################################
# Map Align Part II: Countour Map Creation
##############################################
	def feature_finder(self,data,map_name="turtlebot", visualize=False,lower_h_limit=7):
		# ONLY Feature extract if NOT visualizing. 
		# IF visualizing, then NOT appending data to feature extraction list.

			# Containers to determine the ellipse dimensions
			a_list = []
			b_list = []
			ell_center = []
			index_list = []
			# Getting color gradient for visualization
			red = Color("red")
			colors=list(red.range_to(Color("green"),(len(data)+1)))
			#print("Colors", colors)
			#print("LENGTH COMP. Color", len(colors)-2, len(data)-1)
			#print(str(colors[1]))

			# Pull in contour data 
			for h in range(lower_h_limit,len(data)): # Step = 4
				x_data = np.array(data[h][1][0])
				y_data = np.array(data[h][1][1])
				# Plotting an ellipse for each. 
				# Need params for each ellipse. 
				v = self.fit_ellipse(x_data,y_data)
				ellipse_data = self.polyToParams(v)
				#######################################
				#print("ELLIPSE DATA")
				#print("Center at: ",ellipse_data[0],ellipse_data[1])
				#print("Axes Gains are a: ", ellipse_data[2]," | b: ", ellipse_data[3])
				#print("Tilt is: ",ellipse_data[4]," degrees")
				########################################
				## Get the arrays needed to plot the ellipse.
				t = np.linspace(0,2*3.14,100)
				Ell = np.array([ellipse_data[2]*np.cos(t),ellipse_data[3]*np.sin(t)])

				Ell_rot = np.zeros((2,Ell.shape[1]))
				for i in range(Ell.shape[1]):
					Ell_rot[:,i]=np.dot(ellipse_data[5],Ell[:,i])
				########################################
				if visualize:
					## Plot
					plt.figure(1)
					self.plot = plt.plot(ellipse_data[0]+Ell[0,:],ellipse_data[1]+Ell[1,:],color=str(colors[h-1]))
					plt.scatter(x_data,y_data,color=str(colors[h-1]))
					#plt.scatter(ellipse_data[0],ellipse_data[1],color=str(colors[h-1]))
					#########
					#plt.plot(ellipse_data[0]+Ell_rot[0,:],ellipse_data[1]+Ell_rot[1,:],'darkorange')
					#########

					plt.grid()
					plt.axis("equal")
					plt.xlim(0,11.0)
					plt.ylim(-2.0,9.0)

				# Save Data 
				a_list.append(ellipse_data[2])
				b_list.append(ellipse_data[3])
				ell_center.append((ellipse_data[0],ellipse_data[1]))
				index_list.append(h)

##############################################
# Map Align Part III: Countour Map Feature Creation
##############################################
			# Compare Ellipse dimensions
			# a = x offset. b = y offset.
			# The larger one indicates an elongation of the blade
			# in that dimension. Save extreme points on the ellipse
			# using the larger "a" or "b" value. 
			x1_list = []
			x2_list = []
			y1_list = []
			y2_list = []
			stretch_in = 0
			for i in range(0,len(index_list)):
				a = a_list[i]
				b = b_list[i]
				# If a > b, x is elongated. Use that for feature extraction. 
				if a > b:
					x1 = ell_center[i][0]+a
					x2 = ell_center[i][0]-a
					y1 = ell_center[i][1]
					y2 = y1
					stretch_in = "a"
				# else, y is elongated
				else:
					x1 = ell_center[i][0]
					x2 = x1
					y1 = ell_center[i][1]+b
					y2 = ell_center[i][1]-b
					stretch_in = "b"

				# Save Data 
				x1_list.append(x1)
				x2_list.append(x2)
				y1_list.append(y1)
				y2_list.append(y2)

			# Determine which side of the blade is which. 
			# The lowest std deviation will be the side at the windblade 
			# Handle. This can be seen when plotted.  
			x1_std,x2_std,y1_std,y2_std = np.std(x1_list),np.std(x2_list),np.std(y1_list),np.std(y2_list)
			# Only care about std deviation in longer part. std deviation for shorter will be the same for both sets of points. 
			if stretch_in == "b":
				std_list = [y1_std,y2_std]
			else:
				std_list = [x1_std,x2_std]

			#print("STD LIST", std_list)
			std_list_min = min(std_list)
			std_list_min_ind=std_list.index(std_list_min)
			#print("LOWEST INDEX",std_list_min_ind)

			# Use index to determine if going to add or substract "a" or "b". 
			# IF the index == 0, ADD a or b. if index == 1 SUBTRACT "a" or "b".
			# This is for the windblade handle. For the opposite, it'd be the
			# opposite. 
			### Save feature point list 
			feature_list = []
			feature_list_offset = []
			# How much offset inwards the second feature point should be.
			# This will be used to align initial rotations. 
			offset = 0.02
			for i in range(0,len(index_list)):
				a = a_list[i]
				b = b_list[i]
				#///////////
				if stretch_in == "b":
					# Same x 
					x = ell_center[i][0]
					if std_list_min_ind == 0:
						y = ell_center[i][1]+b
						y_off = ell_center[i][1]+b-offset
					else:
						y = ell_center[i][1]-b
						y_off = ell_center[i][1]-b+offset

					if visualize:
						plt.scatter(x,y_off,color="blue")
					else:
						# Append to feature list offset
						feature_list_offset.append([x,y_off,0])
				else:
					# Same y 
					#rospy.loginfo("THE a %s at i %s for Map %s",str(a),str(i), str(map_name))
					y = ell_center[i][1]
					if std_list_min_ind == 0:
						x = ell_center[i][0]+a
						x_off = ell_center[i][0]+a-offset
					else:
						x = ell_center[i][0]-a
						x_off = ell_center[i][0]-a+offset

					if visualize:
						plt.scatter(x_off,y,color="blue")
					else:
						# Append to feature list offset
						feature_list_offset.append([x_off,y,0])
				#///////////

				#///////////
				if visualize:
					plt.scatter(x,y,color="blue")
				else:
					# Append to feature list 
					feature_list.append([x,y,0])
				#///////////

			# If visualizing, then will not be appending data to be used for alignment.
			# Otherwise, appending data. 
			if not visualize:
				# Pull back in the height references to each reference point. 
				for i in range(0,len(index_list)):
					# Round to 3 decimals.
					feature_list[i][0]=round(feature_list[i][0],3)
					feature_list[i][1]=round(feature_list[i][1],3)
					feature_list[i][2]=data[index_list[i]][0]

					feature_list_offset[i][0]=round(feature_list_offset[i][0],3)
					feature_list_offset[i][1]=round(feature_list_offset[i][1],3)
					feature_list_offset[i][2]=data[index_list[i]][0]

				# AT this point, can select any countour. 
				# Going to select the last one (The highest elevated one) from the list.
				self.features.append([map_name,feature_list[-1],feature_list_offset[-1]])

				## Find the nearest voxels to these points.
				# Select newest added features.
				voxel_size = 0.01
				for i in range(1,3):
					voxel_x = round(round(self.features[-1][i][0]/voxel_size,0)*voxel_size,3)
					voxel_y = round(round(self.features[-1][i][1]/voxel_size,0)*voxel_size,3)
					self.features[-1][i][0] = voxel_x
					self.features[-1][i][1] = voxel_y

				rospy.loginfo("Features %s",str(self.features))
			
			# For external use if visualizing.
			else:
				# Visualized?
				return True

#############################
# Ellipse Curve Fitting (USED in Alignment PART II)
# Taken from: ADD SOURCE
############################
	def fit_ellipse(self,x,y):
		### ADD SOURCE 
		# Increase dimension by 1
		x = x[:,np.newaxis]
		y = y[:,np.newaxis]

		J = np.hstack((x*x, x*y, y*y, x, y))
		K = np.ones_like(x) # column of ones

		JT=J.transpose()
		JTJ=np.dot(JT,J)
		InvJTJ=np.linalg.inv(JTJ)
		ABC = np.dot(InvJTJ,np.dot(JT,K))

		# ABC has polynomial coeffs A,B,C,D,E
		# A x^2 + B xy + C y^2 + Dx + Ey - F = 0
		# Here, F = -1
		eansa=np.append(ABC,-1)
		return eansa
	
	def polyToParams(self,v):
		# Converting the polynomial form of ellipse to usable params. 
		Amat= np.array(
			[
				[v[0],    v[1]/2.0,v[3]/2.0],
				[v[1]/2.0,v[2],    v[4]/2.0],
				[v[3]/2.0,v[4]/2.0,v[5]]
			])
		# Finding the Center of the circle 
		A2=Amat[0:2,0:2]
		A2Inv=np.linalg.inv(A2)
		ofs=v[3:5]/2.0
		cc=-np.dot(A2Inv,ofs)

		# Center Ellipse at origin.
		Tofs = np.eye(3)
		Tofs[2,0:2]=cc
		R=np.dot(Tofs,np.dot(Amat,Tofs.T))

		R2 = R[0:2,0:2]
		s1=-R[2,2]
		RS=R2/s1
		(e1,ec)=np.linalg.eig(RS)

		recip=1.0/np.abs(e1)
		axes=np.sqrt(recip)

		rads=np.arctan2(ec[1,0],ec[0,0])
		deg=np.degrees(rads)

		inve=np.linalg.inv(ec) # inverse is actually the transpose

		return(cc[0],cc[1],axes[0],axes[1],deg,inve)

##############################################
# Map Align Part IV: Map Alignment
##############################################
	def align_maps(self):
			if len(self.final_map_info_rbkairos.markers) > 0 and len(self.final_map_info_turtle.markers) > 0:
			
			# 1. Combine voxels will be using into 1 full map. 
				#rospy.loginfo(self.final_map_info_turtle.markers)
				self.fullmerge.markers = self.final_map_info_turtle.markers
				for i in range (0,len(self.final_map_info_rbkairos.markers)):
					self.fullmerge.markers.append(self.final_map_info_rbkairos.markers[i])

			# 2. Search for Translation Offset
				#rospy.loginfo(".........The Features are: %s",str(self.features))
				# TRANSLATING the RbKairos Outermost point [1][2] to the Turtle [0][2]
				map_feature_1 = self.features[0][2]
				moving_feature_1 = self.features[1][2]
				# Get offset
				dx = round(map_feature_1[0]-moving_feature_1[0],3)
				dy = round(map_feature_1[1]-moving_feature_1[1],3)

				# APPLY Offset
				#rospy.loginfo("ALIGNMENT Rbkairos point BEFORE Translation %s", str(self.fullmerge.markers[3].points[0]))

				for i in range(2,4):
					for point in range(0,len(self.fullmerge.markers[i].points)):
						x = self.fullmerge.markers[i].points[point].x
						x+=dx
						self.fullmerge.markers[i].points[point].x = x
						y = self.fullmerge.markers[i].points[point].y
						y+=dy
						self.fullmerge.markers[i].points[point].y = y

				#rospy.loginfo("ALIGNMENT Rbkairos point AFTER Translation %s", str(self.fullmerge.markers[3].points[0]))

			# 3. Search for Rotation Offset
				# ROTATING the RbKairos Outermost point [1][1] to the Turtle [0][1]
				map_feature_2 = self.features[0][1]
				moving_feature_2 = self.features[1][1]
				## Need to apply offset to moving features too. 
				#print("ROTATE FEATURES BEFORE", map_feature_2,moving_feature_2)
				moving_feature_2[0]= round(moving_feature_2[0]+dx,3)
				moving_feature_2[1]= round(moving_feature_2[1]+dy,3)
				print("ROTATE FEATURES AFTER", map_feature_2,moving_feature_2)

				## Need to Determine the angle offset to get from the Rbkairos point to the 
				## Turtle point. 
				# Rotation is about map_feature_1 = self.features[0][2]

				x_map = round(map_feature_1[0]-map_feature_2[0],8)
				y_map = round(map_feature_1[1]-map_feature_2[1],8)
				x_moving = round(map_feature_1[0]-moving_feature_2[0],8)
				y_moving = round(map_feature_1[1]-moving_feature_2[1],8)
				print("NEW ROT XY: xmap",x_map,"y map",y_map,"x moving",x_moving,"y moving",y_moving)

				angle_map = math.atan2(y_map,x_map)
				angle_moving = math.atan2(y_moving,x_moving)
				angle_diff = angle_moving-angle_map
				print("THE ANGLES ARE ........Map", angle_map, "Moving", angle_moving, "Angle Diff", angle_diff)

				## Apply this offset to each point in Rbkairos Map 
				for i in range(2,4):
					for point in range(0,len(self.fullmerge.markers[i].points)):
						x = self.fullmerge.markers[i].points[point].x
						y = self.fullmerge.markers[i].points[point].y
						# The offset from the point of rotation to the given point. 
						dx_point = round(map_feature_1[0]-x,8)
						dy_point = round(map_feature_1[1]-y,8)
						# The angle and radius 
						r = math.sqrt(dx_point**2+dy_point**2)
						angle_point = math.atan2(dy_point,dx_point)
						# The new angle and coordinates relative to the rotation point. 
						new_angle = angle_point+angle_diff
						x = r*math.cos(new_angle)
						y = r*math.sin(new_angle)
						# New angle coordinates relative to the world frame. Undo the first subtractions.
						x = round(map_feature_1[0]+x,3)
						y = round(map_feature_1[1]+y,3)

						# Save new point data  
						self.fullmerge.markers[i].points[point].x = x
						self.fullmerge.markers[i].points[point].y = y

				self.aligned = True

##############################################
# Map Align Part V: Map Resclaing 
##############################################
	# Need this to ensure that the merge process works right. 
	def map_rescaling(self):
		# RESCALE values to be PERFECTLY in grid spaces. 
		for i in range(0,len(self.fullmerge.markers)):
			# ASSUMES CUBE voxels.
			voxel_size = self.fullmerge.markers[i].scale.x
			for point in range(0,len(self.fullmerge.markers[i].points)):
				self.fullmerge.markers[i].points[point].x = round(round(self.fullmerge.markers[i].points[point].x/voxel_size,1)*voxel_size,3)
				self.fullmerge.markers[i].points[point].y = round(round(self.fullmerge.markers[i].points[point].y/voxel_size,1)*voxel_size,3)
				self.fullmerge.markers[i].points[point].z = round(round(self.fullmerge.markers[i].points[point].z/voxel_size,1)*voxel_size,3)

		self.map_rescaled = True

##############################################
# Map Align Part VI: Initial Map Merge
##############################################
	def map_merge(self):
		# Check if exist 
		if len(self.final_map_info_rbkairos.markers) > 0 and len(self.final_map_info_turtle.markers) > 0:
			'''
			# Debug Some Stuff.
			info_size_list2 = []
			for i in range(0,len(self.fullmerge.markers)):
				item = (i, len(self.fullmerge.markers[i].points), len(self.fullmerge.markers[i].colors))
				info_size_list2.append(item)
				#rospy.loginfo("Rbkairos %s",str(i))
			rospy.loginfo("Rbkairos List %s",str(info_size_list2))
			rospy.loginfo("Merge MAP LENGTH %s",str(len(self.fullmerge.markers)))
			'''

			# Redefine Some Stuff 
			for i in range(0,len(self.fullmerge.markers)):
				#print(self.fullmerge.markers[i].color)
				self.fullmerge.markers[i].id = i
				if i in [0,1]:
					# Turtlebot Stuff 
					self.fullmerge.markers[i].color.r = 1.0
					self.fullmerge.markers[i].color.g = 0.0
					self.fullmerge.markers[i].color.b = 0.0
					self.fullmerge.markers[i].color.a = 1.0
				else:
					# RbKairos Stuff 
					self.fullmerge.markers[i].color.r = 0.0
					self.fullmerge.markers[i].color.g = 1.0
					self.fullmerge.markers[i].color.b = 0.0
					self.fullmerge.markers[i].color.a = 1.0

					'''
					# The Values don't update, so each voxel needs to be updated manually 
					# For map alignment purposes. 
					rospy.loginfo("Rbkairos point BEFORE Translation %s", str(self.fullmerge.markers[i].points[0]))
					self.fullmerge.markers[i].pose.position.x = 10
					rospy.loginfo("Rbkairos point AFTER Translation %s", str(self.fullmerge.markers[i].points[0]))
					'''

			# Prevent infinite concatenation. 
			self.merged = True

###################################
###################################

# Dynamic Reconfigure 
def reconfigure(config,level):
	rospy.logwarn("Reconfigured!")
	rospy.loginfo(config)
	return config

###################################
###################################

if __name__ == '__main__':
	rospy.init_node("map_alignment",anonymous=True, log_level=rospy.DEBUG)
	# If plot Visualize - True, need to call it in the main loop.
	alignment = MapAlignment(plot_visualize=True)

	if alignment.plot_visualize:
		visualized_map=False

	while not rospy.is_shutdown():
		try:
			#alignment.align_pub1.publish(alignment.final_map_info_turtle)
			#alignment.align_pub2.publish(alignment.final_map_info_rbkairos)

			# Wait for appropriate data... Will only be true once the robot is near the set waypoint.
			if len(alignment.contour_data[0]) > 0 and len(alignment.contour_data[1]) > 0:
				# Call the Feature Finders ONCE...
				if not alignment.features_found:
					alignment.feature_finder(data=alignment.contour_data[0],map_name="Turtle")
					alignment.feature_finder(data=alignment.contour_data[1],map_name="Rbkairos")
					alignment.features_found = True
				# Call the Map Alignment ONCE....
				if not alignment.aligned:
					alignment.align_maps()
				# Call the map verification ONCE...
				if not alignment.map_rescaled:
					alignment.map_rescaling()
				# Call the map Merging ONCE After maps aligned and verified
				if not alignment.merged and alignment.align_maps and alignment.map_rescaled:
					alignment.map_merge()
					
				# If merge successful:
				if alignment.merged:
					alignment.align_pub.publish(alignment.fullmerge)
					# Show plots if visualization enabled. 
					if alignment.plot_visualize:
						if not visualized_map:
							for i in range(0,len(alignment.contour_data)):
								alignment.feature_finder(data=alignment.contour_data[i],map_name="merged",visualize=True)
							visualized_map = True
							#print("ALIGNMENT LENGTH",len(alignment.contour_data))

						plt.show(alignment.plot)

				# Dynamic Reconfigure 
				#srv = Server(rbkairos_alignmentConfig, reconfigure)
				#converter.debugger()
			else:
				rospy.loginfo_once("Waiting for contour data.")
				if len(alignment.contour_data[0]) == 0 and len(alignment.contour_data[0]) == 0:
					rospy.loginfo_once("Length Data 1: %s | Length Data 2: %s", str(len(alignment.contour_data[0])),str(len(alignment.contour_data[1])))
				elif len(alignment.contour_data[0]) == 1:
					rospy.loginfo_once("Length Data 1: %s | Length Data 2: %s", str(len(alignment.contour_data[0])),str(len(alignment.contour_data[1])))
				elif len(alignment.contour_data[1]) == 1:
					rospy.loginfo_once("Length Data 1: %s | Length Data 2: %s", str(len(alignment.contour_data[0])),str(len(alignment.contour_data[1])))


		except rospy.ROSInterruptException:
			rospy.logfatal(rospy.ROSInterruptException)