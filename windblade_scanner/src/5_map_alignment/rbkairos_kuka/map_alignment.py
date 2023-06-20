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
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

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

	def __init__(self, loop_rate=5.0, plot_visualize = False, analyze=False):

		# Safety 
		self.rate = rospy.Rate(loop_rate)

		# Time stamp for both
		self.time_stamp = rospy.Time.now()

        # Define topics 
		self.node_name = "map_alignment/"
		self.reference_map = str(rospy.get_param(self.node_name + "reference_map"))
		self.incoming_map = str(rospy.get_param(self.node_name + "incoming_map"))
		self.odom = str(rospy.get_param(self.node_name + "odom_topic"))

		# Waypoint Check Data 
		self.waypoint_x = float(rospy.get_param(self.node_name + "waypoint_x"))
		self.waypoint_y = float(rospy.get_param(self.node_name + "waypoint_y"))
		self.waypoint_tol = float(rospy.get_param(self.node_name + "waypoint_tol"))

		# Turtlemap filter limits
		self.xmin_reference = float(rospy.get_param(self.node_name + "xmin_reference"))
		self.xmax_reference = float(rospy.get_param(self.node_name + "xmax_reference"))
		self.ymin_reference = float(rospy.get_param(self.node_name + "ymin_reference"))
		self.ymax_reference = float(rospy.get_param(self.node_name + "ymax_reference"))
		self.zmin_reference = float(rospy.get_param(self.node_name + "zmin_reference"))
		self.zmax_reference = float(rospy.get_param(self.node_name + "zmax_reference"))

		# Rbkairos filter limits
		self.xmin_incoming = float(rospy.get_param(self.node_name + "xmin_incoming"))
		self.xmax_incoming = float(rospy.get_param(self.node_name + "xmax_incoming"))
		self.ymin_incoming = float(rospy.get_param(self.node_name + "ymin_incoming"))
		self.ymax_incoming = float(rospy.get_param(self.node_name + "ymax_incoming"))
		self.zmin_incoming = float(rospy.get_param(self.node_name + "zmin_incoming"))
		self.zmax_incoming = float(rospy.get_param(self.node_name + "zmax_incoming"))

		# Subscribers, publishers 																								
		self.align_sub_reference_map=rospy.Subscriber(self.reference_map, MarkerArray, self.reference_map_scrubber)
		# Loaded Map 
		self.align_sub_incoming_map=rospy.Subscriber(self.incoming_map, MarkerArray, self.incoming_map_scrubber)
		# For waypoint checking.Will align when at x, y, coordinates. 
		self.odom_sub=rospy.Subscriber(self.odom, Odometry,self.waypoint_checker)


		self.align_pub1 = rospy.Publisher("/final_map_info_ref",MarkerArray, queue_size=1)
		self.align_pub2 = rospy.Publisher("/final_map_info_inc",MarkerArray, queue_size=1)
		self.align_pub = rospy.Publisher("/occupied_cells_map_scrubbed_merged",MarkerArray, queue_size=1)

		# Msgs
		self.loaded_map_info_reference = MarkerArray()
		self.final_map_info_reference = MarkerArray()
		self.loaded_map_info_incoming = MarkerArray()
		self.final_map_info_incoming = MarkerArray()
		# Fullmerge
		self.fullmerge=MarkerArray()
		# TEST
		self.final_merge = MarkerArray()

	## Waypoints 
	# DEBUG. SETTING TO TRUE. Normally False When robot following path. 
		self.at_waypoint = True

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
		self.reference_map_z_levels = []
		self.aligned = False
		self.map_rescaled = False

	## Merging 
		self.merged = False

	##Data Analysis
		# Do data analysis?
		if analyze == False: 
			self.analyzed = True
		else:
			self.analyzed = False
		# Ellipse pt containers. 
		self.data_map_analysis = []
		self.data_aligned_analysis = []

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
			#rospy.loginfo("x Position: %s. Want between %s and %s |  y Position: %s. Want between %s and %s", str(x_pos),str(round(self.waypoint_x-self.waypoint_tol,3)), str(round(self.waypoint_x+self.waypoint_tol,3)),str(y_pos),str(round(self.waypoint_y-self.waypoint_tol,3)),str(round(self.waypoint_y+self.waypoint_tol,3)))

			if x_pos >= self.waypoint_x-self.waypoint_tol and x_pos <= self.waypoint_x+self.waypoint_tol:
			    if y_pos >= self.waypoint_y-self.waypoint_tol and y_pos <= self.waypoint_y+self.waypoint_tol:
					self.at_waypoint = True

##########################
### REFERENCE (Turtle) scrubber
##########################
	def reference_map_scrubber(self, msg):
		# ONLY want 1 MSG. 
		rospy.loginfo("LOADED Reference Map")
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
			#rospy.loginfo("Turtle List %s",str(info_size_list1))
			'''

			# Save Raw Data
			self.loaded_map_info_reference.markers = [loaded_info_array1[15],loaded_info_array1[16]]
			rospy.logdebug("REFERENCE | Loaded Data length 15: %s | 16: %s",str(len(self.loaded_map_info_reference.markers[0].points)),str(len(self.loaded_map_info_reference.markers[1].points)))

			############################
			# RANGE FILTER
			############################

			for array in range(0,len(self.loaded_map_info_reference.markers)):
				anomoly_count_range1 = 0
				anomoly_list_range1 = []

				list1 = []

				# Range Filter
				for i in range(0,len(self.loaded_map_info_reference.markers[array].points)):
					z = self.loaded_map_info_reference.markers[array].points[i].z
					y = self.loaded_map_info_reference.markers[array].points[i].y
					x = self.loaded_map_info_reference.markers[array].points[i].x
					if x < (self.xmin_reference) or x > (self.xmax_reference) or y < (self.ymin_reference) or y > (self.ymax_reference) or z< (self.zmin_reference) or z > (self.zmax_reference):
						'''
						if x < (self.xmin_reference) or x > (self.xmax_reference):
							list1.append((x,y,z,"x viol", "xmin:",self.xmin_reference, "xmax:", self.xmax_reference))
						elif y < (self.ymin_reference) or y > (self.ymax_reference):
							list1.append((x,y,z, "y viol", "ymin:",self.ymin_reference, "ymax:", self.ymax_reference))
						else:
							list1.append((x,y,z, "z viol","zmin:",self.zmin_reference, "zmax:", self.zmax_reference))
						'''
						anomoly_count_range1+=1
						item = ("index", i, x,y,z)
						anomoly_list_range1.append(item)

				#######################
				# Remove voxels out of range
				#######################
				for i in range(0,len(anomoly_list_range1)):
					index = anomoly_list_range1[i][1]-i
					del self.loaded_map_info_reference.markers[array].points[index]
					del self.loaded_map_info_reference.markers[array].colors[index]
				rospy.logdebug("REFERENCE | Scrubbed Length for level %s of the REFERENCE map is: %s", str(array),str(len(self.loaded_map_info_reference.markers[array].points)))
			
			# Save Data
			self.final_map_info_reference.markers = self.loaded_map_info_reference.markers

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
			for i in range(0,len(self.loaded_map_info_reference.markers[1].points)):
				# Eliminate extraneous decimals
				x = round(self.loaded_map_info_reference.markers[1].points[i].x,3)
				y = round(self.loaded_map_info_reference.markers[1].points[i].y,3)
				z = round(self.loaded_map_info_reference.markers[1].points[i].z,3)
				# Save unique elevation data, least to greatest. 
				if z not in z_data1:
					z_counter1+=1
					z_level1.append(z)
					z_level1.sort()
				# Save Data
				z_data1.append(z)
				voxel_list1.append((x,y,z))

		###### Store the elevation data for reference map. 
			self.reference_map_z_levels = z_data1

			# Make List to store values for contour fitting.  
			contour_data1 = []
			for i in range(0,z_counter1):
				contour_data1.append((z_level1[i],[[],[]]))

			rospy.logdebug("REFERENCE | Voxel elevation count for the REFERENCE map is: %s", str(z_counter1))
			rospy.logdebug("REFERENCE | Voxel elevations for the REFERENCE map are: %s", str(z_level1))

			# Resort Data for Curve Fitting based on voxel grid elevation.
			for i in range(0,len(voxel_list1)):
				# Check for elevation match. Save x,y if found. 
				for level in range(0,len(contour_data1)):
					if voxel_list1[i][2] == contour_data1[level][0]:
						contour_data1[level][1][0].append(voxel_list1[i][0])
						contour_data1[level][1][1].append(voxel_list1[i][1])
						break

			# Save Data for plotting 
			self.contour_data[0] = contour_data1

###########################
### INCOMING (Kuka Base) scrubber
###########################
	def incoming_map_scrubber(self, msg):
		rospy.loginfo("LOADED incoming map.")
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
			rospy.logdebug("Rbkairos List %s",str(info_size_list2))
			'''
			# Save Raw Data
			self.loaded_map_info_incoming.markers = [loaded_info_array2[15],loaded_info_array2[16]]
			rospy.logdebug("INCOMING | Loaded Data length 15: %s | 16: %s",str(len(self.loaded_map_info_incoming.markers[0].points)),str(len(self.loaded_map_info_incoming.markers[1].points)))

			############################
			# RANGE FILTER
			############################

			for array in range(0,len(self.loaded_map_info_incoming.markers)):
				#rospy.logdebug("Incoming scrub loop iteration %s", str(array))
				anomoly_count_range2 = 0
				anomoly_list_range2 = []

				# Range Filter
				for i in range(0,len(self.loaded_map_info_incoming.markers[array].points)):
					z = self.loaded_map_info_incoming.markers[array].points[i].z
					y = self.loaded_map_info_incoming.markers[array].points[i].y
					x = self.loaded_map_info_incoming.markers[array].points[i].x
					if x < (self.xmin_incoming) or x > (self.xmax_incoming) or y < (self.ymin_incoming) or y > (self.ymax_incoming) or z< (self.zmin_incoming) or z > (self.zmax_incoming):
						anomoly_count_range2+=1
						item = ("index", i, x,y,z)
						anomoly_list_range2.append(item)

				#######################
				# Remove Anomolies
				#######################
				for i in range(0,len(anomoly_list_range2)):
					index = anomoly_list_range2[i][1]-i
					del self.loaded_map_info_incoming.markers[array].points[index]
					del self.loaded_map_info_incoming.markers[array].colors[index]

				rospy.logdebug("INCOMING | Scrubbed Length for level %s of the INCOMING map is: %s", str(array),str(len(self.loaded_map_info_incoming.markers[array].points)))

			# Save Data
			self.final_map_info_incoming.markers = self.loaded_map_info_incoming.markers


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
			for i in range(0,len(self.loaded_map_info_incoming.markers[1].points)):
				# Eliminate extraneous decimals
				x = round(self.loaded_map_info_incoming.markers[1].points[i].x,3)
				y = round(self.loaded_map_info_incoming.markers[1].points[i].y,3)
				z = round(self.loaded_map_info_incoming.markers[1].points[i].z,3)
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

			rospy.logdebug("INCOMING | Voxel elevation count for the INCOMING map is: %s", str(z_counter2))
			rospy.logdebug("INCOMING | Voxel elevations for the INCOMING map are: %s", str(z_level2))

			# Resort Data for Curve Fitting
			for i in range(0,len(voxel_list2)):
				# Check for elevation match. Save x,y if found. 
				for level in range(0,len(contour_data2)):
					if voxel_list2[i][2] == contour_data2[level][0]:
						contour_data2[level][1][0].append(voxel_list2[i][0])
						contour_data2[level][1][1].append(voxel_list2[i][1])
						break

			# Save Data for plotting 
			self.contour_data[1] = contour_data2

##############################################
# Map Align Part II: Countour Map Creation
##############################################
	def feature_finder(self,data,map_name="map", visualize=False,lower_h_limit=7, upper_h_limit = 15, fig = 1):
		rospy.loginfo("%s map feature searching | Lower elevation limit: %s | Upper limit: %s",str(map_name).upper(),str(lower_h_limit),str(upper_h_limit))
		if upper_h_limit > len(data):
			rospy.logwarn("Length of elevation data is %s. The entered upper limit is %s, which is higher. Defaulting to the highest elevation.", str(len(data)), str(upper_h_limit))
			upper_h_limit = len(data)

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
		
		# Pull in contour data 
		for h in range(lower_h_limit,upper_h_limit): # Step = 4
			x_data = np.array(data[h][1][0])
			y_data = np.array(data[h][1][1])
			rospy.logdebug("%s map DEBUG | x_data length %s | y_data length %s",str(map_name).upper(),str(len(x_data)),str(len(y_data)))
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
			'''
			## Get the arrays needed to plot the ellipse.
			t = np.linspace(0,2*3.14,100)
			Ell = np.array([ellipse_data[2]*np.cos(t),ellipse_data[3]*np.sin(t)])
			rospy.logdebug("%s map DEBUG | Ellipse array length %s",str(map_name).upper(),str(Ell))

			########################################
			# WIP. Need to fix this. 
			#Ell_rot = np.zeros((2,Ell.shape[1]))
			#for i in range(Ell.shape[1]):
			#	Ell_rot[:,i]=np.dot(ellipse_data[5],Ell[:,i])
			########################################
			'''
			if visualize:
				t = np.linspace(0,2*3.14,100)
				Ell = np.array([ellipse_data[2]*np.cos(t),ellipse_data[3]*np.sin(t)])
				## Plot
				plt.figure(fig)
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

		rospy.logwarn("||||||||||||||||||||||")
		rospy.logdebug("%s map DEBUG | a_list %s",str(map_name).upper(),str(len(a_list)))
		rospy.logdebug("%s map DEBUG | b_list %s",str(map_name).upper(),str(len(b_list)))
		rospy.logdebug("%s map DEBUG | ellipse center %s",str(map_name).upper(),str(len(ell_center)))
		rospy.logdebug("%s map DEBUG | index_list %s",str(map_name).upper(),str(len(index_list)))
		rospy.logwarn("||||||||||||||||||||||")

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
				rospy.logdebug("%s map DEBUG | stretch in a DIR.",str(map_name).upper())
			# else, y is elongated
			else:
				x1 = ell_center[i][0]
				x2 = x1
				y1 = ell_center[i][1]+b
				y2 = ell_center[i][1]-b
				stretch_in = "b"
				rospy.logdebug("%s map DEBUG | stretch in b DIR.",str(map_name).upper())

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

		std_list_min = min(std_list)
		std_list_min_ind=std_list.index(std_list_min)

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
				#rospy.logdebug("THE a %s at i %s for Map %s",str(a),str(i), str(map_name))
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

		rospy.loginfo("%s map DEBUG | Visualizing %s",str(map_name).upper(), str(visualize))
		
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
			if len(feature_list) == 0 or len(feature_list_offset) == 0:
				rospy.logfatal("NO FEATURES FOUND. Length of feature list is %s. Length of feature offset is %s.",str(len(feature_list)),str(len(feature_list_offset)))
			else:
				self.features.append([map_name,feature_list[-1],feature_list_offset[-1]])

				## Find the nearest voxels to these points.
				# Select newest added features.
				voxel_size = 0.01
				for i in range(1,3):
					voxel_x = round(round(self.features[-1][i][0]/voxel_size,0)*voxel_size,3)
					voxel_y = round(round(self.features[-1][i][1]/voxel_size,0)*voxel_size,3)
					self.features[-1][i][0] = voxel_x
					self.features[-1][i][1] = voxel_y
			
				rospy.loginfo("Features found for %s map.",str(map_name).upper())

		# For external use if visualizing.
		else:
			# Visualized?
			return True
		
#############################
# Ellipse Curve Fitting (USED in Alignment PART II)
# Taken and Modified from: http://juddzone.com/ALGORITHMS/least_squares_ellipse.html
############################
	def fit_ellipse(self,x,y):
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
# Map Align Part IVa: Full Map Concatenation
##############################################
	def map_concatenated(self,num_maps=2):

		# Get Unique Resolution Sizes (Assume cubic voxels. Assume same dimensions.)
		res_list = []
		for i in range(0,len(self.final_map_info_reference.markers)):
			if self.final_map_info_reference.markers[i].scale.x not in res_list:
				res_list.append(self.final_map_info_reference.markers[i].scale.x)
			if self.loaded_map_info_reference.markers[i].scale.x not in res_list:
				res_list.append(self.loaded_map_info_reference.markers[i].scale.x)
		rospy.logdebug("map_concatenated DEBUG | RES_LIST REALIGNED contents: %s", str(res_list))

		# Make the merged marker array. Based on number of maps merging. 
		for i in range(0,int(len(self.final_map_info_reference.markers))*num_maps):
			self.fullmerge.markers.append(Marker())
		rospy.logdebug("map_concatenated DEBUG | MERGED array REALIGNED LENGTH: %s", str(len(self.fullmerge.markers)))

		# Create fullmerge marker array
		for i in range(0,len(self.fullmerge.markers)):
			########
			# Set Header 
			self.fullmerge.markers[i].header.seq=0
			self.fullmerge.markers[i].header.stamp=rospy.Time.now()
			self.fullmerge.markers[i].header.frame_id="world"

			# Namespace
			self.fullmerge.markers[i].ns = self.final_map_info_reference.markers[0].ns
			# Set id 
			self.fullmerge.markers[i].id = i
			# Type is voxel
			self.fullmerge.markers[i].type = 6
			# Action is add 
			self.fullmerge.markers[i].action = 0
			# Position is original 
			self.fullmerge.markers[i].pose.position.x = 0.0
			self.fullmerge.markers[i].pose.position.y = 0.0
			self.fullmerge.markers[i].pose.position.z = 0.0
			# Orientation is original 
			self.fullmerge.markers[i].pose.orientation.x = 0.0
			self.fullmerge.markers[i].pose.orientation.y = 0.0
			self.fullmerge.markers[i].pose.orientation.z = 0.0
			self.fullmerge.markers[i].pose.orientation.w = 1.0

			# Copy scalars
			if i in [0,2]:
				#rospy.logdebug("ITEM is %s for %s", str(res_list[0]), str(i))
				self.fullmerge.markers[i].scale.x = res_list[0]
				self.fullmerge.markers[i].scale.y = res_list[0]
				self.fullmerge.markers[i].scale.z = res_list[0]
			elif i in [1,3]:
				#rospy.logdebug("ITEM is %s for %s", str(res_list[1]), str(i))
				self.fullmerge.markers[i].scale.x = res_list[1]
				self.fullmerge.markers[i].scale.y = res_list[1]
				self.fullmerge.markers[i].scale.z = res_list[1]
			else:
				rospy.logwarn("map_concatenated DEBUG | Error in map alignment. Could not copy scalars to template list.")

			# Set colors. 
			self.fullmerge.markers[i].color.r = 1.0
			self.fullmerge.markers[i].color.g = 0.0
			self.fullmerge.markers[i].color.b = 0.0
			self.fullmerge.markers[i].color.a = 1.0

			# Frame locked 
			self.fullmerge.markers[i].frame_locked = False

			pt_counter = 0
			# Copy Voxel point data.
			if i in [0,1]:
				while len(self.fullmerge.markers[i].points) < len(self.final_map_info_reference.markers[i].points):
					# Points
					self.fullmerge.markers[i].points.append(Point())
					self.fullmerge.markers[i].points[-1].x=self.final_map_info_reference.markers[i].points[pt_counter].x
					self.fullmerge.markers[i].points[-1].y=self.final_map_info_reference.markers[i].points[pt_counter].y
					self.fullmerge.markers[i].points[-1].z=self.final_map_info_reference.markers[i].points[pt_counter].z

					# Colors
					self.fullmerge.markers[i].colors.append(ColorRGBA())
					self.fullmerge.markers[i].colors[pt_counter].r = 1.0
					self.fullmerge.markers[i].colors[pt_counter].g = 1.0
					self.fullmerge.markers[i].colors[pt_counter].b = 1.0
					self.fullmerge.markers[i].colors[pt_counter].a = 1.0

					# Update counter
					pt_counter+=1

			elif i in [2,3]:
				while len(self.fullmerge.markers[i].points) < len(self.final_map_info_incoming.markers[i-num_maps].points):
					# Points
					self.fullmerge.markers[i].points.append(Point())
					self.fullmerge.markers[i].points[-1].x=self.final_map_info_incoming.markers[i-num_maps].points[pt_counter].x
					self.fullmerge.markers[i].points[-1].y=self.final_map_info_incoming.markers[i-num_maps].points[pt_counter].y
					self.fullmerge.markers[i].points[-1].z=self.final_map_info_incoming.markers[i-num_maps].points[pt_counter].z

					# Colors
					self.fullmerge.markers[i].colors.append(ColorRGBA())
					self.fullmerge.markers[i].colors[pt_counter].r = 1.0
					self.fullmerge.markers[i].colors[pt_counter].g = 1.0
					self.fullmerge.markers[i].colors[pt_counter].b = 1.0
					self.fullmerge.markers[i].colors[pt_counter].a = 1.0

					# Update counter
					pt_counter+=1
			else:
				rospy.logfatal("map_concatenated FATAL | Out of index range. Cannot make voxels. ")

			rospy.logdebug("map_concatenated DEBUG | Length of concat points is: %s | Length of concat colors is %s", str(len(self.fullmerge.markers[i].points)), str(len(self.fullmerge.markers[i].colors)))

##############################################
# Map Align Part IVb: Map Alignment
##############################################
	def align_maps(self, dxi = 0, dyi = 0, dzi = 0):
			# dxi, dyi, dzi inputs are for manual tuning if desired. 
			if len(self.final_map_info_incoming.markers) > 0 and len(self.final_map_info_reference.markers) > 0:
				rospy.loginfo("Attempting map alignment.")

			# 1. Combine voxels will be using into 1 full map. 
				self.map_concatenated(num_maps=2)

				'''
				self.fullmerge.markers = self.final_map_info_reference.markers
				for i in range (0,len(self.final_map_info_incoming.markers)):
					self.fullmerge.markers.append(self.final_map_info_incoming.markers[i])
				'''

			# 2. Search for Translation Offset
				# TRANSLATING the RbKairos Outermost point [1][2] to the Turtle [0][2]
				map_feature_1 = self.features[0][2]
				moving_feature_1 = self.features[1][2]
				# Get offset
				dx = round(map_feature_1[0]-moving_feature_1[0],3)
				dy = round(map_feature_1[1]-moving_feature_1[1],3)

				# APPLY TRANSLATION Offset
				rospy.logdebug("map_alignment DEBUG | dx %s | dy %s |", str(dx),str(dy))
				rospy.logdebug("map_alignment DEBUG | x BEFORE TRANS %s | y BEFORE TRANS %s |", str(self.fullmerge.markers[2].points[0].x),str(self.fullmerge.markers[2].points[0].y))
				for i in range(2,4):
					for point in range(0,len(self.fullmerge.markers[i].points)):
						x = self.fullmerge.markers[i].points[point].x
						x+=(dx+dxi)
						self.fullmerge.markers[i].points[point].x = x
						y = self.fullmerge.markers[i].points[point].y
						y+=(dy+dyi)
						self.fullmerge.markers[i].points[point].y = y
						# Manual z offset (WIP to Automate.)
						z = self.fullmerge.markers[i].points[point].z
						z+=dzi
						self.fullmerge.markers[i].points[point].z = z

				rospy.logdebug("map_alignment DEBUG | x AFTER TRANS %s | y AFTER TRANS %s |", str(self.fullmerge.markers[2].points[0].x),str(self.fullmerge.markers[2].points[0].y))

			# 3. Search for Rotation Offset
				# ROTATING the RbKairos Outermost point [1][1] to the Turtle [0][1]
				map_feature_2 = self.features[0][1]
				moving_feature_2 = self.features[1][1]
				## Need to apply offset to moving features too. 
				rospy.logdebug("map_alignment DEBUG | ROTATE FEATURES BEFORE | %s | %s |", str(map_feature_2),str(moving_feature_2))
				moving_feature_2[0]= round(moving_feature_2[0]+dx,3)
				moving_feature_2[1]= round(moving_feature_2[1]+dy,3)
				rospy.logdebug("map_alignment DEBUG | ROTATE FEATURES AFTER | %s | %s |", str(map_feature_2),str(moving_feature_2))

				## Need to Determine the angle offset to get from the Rbkairos point to the 
				## Turtle point. 
				# Rotation is about map_feature_1 = self.features[0][2]

				x_map = round(map_feature_1[0]-map_feature_2[0],8)
				y_map = round(map_feature_1[1]-map_feature_2[1],8)
				x_moving = round(map_feature_1[0]-moving_feature_2[0],8)
				y_moving = round(map_feature_1[1]-moving_feature_2[1],8)

				angle_map = math.atan2(y_map,x_map)
				angle_moving = math.atan2(y_moving,x_moving)
				angle_diff = angle_moving-angle_map
				rospy.logdebug("map_alignment DEBUG | THE INCOMING map must be rotated %s radians", str(angle_diff))

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
# Map Align Part VI: Map Merge
##############################################
	def map_merge(self):
		# Check if exist 
		if len(self.final_map_info_incoming.markers) > 0 and len(self.final_map_info_reference.markers) > 0:
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
			'''
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

			# Get Resolution Sizes (Assume cubic voxels)
			res_list = []
			for i in range(0,len(self.fullmerge.markers)):
				if self.fullmerge.markers[i].scale.x not in res_list:
					res_list.append(self.fullmerge.markers[i].scale.x)

			# RES_list length is 2 

			# Container merge data based on resolution. Size based on num of resolutions.
			merge_list = []
			for i in range(0,len(res_list)):
				merge_list.append([res_list[i],0])

			# Loop through data. Merge it 
			for i in range(0,len(self.fullmerge.markers)/len(res_list)):
				# Copy 2 sets trying to merge so can modify
				voxels1 = []
				voxels2 = []
				# The base map. 
				for point in range(0,len(self.fullmerge.markers[i].points)):
					x = self.fullmerge.markers[i].points[point].x
					y = self.fullmerge.markers[i].points[point].y
					z = self.fullmerge.markers[i].points[point].z
					voxels1.append([x,y,z])
				# The currently seen map. 
				for point in range(0,len(self.fullmerge.markers[i+len(res_list)].points)):
					x = self.fullmerge.markers[i+len(res_list)].points[point].x
					y = self.fullmerge.markers[i+len(res_list)].points[point].y
					z = self.fullmerge.markers[i+len(res_list)].points[point].z
					voxels2.append([x,y,z])

				'''
				Looping method. Works but SLOW. Don't use. 
				'''
				'''
				time_start = rospy.get_time()
				# Look for matches 
				merge_list_part = []
				merge_purge_list = [[],[]]
				for point in range(0,len(voxels1)):
					# If match, copy 1. Save index so can Delete the match from both lists later
					if voxels1[point] in voxels2:
						merge_list_part.append(voxels1[i])
						merge_purge_list[0].append(point)
						merge_purge_list[1].append(voxels2.index(voxels1[point]))

				#print("MERGE PURGES",len(merge_purge_list[0]),len(merge_purge_list[1]))
				# Perform purges
				# Sort and Reverse so can pop correctly 
				merge_purge_list[0] = merge_purge_list[0][::-1]
				merge_purge_list[1].sort()
				merge_purge_list[1] = merge_purge_list[1][::-1]
				for point in range(0,len(merge_purge_list[0])):
					voxels1.pop(merge_purge_list[0][point])
					voxels2.pop(merge_purge_list[1][point])
				

				# Merge into new list 
				merge_list_part=merge_list_part+voxels1+voxels2

				# Save 
				merge_list[i][1] = merge_list_part

				time_diff = rospy.get_time()-time_start
				print("Time Diff for LOOP MERGING:", i, "is", time_diff, "Seconds")
				'''
				'''
				Alternative merge method using strings and sets. 
				ORDERS of Magnitude faster. 
				'''

				time_start = rospy.get_time()
				# Create Combined list 
				merge_list2=voxels1+voxels2
				merge_list_set=set()
				# Cast to set to eliminate duplicates
				for item in range(0,len(merge_list2)):
					# Need to convert inner lists to clean strings
					merge_list_set.add(str(merge_list2[item])[1:-1].replace(" ",""))

				# Convert back from set to list
				merge_list2 = list(merge_list_set)
				for item in range(0,len(merge_list2)):
					merge_list2[item]=(merge_list2[item].split(","))

					for num in range(0,len(merge_list2[item])):
						merge_list2[item][num]=float(merge_list2[item][num])

				time_diff = rospy.get_time()-time_start
				rospy.logdebug("Map Merge DEBUG | Time Diff for SET MERGING at level %s is %s Seconds.", str(i), str(time_diff))

				# Save
				merge_list[i][1] = merge_list2

				
			# For a comparison of lengths
			info_size_list2 = []
			for i in range(0,len(self.fullmerge.markers)):
				item = (i, len(self.fullmerge.markers[i].points), len(self.fullmerge.markers[i].colors))
				info_size_list2.append(item)

			rospy.logdebug("Map Merge DEBUG | ORIGINAL List by Size | Voxel size: %s, Length %s | Voxel size: %s, Length %s",str(merge_list[0][0]),str(info_size_list2[0][1]+info_size_list2[2][1]),str(merge_list[1][0]),str(info_size_list2[1][1]+info_size_list2[3][1]))
			rospy.logdebug("Map Merge DEBUG | MERGED List by Size | Voxel size: %s, Length %s | Voxel size: %s, Length %s",str(merge_list[0][0]), str(len(merge_list[0][1])), str(merge_list[1][0]), str(len(merge_list[1][1])))

			# Make the merged marker array.
			for i in range(0,2):
				self.final_merge.markers.append(Marker())

			for i in range(0,2):
				# Set Header 
				self.final_merge.markers[i].header.seq=0
				self.final_merge.markers[i].header.stamp=rospy.Time.now()
				self.final_merge.markers[i].header.frame_id="world"

				# Namespace
				self.final_merge.markers[i].ns = self.fullmerge.markers[0].ns
				# Set id 
				self.final_merge.markers[i].id = i
				# Type is voxel
				self.final_merge.markers[i].type = 6
				# Action is add 
				self.final_merge.markers[i].action = 0
				# Position is original 
				self.final_merge.markers[i].pose.position.x = 0.0
				self.final_merge.markers[i].pose.position.y = 0.0
				self.final_merge.markers[i].pose.position.z = 0.0
				# Orientation is original 
				self.final_merge.markers[i].pose.orientation.x = 0.0
				self.final_merge.markers[i].pose.orientation.y = 0.0
				self.final_merge.markers[i].pose.orientation.z = 0.0
				self.final_merge.markers[i].pose.orientation.w = 1.0

				# Copy scalars
				self.final_merge.markers[i].scale.x = merge_list[i][0]
				self.final_merge.markers[i].scale.y = merge_list[i][0]
				self.final_merge.markers[i].scale.z = merge_list[i][0]

				# Set colors. 
				self.final_merge.markers[i].color.r = 1.0
				self.final_merge.markers[i].color.g = 0.0
				self.final_merge.markers[i].color.b = 0.0
				self.final_merge.markers[i].color.a = 1.0

				# Frame locked 
				self.final_merge.markers[i].frame_locked = False

				# Redimensionalize point and color data 
				original_length = len(self.final_merge.markers[i].points)
				merged_length = len(merge_list[i][1])
				#print("ORIGINAL LENGTH", original_length, "MERGED LENGTH", merged_length)

				# Want dimensions EXACTLY the same. 
				while original_length != merged_length:
					self.final_merge.markers[i].points.append(Point())
					self.final_merge.markers[i].colors.append(ColorRGBA())

					# Update comparison.
					original_length = len(self.final_merge.markers[i].points)

				rospy.logdebug("Map Merge DEBUG | DONE REDIMENSIONALIZING for level %s",str(i))

				# Fix point data
				for point in range(0,merged_length):
					# Copy points over 
					self.final_merge.markers[i].points[point].x = merge_list[i][1][point][0]
					self.final_merge.markers[i].points[point].y = merge_list[i][1][point][1]
					self.final_merge.markers[i].points[point].z = merge_list[i][1][point][2]
					# Set colors. 
					self.final_merge.markers[i].colors[point].r = 1.0
					self.final_merge.markers[i].colors[point].g = 1.0
					self.final_merge.markers[i].colors[point].b = 1.0
					self.final_merge.markers[i].colors[point].a = 1.0
				
			# Prevent infinite concatenation. 
			self.merged = True

##############################################
# Map Align Part VII: Data Analytics
##############################################
	def map_analysis(self,lower_h_limit=7):
	##### Need to do data sorting on fullmerge data to sort by height... again. 
	##### Might as well do it for entire thing.
		# Port Data 
		'''
		# Original unmerged data analysis 
		analysis_data_map = self.fullmerge.markers[0:2]
		analysis_data_aligned = self.fullmerge.markers[2:4]
		analysis_data = [analysis_data_map,analysis_data_aligned]
		'''
		# Merged data Analysis 
		analysis_data_map = self.fullmerge.markers[0:2]
		analysis_data_aligned = self.final_merge.markers
		analysis_data = [analysis_data_map,analysis_data_aligned]

		### 1. Get elevation data. How many curves working with. 
		# Voxel list 
		voxel_list = [[[],[]],[[],[]]]
		# Elevation data 
		z_counter = [[0,0],[0,0]]
		z_level = [[[],[]],[[],[]]]
		z_data = [[[],[]],[[],[]]]
		for data in range(0,len(analysis_data)):
			for marker in range(0,len(analysis_data[data])):
				for i in range(0,len(analysis_data[data][marker].points)):					
					x = round(analysis_data[data][marker].points[i].x,3)
					y = round(analysis_data[data][marker].points[i].y,3)
					z = round(analysis_data[data][marker].points[i].z,3)
					# Save unique elevation data, least to greatest. 
					if z not in z_data[data][marker]:
						z_counter[data][marker]+=1
						z_level[data][marker].append(z)
						z_level[data][marker].sort()
					# Save Data
					z_data[data][marker].append(z)
					voxel_list[data][marker].append((x,y,z))

		# Make List to store values for contour fitting.  
		contour_data = [[[],[]],[[],[]]]
		for data in range(0,len(analysis_data)):
			for marker in range(0,len(analysis_data[data])):
				for i in range(0,z_counter[data][marker]):
					contour_data[data][marker].append((z_level[data][marker][i],[[],[]]))

		rospy.logwarn("|||||||||||||||||||||")
		rospy.logdebug("Map Analysis DEBUG | Z_count map1 is %s", str(z_counter[0][0]))
		rospy.logdebug("Map Analysis DEBUG | Z_count aligned1 is %s", str(z_counter[1][0]))
		rospy.logdebug("Map Analysis DEBUG | Z_level map1 is %s", str(z_level[0][0]))
		rospy.logdebug("Map Analysis DEBUG | Z_level aligned1 is %s", str(z_level[1][0]))
		rospy.logwarn("|||||||||||||||||||||")
		rospy.logdebug("Map Analysis DEBUG | Z_count map2 is %s", str(z_counter[0][1]))
		rospy.logdebug("Map Analysis DEBUG | Z_count aligned2 is %s", str(z_counter[1][1]))
		rospy.logdebug("Map Analysis DEBUG | Z_level map2 is %s", str(z_level[0][1]))
		rospy.logdebug("Map Analysis DEBUG | Z_level aligned2 is %s", str(z_level[1][1]))
		rospy.logwarn("|||||||||||||||||||||")

		# Resort Data for Curve Fitting
		for data in range(0,len(analysis_data)):
			for marker in range(0,len(analysis_data[data])):
				for i in range(0,len(voxel_list[data][marker])):
					# Check for elevation match. Save x,y if found. 
					for level in range(0,len(contour_data[data][marker])):
						if voxel_list[data][marker][i][2] == contour_data[data][marker][level][0]:
							contour_data[data][marker][level][1][0].append(voxel_list[data][marker][i][0])
							contour_data[data][marker][level][1][1].append(voxel_list[data][marker][i][1])
							break

	##### Can just check alignment for the 1 cm resolution vs the original map 1 cm resolution
		for i in range(0,2):
			if i == 0:
				data = self.contour_data[0]
			else:
				data = contour_data[1][1]
		# Pull in contour data 
			for h in range(lower_h_limit,len(data)):
				x_data = np.array(data[h][1][0])
				y_data = np.array(data[h][1][1])
				# Plotting an ellipse for each. 
				# Need params for each ellipse. 
				v = self.fit_ellipse(x_data,y_data)
				ellipse_data = self.polyToParams(v)

				## Get the arrays needed to plot the ellipse.
				t = np.linspace(0,2*3.14,100)
				Ell = np.array([ellipse_data[2]*np.cos(t),ellipse_data[3]*np.sin(t)])

				if i == 0:
					self.data_map_analysis.append([h,[ellipse_data[0]+Ell[0,:]],[ellipse_data[1]+Ell[1,:]]])
				else:
					self.data_aligned_analysis.append([h,[ellipse_data[0]+Ell[0,:]],[ellipse_data[1]+Ell[1,:]]])
				
		# Do some RMSE Stuff between marged map and original map
		if len(self.data_map_analysis) != len(self.data_aligned_analysis):
			pass
			#rospy.logwarn("Map Analysis dimensions NOT same. Will not perform RMSE analysis. Map data length is: %s. Aligned data length is %s.",str(self.data_map_analysis),str(self.data_aligned_analysis))
		else:
			# Store the values 
			rmse_err = []
			for i in range(0,len(self.data_map_analysis)):
				rmse_err.append([self.contour_data[0][self.data_map_analysis[i][0]][0],0])

			# Get RMSE for each height
			for i in range(0,len(self.data_map_analysis)):
				rmse_err_htop = 0

				listed_pointsx1 = np.ndarray.tolist(self.data_map_analysis[i][1][0])
				listed_pointsy1 = np.ndarray.tolist(self.data_map_analysis[i][2][0])
				listed_pointsx2 = np.ndarray.tolist(self.data_aligned_analysis[i][1][0])
				listed_pointsy2 = np.ndarray.tolist(self.data_aligned_analysis[i][2][0])


				for point in range(0,len(listed_pointsx1)):
					x1 = listed_pointsx1[point]
					y1 = listed_pointsy1[point]
					x2 = listed_pointsx2[point]
					y2 = listed_pointsy2[point]
					err = (x2-x1)**2+(y2-y1)**2
					rmse_err_htop+=err
				
				# Save average RMSE for height
				rmse_err[i][1] = math.sqrt(rmse_err_htop/(len(listed_pointsx1)*2))
			
			# Get Average RMSE for all contours
			rmse_err_avg = 0
			for i in range(0,len(rmse_err)):
				rmse_err_avg+=rmse_err[i][1]
			rmse_err_avg/=len(rmse_err)

			rospy.loginfo("Average RMSE err per height | %s", str(rmse_err))
			rospy.loginfo("Average RMSE err is %s.", str(rmse_err_avg))

		self.analyzed = True

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
	log_level = rospy.DEBUG
	rospy.init_node("map_alignment",anonymous=True, log_level=log_level)
	# If plot Visualize - True, need to call it in the main loop.
	alignment = MapAlignment(plot_visualize=False,analyze=True)

	if alignment.plot_visualize:
		visualized_map=False

	while not rospy.is_shutdown():
		try:
			# Wait for Data and robot to be at waypoint.
			if log_level == rospy.DEBUG:
				alignment.align_pub1.publish(alignment.final_map_info_reference)
				alignment.align_pub2.publish(alignment.final_map_info_incoming)

			if len(alignment.contour_data[0]) > 0 and len(alignment.contour_data[1]) > 0 and alignment.at_waypoint:
				rospy.loginfo_once("Near Waypoint AND Voxel Data loaded.")

				# Call the Feature Finders ONCE...
				if not alignment.features_found:
					rospy.loginfo("Looking for map features.")
					alignment.feature_finder(data=alignment.contour_data[0],map_name="Top REFERENCE",lower_h_limit=7,upper_h_limit = 13)
					alignment.feature_finder(data=alignment.contour_data[1],map_name="Bottom INCOMING",lower_h_limit=2,upper_h_limit = 6)
					rospy.logdebug("Features %s",str(alignment.features))
					alignment.features_found = True

				# Call the Map Alignment ONCE....
				if not alignment.aligned:
					alignment.align_maps(dxi = 0, dyi = 0, dzi = -1.01)
					rospy.loginfo("Maps aligned.")

				# Call the map verification ONCE...
				if not alignment.map_rescaled:
					alignment.map_rescaling()
					rospy.loginfo("Maps Rescaled.")

				# Call the map Merging ONCE After maps aligned and verified
				if not alignment.merged and alignment.align_maps and alignment.map_rescaled:
					rospy.loginfo("Maps Merging.")
					alignment.map_merge()
					rospy.loginfo("Maps Merged.")
					
				# If merge successful:
				if alignment.merged:
					alignment.align_pub.publish(alignment.fullmerge)

					# Analyze the data if enabled Opposite of the analyze boolean.
					if alignment.analyzed == False:
						alignment.map_analysis()

					# Show plots if visualization enabled. 
					if alignment.plot_visualize:
						if not visualized_map:
							rospy.loginfo("VISUALIZING MAP.")
							'''
							for i in range(0,len(alignment.contour_data)):
								alignment.feature_finder(data=alignment.contour_data[i],map_name="merged",visualize=True,lower_h_limit=0,upper_h_limit = 8, fig = 1)
							'''
							# REFERENCE DATA is TOP = 0
							# INCOMING DATA is BOTTOM = 1
							alignment.feature_finder(data=alignment.contour_data[0],map_name="top",visualize=True,lower_h_limit=7,upper_h_limit = 13)
							# EXCLUDE 0,1,7+
							alignment.feature_finder(data=alignment.contour_data[1],map_name="bottom",visualize=True,lower_h_limit=2,upper_h_limit = 6)

							visualized_map = True
							#print("ALIGNMENT LENGTH",len(alignment.contour_data))

						plt.show(alignment.plot)

				# Dynamic Reconfigure 
				#srv = Server(rbkairos_alignmentConfig, reconfigure)
				#converter.debugger()
			else:
				rospy.logdebug_throttle(10,"Countor data 1 len: %s | 2: %s",str(len(alignment.contour_data[0])),str(len(alignment.contour_data[1])))
				if alignment.at_waypoint:
					rospy.loginfo_throttle(10,"Waiting to get near Waypoint")
				else:
					rospy.loginfo_throttle(10,"Near waypoint. Waiting for voxel data.")
					if len(alignment.contour_data[0]) >= 1:
						rospy.loginfo_throttle(10,"Map data loaded.")
					elif len(alignment.contour_data[1]) >= 1:
						rospy.loginfo_throttle(10,"Incoming data loaded.")

		except rospy.ROSInterruptException:
			rospy.logfatal(rospy.ROSInterruptException)