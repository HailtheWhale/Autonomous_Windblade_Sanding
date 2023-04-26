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
# Dynamic Reconfigure Enabled 
from dynamic_reconfigure.server import Server
from windblade_scanner.cfg import rbkairos_alignmentConfig

class MapAlignment():

	def __init__(self, loop_rate=5.0):

		# Safety 
		self.rate = rospy.Rate(loop_rate)

		# Time stamp for both
		self.time_stamp = rospy.Time.now()

        # Define topics 
		self.node_name = "map_alignment/"
		self.map_turtle = str(rospy.get_param(self.node_name + "loaded_map_topic_turtlebot"))
		self.map_rbkairos = str(rospy.get_param(self.node_name + "loaded_map_topic_rbkairos"))

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
		self.align_sub_turtle=rospy.Subscriber(self.map_turtle,MarkerArray, self.turtle_scrubber)
		self.align_sub_rbkairos=rospy.Subscriber(self.map_rbkairos,MarkerArray, self.rbkairos_scrubber)

		#self.align_pub1 = rospy.Publisher("/occupied_cells_loaded_map_scrubbed_turtle",MarkerArray, queue_size=1)
		#self.align_pub2 = rospy.Publisher("/occupied_cells_loaded_map_scrubbed_rbkairos",MarkerArray, queue_size=1)
		self.align_pub = rospy.Publisher("/occupied_cells_loaded_map_scrubbed_merged",MarkerArray, queue_size=1)

		# Msgs
		self.loaded_map_info_turtle = MarkerArray()
		self.final_map_info_turtle = MarkerArray()
		self.loaded_map_info_rbkairos = MarkerArray()
		self.final_map_info_rbkairos = MarkerArray()
		# Fullmerge
		self.fullmerge=MarkerArray()

		# Merged?
		self.merged = False

# Helper functions
	def quat_to_euler(self, x, y, z, w):
		quat = [x,y,z,w]
		roll,pitch,yaw = tf.transformations.euler_from_quaternion(quat)
		return [roll,pitch,yaw]

	def euler_to_quat(self, r, p, y):
		x,y,z,w = tf.transformations.quaternion_from_euler(r,p,y)
		return [x,y,z,w]

### Turtle scrubber
	def turtle_scrubber(self, msg):
		loaded_info_array1 = msg.markers
		# print(len(msg.markers)) # === 17

		######################
		# Partition Data
		######################
		info_size_list1 = []
		for i in range(0,len(loaded_info_array1)):
			item = (i, len(loaded_info_array1[i].points), len(loaded_info_array1[i].colors))
			info_size_list1.append(item)
			#rospy.loginfo("Turtle %s",str(i))

		#rospy.loginfo("Turtle List %s",str(info_size_list1))

		# Save Raw Data
		self.loaded_map_info_turtle.markers = [loaded_info_array1[15],loaded_info_array1[16]]

		##########################
		# FILTER DATA 
		##########################

		############################
		# RANGE FILTER
		############################

		for array in range(0,len(self.loaded_map_info_turtle.markers)):
			#print(array)
			#print("Original Length 1", len(self.loaded_map_info_turtle.markers[array].points))

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
			#print("Scrubbed Length 1", len(self.loaded_map_info_turtle.markers[array].points))

		self.final_map_info_turtle.markers = self.loaded_map_info_turtle.markers

### Rbkairos scrubber
	def rbkairos_scrubber(self, msg):
		loaded_info_array2 = msg.markers
		# print(len(msg.markers)) # === 17

		######################
		# Partition Data
		######################
		info_size_list2 = []
		for i in range(0,len(loaded_info_array2)):
			item = (i, len(loaded_info_array2[i].points), len(loaded_info_array2[i].colors))
			info_size_list2.append(item)
			#rospy.loginfo("Rbkairos %s",str(i))

		#rospy.loginfo("Rbkairos List %s",str(info_size_list2))

		# Save Raw Data
		self.loaded_map_info_rbkairos.markers = [loaded_info_array2[15],loaded_info_array2[16]]

		##########################
		# FILTER DATA 
		##########################

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
			#print("Scrubbed Length 1", len(self.loaded_map_info_rbkairos.markers[array].points))
		
		# Reorient the map
		self.final_map_info_rbkairos.markers = self.loaded_map_info_rbkairos.markers
		self.roll,self.pitch,self.yaw = 0,0,1.52
		ox,oy,oz,ow = self.euler_to_quat(self.roll,self.pitch,self.yaw)
		for i in range(0,len(self.final_map_info_rbkairos.markers)):
			self.final_map_info_rbkairos.markers[i].pose.position.x = 0.38
			self.final_map_info_rbkairos.markers[i].pose.position.y = 2.67
			self.final_map_info_rbkairos.markers[i].pose.position.z = 0.05
			self.final_map_info_rbkairos.markers[i].pose.orientation.x = ox
			self.final_map_info_rbkairos.markers[i].pose.orientation.y = oy
			self.final_map_info_rbkairos.markers[i].pose.orientation.z = oz
			self.final_map_info_rbkairos.markers[i].pose.orientation.w = ow
			for point in range(0,len(self.final_map_info_rbkairos.markers[i].colors)):
				self.final_map_info_rbkairos.markers[i].colors[point].r = 1.0
				self.final_map_info_rbkairos.markers[i].colors[point].g = 0.0
				self.final_map_info_rbkairos.markers[i].colors[point].b = 0.0
				self.final_map_info_rbkairos.markers[i].colors[point].a = 1.0


# Merge maps 
	def map_merge(self):
		# To prevent infinite concatenation
		if not self.merged:
			# Check if exist 
			if len(self.final_map_info_rbkairos.markers) > 0 and len(self.final_map_info_turtle.markers) > 0:
				self.fullmerge = self.final_map_info_turtle
				for i in range (0,len(self.final_map_info_rbkairos.markers)):
					self.fullmerge.markers.append(self.final_map_info_rbkairos.markers[i])
				'''
				# Check for location matches. Scrub duplicates. 
				xyz_merge_list = []
				for i in range(0,len(self.fullmerge.markers)):
					for index in range(0,len(self.fullmerge.markers[i].points)):
						x = self.fullmerge.markers[i].points[index].x
						y = self.fullmerge.markers[i].points[index].y
						z = self.fullmerge.markers[i].points[index].z
						xyz_merge_list.append((x,y,z,i))

				print(len(xyz_merge_list))
				xyz_merge_list_purged = []
				for index in range(0,len(xyz_merge_list)):
					x = xyz_merge_list[index][0]
					y = xyz_merge_list[index][1]
					z = xyz_merge_list[index][2]
					i = index+1
					while i<=len(xyz_merge_list)-1:
						xi = xyz_merge_list[i][0]
						yi = xyz_merge_list[i][1]
						zi = xyz_merge_list[i][2]
						if x == xi and y == yi and z == zi:
							xyz_merge_list_purged.append((xi,yi,zi,i))
						i+=1

				print(xyz_merge_list_purged)
				'''
				for i in range(0,len(self.fullmerge.markers)):
					#print(self.fullmerge.markers[i].color)
					self.fullmerge.markers[i].id = i
					self.fullmerge.markers[i].scale.x = 0.01
					self.fullmerge.markers[i].scale.y = 0.01
					self.fullmerge.markers[i].scale.z = 0.01
					if i in [0,1]:
						self.fullmerge.markers[i].color.r = 1.0
						self.fullmerge.markers[i].color.g = 0.0
						self.fullmerge.markers[i].color.b = 0.0
						self.fullmerge.markers[i].color.a = 1.0
					else:
						self.fullmerge.markers[i].color.r = 0.0
						self.fullmerge.markers[i].color.g = 1.0
						self.fullmerge.markers[i].color.b = 0.0
						self.fullmerge.markers[i].color.a = 1.0

				self.merged = True
# Debug
	def debugger(self):
		print("rospy time now",type(rospy.Time.now()))
		print("header time stamp",type(self.time_stamp))

# Dynamic Reconfigure 
def reconfigure(config,level):
	rospy.logwarn("Reconfigured!")
	rospy.loginfo(config)
	return config

if __name__ == '__main__':
	rospy.init_node("map_alignment",anonymous=True, log_level=rospy.INFO)
	alignment = MapAlignment()

	while not rospy.is_shutdown():
		try:
			#alignment.align_pub1.publish(alignment.final_map_info_turtle)
			#alignment.align_pub2.publish(alignment.final_map_info_rbkairos)
			alignment.map_merge()
			if alignment.merged:
				alignment.align_pub.publish(alignment.fullmerge)

			# Dynamic Reconfigure 
			#srv = Server(rbkairos_alignmentConfig, reconfigure)
			#converter.debugger()

		except rospy.ROSInterruptException:
			rospy.logfatal(rospy.ROSInterruptException)

'''
		##########################
		# NEAREST NEIGHBOR FILTER
		#########################		
		# How many POINTS to consider at a time?
		nearest_neighbor_res = 100
		percent_dev = 0.1
		avg_percent_dev = 0.1
		# How Many DEVIATIONS from the avg were found?
		anomoly_countA = 0
		anomoly_listA = []
		anomoly_countB = 0
		anomoly_listB = []

		# Slice up the pointcloud data into the right size. 	
		total_avg_z = 0
		total_avg_y = 0
		total_avg_x = 0

		avg_z_list = []
		avg_y_list = []
		avg_x_list = []

		for kernel in range(0,int(len(self.loaded_map_info_turtle.markers[0].points)/nearest_neighbor_res)):
			avg_z = 0
			avg_y = 0
			avg_x = 0
			# Get the average for each kernel
			for i in range(0,nearest_neighbor_res):
				index = kernel*10+i
				avg_z += self.loaded_map_info_turtle.markers[0].points[index].z
				avg_y += self.loaded_map_info_turtle.markers[0].points[index].y
				avg_x += self.loaded_map_info_turtle.markers[0].points[index].x
			avg_z /= nearest_neighbor_res
			tol_z = avg_z*percent_dev
			avg_y /= nearest_neighbor_res
			tol_y = avg_y*percent_dev
			avg_x /= nearest_neighbor_res
			tol_x = avg_x*percent_dev


		# For type 2 comparison
			total_avg_z+=avg_z
			tol_z = total_avg_z*avg_percent_dev
			total_avg_y+=avg_y
			tol_y = total_avg_y*avg_percent_dev
			total_avg_x+=avg_x
			tol_x = total_avg_x*avg_percent_dev

			avg_z_list.append(avg_z)
			avg_y_list.append(avg_y)
			avg_x_list.append(avg_x)

		for kernel in range(0,int(len(self.loaded_map_info_turtle.markers[0].points)/nearest_neighbor_res)):

			z = avg_z_list[kernel]
			y = avg_y_list[kernel]
			x = avg_x_list[kernel]

			if z > (total_avg_z+tol_z) or z < (total_avg_z-tol_z) or y > (total_avg_y+tol_y) or y < (total_avg_y-tol_y) or x > (total_avg_x+tol_x) or x < (total_avg_x-tol_x):
				anomoly_countB+=1
				item = ("kernel", kernel)
				anomoly_listB.append(item)

		print("anomoly_countA",anomoly_countA)
		print("anomoly_countB",anomoly_countB)
		#print(anomoly_listA)
'''