#!/usr/bin/python
'''
The octomap merge package expects the "Neighbor" Octomap to use the custom
msg type defined by the package... so, even if merging only 2 maps, need to 
convert msg types to make this possible. 
'''
# ROS general imports
import rospy

# Msg imports
from visualization_msgs.msg import MarkerArray, Marker

class MapAlignment():

    def __init__(self, loop_rate=5.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)

        # Time stamp for both
        self.time_stamp = rospy.Time.now()

        # Subscribers, publishers 
        self.align_pub = rospy.Publisher("/occupied_cells_loaded_map_scrubbed",MarkerArray, queue_size=1)
        self.align_sub=rospy.Subscriber("/occupied_cells_vis_array_loaded_map",MarkerArray, self.scrubber)

        # Msgs
        self.loaded_map_info = MarkerArray()
        self.final_map_info = MarkerArray()


    def scrubber(self, msg):
		loaded_info_array = msg.markers
		# print(len(msg.markers)) # === 17

		######################
		# Partition Data
		######################
		info_size_list = []
		for i in range(0,len(loaded_info_array)):
			item = (i, len(loaded_info_array[i].points), len(loaded_info_array[i].colors))
			info_size_list.append(item)

		print(info_size_list)

		# Save Raw Data
		self.loaded_map_info.markers = [loaded_info_array[15],loaded_info_array[16]]

		##########################
		# FILTER DATA 
		##########################

		############################
		# RANGE FILTER
		############################

		for array in range(0,len(self.loaded_map_info.markers)):
			print(array)
			print("Original Length 1", len(self.loaded_map_info.markers[array].points))

			anomoly_count_range = 0
			anomoly_list_range = []

			# Range Filter
			for i in range(0,len(self.loaded_map_info.markers[array].points)):
				z = self.loaded_map_info.markers[array].points[i].z
				y = self.loaded_map_info.markers[array].points[i].y
				x = self.loaded_map_info.markers[array].points[i].x
				if x < (0) or x > (1) or z< 0.62:
					anomoly_count_range+=1
					item = ("index", i, x,y,z)
					anomoly_list_range.append(item)

			#######################
			# Remove Anomolies
			#######################
			for i in range(0,len(anomoly_list_range)):
				index = anomoly_list_range[i][1]-i
				del self.loaded_map_info.markers[array].points[index]
				del self.loaded_map_info.markers[array].colors[index]

			# Save Data
			print("SCRUBBED 1!")
			print("Scrubbed Length 1", len(self.loaded_map_info.markers[array].points))
		

		self.final_map_info.markers = self.loaded_map_info.markers

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

		for kernel in range(0,int(len(self.loaded_map_info.markers[0].points)/nearest_neighbor_res)):
			avg_z = 0
			avg_y = 0
			avg_x = 0
			# Get the average for each kernel
			for i in range(0,nearest_neighbor_res):
				index = kernel*10+i
				avg_z += self.loaded_map_info.markers[0].points[index].z
				avg_y += self.loaded_map_info.markers[0].points[index].y
				avg_x += self.loaded_map_info.markers[0].points[index].x
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

		for kernel in range(0,int(len(self.loaded_map_info.markers[0].points)/nearest_neighbor_res)):

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

    def debugger(self):
        print("rospy time now",type(rospy.Time.now()))
        print("header time stamp",type(self.time_stamp))

if __name__ == '__main__':
    rospy.init_node("map_alignment",anonymous=True, log_level=rospy.INFO)
    alignment = MapAlignment()

    while not rospy.is_shutdown():
	try:
		alignment.align_pub.publish(alignment.final_map_info)
            #converter.debugger()

	except rospy.ROSInterruptException:
		rospy.logfatal(rospy.ROSInterruptException)