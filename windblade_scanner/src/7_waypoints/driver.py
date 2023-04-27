#!/usr/bin/python

# ROS general imports
import rospy
import math 
import numpy as np 

# Msgs for publishers and subscribers.
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry

#Import Pandas library to read csv files for waypoint data
import pandas as pd

# For tfs to understand robot base orientation. 
from tf.transformations import euler_from_quaternion

class driver(object):
    
	def __init__(self):
		'''
		constructor
		'''
		#Initialize ros node
		rospy.init_node('waypoint_seeker', log_level=rospy.DEBUG)        

		# ROS parameters
		# Node name 
		self.node_name = "waypoint_seeker/"
		# ROS parameters
		# Goals
		self.waypoint_pts = str(rospy.get_param('waypoint_file'))
		#Threshold for distance to goal
		self.goal_th_xy = float(rospy.get_param(self.node_name + 'goal_th_xy'))
		self.goal_th_ang = float(rospy.get_param(self.node_name + 'goal_th_ang'))
		# How many laps?
		self.desired_laps = int(rospy.get_param(self.node_name + 'desired_laps'))
		#Cmd vel speeds 
		self.turn_spd = float(rospy.get_param(self.node_name + 'turn_spd'))
		self.linear_spd = float(rospy.get_param(self.node_name + 'linear_spd'))
		# Define topics 
		self.odom_topic = str(rospy.get_param(self.node_name + "odom_topic"))
		self.cmd_vel_topic = str(rospy.get_param(self.node_name + "cmd_vel_topic"))

		#Defining subscriber
		self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.pose_callback)

		#Defining publisher        
		self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 10)

		#Defining the velocity message
		velocity = Twist()
		velocity.angular.x = 0
		velocity.angular.y = 0
		velocity.angular.z = 0
		velocity.linear.x = 0
		velocity.linear.y = 0
		velocity.linear.z = 0    
		self.vmsg = velocity

		#Point to the first goal
		self.active_goal = 0
			
		# Load  waypoint goals
		self.load_goals()

		# For rapid shutdown
		rospy.on_shutdown(self.shutdown_hook)

##########################################################################
# Helper Functions
##########################################################################

	def angle_wrap(self, ang):
		"""
		Return the angle normalized between [-pi, pi].

		Works with numbers and numpy arrays.

		:param ang: the input angle/s.
		:type ang: float, numpy.ndarray
		:returns: angle normalized between [-pi, pi].
		:rtype: float, numpy.ndarray
		"""
		ang = ang % (2 * np.pi)
		if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
			ang -= 2 * np.pi
		elif isinstance(ang, np.ndarray):
			ang[ang > np.pi] -= 2 * np.pi
		return ang

	def shutdown_hook(self):
		""" For more rapid shutdown """
		self.vmsg.linear.x = 0
		self.vmsg.linear.y = 0
		self.vmsg.angular.z = 0
		self.publish()
		rospy.logfatal("Shutdown!")
		rospy.signal_shutdown()
    

##########################################################################
# Waypoint Stuff 
##########################################################################

	def load_goals(self):
		'''
		Loads the desired waypoint as x,y variables. 
		'''
		# Getting Waypoint File Path 
		waypoint_file = self.waypoint_pts
		# Reading File 
		df = pd.read_csv(waypoint_file, sep=",", header=None)
		# Loaded Data
		loaded_x = df[0]
		loaded_y = df[1]

		#Initialize goals
		self.x = np.array([])
		self.y = np.array([])

		# Appending Data 
		i = 0
		lap_counter=1
		while i < len(loaded_x):
			self.x = np.append(self.x,loaded_x[i])
			self.y = np.append(self.y,loaded_y[i])
			i+=1
			# If not enough laps, reset the counter. Continue to append the list of goals. 
			if (i == len(loaded_x) and lap_counter<self.desired_laps):
				i = 0
				lap_counter+=1

		self.print_goals()

	def print_goals(self):
		'''
		Prints ALL waypoint goals' x and y coordinates.
		'''
		rospy.loginfo("List of goals:")
		rospy.loginfo("X:\t " + str(self.x))
		rospy.loginfo("Y:\t " + str(self.y))

	def check_goal(self):
		'''
		Checks if the robot has arrived at waypoint. Calls next one if true. 
		'''
		if self.has_arrived_xy():
			self.next_goal()

	def next_goal(self):
		'''
		Increments the index of the goal in 1 and checks whether
		or not the robot has reached the final goal
		
		'''
		self.active_goal+=1
		if self.active_goal == len(self.x):
			rospy.logdebug("All waypoints reached!!!!")
			rospy.signal_shutdown('Final goal reached! Shutdown!')
			self.shutdown_hook()
		# Still have waypoints left
		rospy.logdebug("Proceeding to the next waypoint...")

	def print_goal(self):
		'''
		Prints the next goal.
		'''
		rospy.loginfo( "Goal: (" + str(self.x[self.active_goal]) + " , " + str(self.y[self.active_goal]) + ")")

##########################################################################
        
	def print_pose(self):
		'''
		Prints the robot's dead reckoned position
		'''
		rospy.loginfo( "Pose: (" + str(self.position_x) + " , " + str(self.position_y) + " )")
        
	def print_goal_and_pose(self):
		'''
		print_goal_and_pose prints both goal and pose
		'''
		rospy.loginfo("\tPose\t\tGoal")
		rospy.loginfo("X:\t%f\t%f",self.position_x,self.x[self.active_goal])
		rospy.loginfo("Y:\t%f\t%f",self.position_y,self.y[self.active_goal])
        
	def dist_to_goal_xy(self):
		'''
		dist_to_goal_xy computes the distance in x and y direction to the 
		active goal
		'''
		distance = math.sqrt((self.position_x - float(self.x[self.active_goal]))**2+(self.position_y - float(self.y[self.active_goal]))**2)

		return distance
        
	def has_arrived_xy(self):
		'''
		Returns True if distance to waypoint is below set threshold. 
		'''
		return self.dist_to_goal_xy()<self.goal_th_xy
        
	def publish(self):	
		self.pub.publish(self.vmsg)
        
        
	def drive(self):
		'''
		Runs until shutdown
		'''
		self.print_goal()
		while not rospy.is_shutdown():
			rospy.sleep(0.03)

	def pose_callback(self,msg):
		'''
		Reads the actual position of the robot, computes the 
		appropiate velocity, publishes it and check if the goal is reached
		'''
		self.read_position(msg)
		self.compute_velocity()
		self.publish()
		self.check_goal()
        
	def read_position(self,msg):
		'''
		Copy the position received in the message (msg) to the
		internal class variables self.position_x, self.position_y and 
		self.position_theta
		
		Tip: in order to convert from quaternion to euler angles give a look
		at tf.transformations
		'''
		# Retrieving x,y, and z positions for quaternion to Euler transformation.
		self.position_x = msg.pose.pose.position.x        
		self.position_y = msg.pose.pose.position.y 
		self.position_z = msg.pose.pose.position.z

		# Getting quaternions
		quat_x = msg.pose.pose.orientation.x
		quat_y = msg.pose.pose.orientation.y
		quat_z = msg.pose.pose.orientation.z
		quat_w = msg.pose.pose.orientation.w
		quat = [quat_x,quat_y,quat_z,quat_w]

		# Calculating Euler Angles from odometry quaternion angles
		roll,pitch,yaw = euler_from_quaternion(quat)

		# Saving yaw.
		self.position_theta = yaw

		# Normalizing the retrieved angle between -pi and pi.
		self.position_theta = self.angle_wrap(self.position_theta)
		# Normalizing between 0 and 2pi
		if self.position_theta < 0:
			self.position_theta += 6.28
      

	def compute_velocity(self):
		'''
		Controls the velocity commands being published.
		'''

		# Distance to goal prints
		# rospy.loginfo("The distance to the goal left is:")
		# rospy.loginfo(self.dist_to_goal_xy())

		# Getting actual needed bearing between current x,y and desired x,y
		# This is between -pi and pi
		required_angle = self.angle_wrap(math.atan2((self.y[self.active_goal] - self.position_y), (self.x[self.active_goal] - self.position_x)))
		# This is between 0 and 2pi
		if required_angle < 0:
			required_angle += 6.28

		# Not within the previously defined x,y tolerance 
		if not self.has_arrived_xy():	

			# Getting angle difference. Both values here between 0 and 2pi.
			ang_diff = self.position_theta-required_angle

			# If outside the angle threshold, Reorient self.
			if abs(ang_diff) >  self.goal_th_ang:
				# If the difference between the needed angle and the current angle is negative
				if ang_diff <= -3.14:
					# Spin CCW
					self.vmsg.angular.z = -self.turn_spd
					self.vmsg.linear.x = self.linear_spd
				elif ang_diff > -3.14 and ang_diff <= 0:
					# Spin CCW
					self.vmsg.angular.z = self.turn_spd
					self.vmsg.linear.x = self.linear_spd
				elif ang_diff > 0 and ang_diff <= 3.14:
					# Spin CCW
					self.vmsg.angular.z = -self.turn_spd
					self.vmsg.linear.x = self.linear_spd
				else:
					# Spin CW
					self.vmsg.angular.z = self.turn_spd
					self.vmsg.linear.x = self.linear_spd
			# Else, move forward. 
			else:
				self.vmsg.angular.z = 0
				self.vmsg.linear.x = self.linear_spd

		# Within x,y tolerance. Spin to desired bearing. 
		else:
			# Stop the robot. 
			rospy.logdebug('AT THE Waypoint!!!!!!!!!!')
			self.vmsg.linear.x = 0.0
			self.vmsg.angular.z = 0
