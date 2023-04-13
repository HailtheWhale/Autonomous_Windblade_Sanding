#!/usr/bin/python

# ROS general imports
import rospy
import tf 
import numpy as np 
import csv
# For starting and killing ROS nodes 
import subprocess

import math
# Fiducial/ auco_detect imports (FOR QR code detection)
from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
# Msgs for publishers and subscribers.
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry

class StaticArm():

    def __init__(self, loop_rate=500.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)
        rospy.on_shutdown(self.shutdown_hook)
        # Node name 
        self.node_name = "arm_frame_publisher/"
        # Parameters
        self.J1=0.0
        self.J2=0-0.48019
        self.J3=0.005694
        self.J4=-1.3406
        self.J5= -0.0624
        self.J6=1.50808
        self.J7=0.120986

        # Transformers
        self.arm_tf_broadcasterbase0 = tf.TransformBroadcaster()
        self.arm_tf_broadcaster01 = tf.TransformBroadcaster()
        self.arm_tf_broadcaster12 = tf.TransformBroadcaster()
        self.arm_tf_broadcaster23 = tf.TransformBroadcaster()
        self.arm_tf_broadcaster34 = tf.TransformBroadcaster()
        self.arm_tf_broadcaster45 = tf.TransformBroadcaster()
        self.arm_tf_broadcaster56 = tf.TransformBroadcaster()
        self.arm_tf_broadcaster67 = tf.TransformBroadcaster()
        self.arm_tf_broadcaster7ee = tf.TransformBroadcaster()

	self.arm_tf_listener=tf.TransformListener()

###############################################################
# Helper Functions 
###############################################################
    def shutdown_hook(self):
        # Rapid Shutdown
        rospy.logfatal("Shutdown!")

    def quat_to_euler(self, x, y, z, w):
        quat = [x,y,z,w]
        roll,pitch,yaw = tf.transformations.euler_from_quaternion(quat)
        return [roll,pitch,yaw]

    def euler_to_quat(self, r, p, y):
        x,y,z,w = tf.transformations.quaternion_from_euler(r,p,y)
        return (x,y,z,w)

###############################################################
# TF Publishers
###############################################################

    def static_arm_publisher(self):
	# Blue: Z 
	# Red: X
	# Green: Y
        ########################
        # Joint Base0
        ########################
        parent_frame_base0 = "robot_base_link"
        child_frame_base0 = "static_arm_link_0"
        trans_base0 = (0.192,0.0,0.467)
        quat_base0 = self.euler_to_quat(0.0,0.0,0.0)

        self.arm_tf_broadcaster01.sendTransform(trans_base0, 
                                            quat_base0, rospy.Time.now(), child_frame_base0, parent_frame_base0)

        ########################
        # Joint 01
        ########################
        parent_frame_01 = "static_arm_link_0"
        child_frame_01 = "static_arm_link_1"
        trans_01 = (0.0,0.0,0.15)
	# BLUE
        quat_01 = self.euler_to_quat(0.0,0.0,self.J1)

        self.arm_tf_broadcaster01.sendTransform(trans_01, 
                                            quat_01, rospy.Time.now(), child_frame_01, parent_frame_01)
        ########################
        # Joint 12
        ########################
        parent_frame_12 = "static_arm_link_1"
        child_frame_12 = "static_arm_link_2"
        trans_12 = (0.0,0.0,0.190)
	# RED
        quat_12 = self.euler_to_quat(1.571+self.J2,0.0,3.142)

        self.arm_tf_broadcaster12.sendTransform(trans_12, 
                                            quat_12, rospy.Time.now(), child_frame_12, parent_frame_12)
        ########################
        # Joint 23
        ########################
        parent_frame_23 = "static_arm_link_2"
        child_frame_23 = "static_arm_link_3"
        trans_23 = (0.0,0.21,0.0)
	# BLUE
        quat_23 = self.euler_to_quat(1.571,0.0,3.142+self.J3)

        self.arm_tf_broadcaster23.sendTransform(trans_23, 
                                            quat_23, rospy.Time.now(), child_frame_23, parent_frame_23)
        ########################
        # Joint 34
        ########################
        parent_frame_34 = "static_arm_link_3"
        child_frame_34 = "static_arm_link_4"
        trans_34 = (0.0,0.0,0.19)
	# RED
        quat_34 = self.euler_to_quat(1.571+self.J4,0.0,0.0)

        self.arm_tf_broadcaster34.sendTransform(trans_34, 
                                            quat_34, rospy.Time.now(), child_frame_34, parent_frame_34)
        ########################
        # Joint 45
        ########################
        parent_frame_45 = "static_arm_link_4"
        child_frame_45 = "static_arm_link_5"
        trans_45 = (0.0,0.21,0.0)
	# BLUE 
        quat_45 = self.euler_to_quat(-1.571,3.142,self.J5)

        self.arm_tf_broadcaster45.sendTransform(trans_45, 
                                            quat_45, rospy.Time.now(), child_frame_45, parent_frame_45)
        ########################
        # Joint 56
        ########################
        parent_frame_56 = "static_arm_link_5"
        child_frame_56 = "static_arm_link_6"
        trans_56 = (0.0,0.061,0.19)
	# RED
        quat_56 = self.euler_to_quat(1.571+self.J6,0.0,0.0)

        self.arm_tf_broadcaster56.sendTransform(trans_56, 
                                            quat_56, rospy.Time.now(), child_frame_56, parent_frame_56)
        ########################
        # Joint 67
        ########################
        parent_frame_67 = "static_arm_link_6"
        child_frame_67 = "static_arm_link_7"
        trans_67 = (0.0,0.081,-0.0607)
	# BLUE 
        quat_67 = self.euler_to_quat(-1.571,3.142,self.J7)

        self.arm_tf_broadcaster67.sendTransform(trans_67, 
                                            quat_67, rospy.Time.now(), child_frame_67, parent_frame_67)
        ########################
        # Joint 7ee
        ########################
        parent_frame_7ee = "static_arm_link_7"
        child_frame_7ee = "static_arm_link_ee"
        trans_7ee = (0.0,0.0,0.171)
        quat_7ee = self.euler_to_quat(0.0,0.07,0.0)

        self.arm_tf_broadcaster7ee.sendTransform(trans_7ee, 
                                            quat_7ee, rospy.Time.now(), child_frame_7ee, parent_frame_7ee)


    def full_lookup(self):
	########################
	# FULL Lookup, to make a singular static TF for use
	########################
	parent_frame_full = "robot_base_link"
	child_frame_full = "static_arm_link_ee"
        self.arm_tf_listener.waitForTransform(parent_frame_full,child_frame_full,rospy.Time(0),rospy.Duration(0.50))
	(trans,rot) = self.arm_tf_listener.lookupTransform(parent_frame_full, child_frame_full, rospy.Time(0))
	x,y,z,w = rot
	[roll,pitch,yaw] = self.quat_to_euler(x, y, z, w)
	print(trans,[roll,pitch,yaw])

if __name__ == '__main__':
    rospy.init_node("static_kuka_tf_publisher",anonymous=True, log_level=rospy.DEBUG)
    arm_tf = StaticArm()

    while not rospy.is_shutdown():
        try:
            arm_tf.static_arm_publisher()
	    try:
	    	arm_tf.full_lookup()
	    except:
		print("lookup failed")
	    	arm_tf.rate.sleep()
        except rospy.ROSInterruptException:
            arm_tf.shutdown_hook()
