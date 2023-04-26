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

class ZedTracker():

    def __init__(self, loop_rate=10000.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)

        # Transformers
        self.tf_listener=tf.TransformListener()
        self.tf_debugger=tf.TransformListener()
        self.zed_transformer = tf.TransformBroadcaster()

        # Initial arm pose containers 
        self.arm_ee_x = 0
        self.arm_ee_y = 0
        self.arm_ee_z = 0
        self.arm_ee_roll = 0
        self.arm_ee_pitch = 0
        self.arm_ee_yaw = 0

        # Initialized?
        self.initialized = False

###############################################################
# Helper Functions 
###############################################################
    def angle_wrap(self, ang):
        # Normalize angle between -pi and pi
        ang = ang % (2 * np.pi)
        if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
            ang -= 2 * np.pi
        elif isinstance(ang, np.ndarray):
            ang[ang > np.pi] -= 2 * np.pi
        return ang

    def quat_to_euler(self, x, y, z, w):
        quat = [x,y,z,w]
        roll,pitch,yaw = tf.transformations.euler_from_quaternion(quat)
        return [roll,pitch,yaw]

    def euler_to_quat(self, r, p, y):
        x,y,z,w = tf.transformations.quaternion_from_euler(r,p,y)
        return [x,y,z,w]

#########################################################################
# Listener 
#########################################################################
    def static_arm_ee_listener(self):
        parent = "world"
        child = "zed_camera_link"

        self.tf_listener.waitForTransform(child,parent,rospy.Time(0),rospy.Duration(0.5))
        (trans,rot)=self.tf_listener.lookupTransform(parent,child,rospy.Time(0))

        roll,pitch,yaw = self.quat_to_euler(rot[0],rot[1],rot[2],rot[3])
        x,y,z = trans
        
        self.arm_ee_x = round(x,3)
        self.arm_ee_roll = round(roll,3)
        self.arm_ee_pitch = round(pitch,3)
        self.arm_ee_yaw = round(yaw,3)

        # Don't want updates in z direction. 
        if not self.initialized:
            self.arm_ee_z = round(z,3)-0.014

            self.initialized = True

        #print("xyz",self.arm_ee_x, self.arm_ee_y, self.arm_ee_z)
        #print("rpy",self.arm_ee_roll,self.arm_ee_pitch,self.arm_ee_yaw)

#########################################################################
# Publisher 
#########################################################################
    def static_arm_publisher(self):
        parent_frame = "world"
        child_frame = "static_arm_ee"

        # Pulling from odom.
        roll,pitch,yaw = self.arm_ee_roll,self.arm_ee_pitch,self.arm_ee_yaw
        #yaw = 3.14
        #print("rpy",roll,pitch,yaw)
        ox,oy,oz,ow = self.euler_to_quat(roll,pitch,yaw)
        x,y,z = self.arm_ee_x,self.arm_ee_y,self.arm_ee_z
        
        self.zed_transformer.sendTransform((x,y,z), 
                                            (ox,oy,oz,ow), rospy.Time.now(), child_frame, parent_frame)

##########################################################################
# Debugger 
#########################################################################

    def debugger(self):
        parent = "world"
        child = "static_arm_ee"
        try:
            self.tf_listener.waitForTransform(child,parent,rospy.Time(0),rospy.Duration(0.5))
            (trans,rot)=self.tf_listener.lookupTransform(child,parent,rospy.Time(0))
            #(trans,rot)=self.tf_listener.lookupTransform(parent,child,rospy.Time(0))

            roll,pitch,yaw = self.quat_to_euler(rot[0],rot[1],rot[2],rot[3])
            x,y,z = trans

            ########### DEBUG ##########
            x1,y1,z1 = trans
            (trans,rot)=self.tf_listener.lookupTransform(parent,child,rospy.Time(0))
            #(trans,rot)=self.tf_listener.lookupTransform(child,parent,rospy.Time(0))
            x2,y2,z2 = trans
            roll,pitch,yaw = self.quat_to_euler(rot[0],rot[1],rot[2],rot[3])
            print("xx2c",x1,x2)
            #print("RPY",roll,pitch,yaw)
        except: 
            print("debug tf failed.")

if __name__ == '__main__':
    rospy.init_node("zed2i_tracker",anonymous=True, log_level=rospy.INFO)
    zed_tf= ZedTracker()

    while not rospy.is_shutdown():
        try:
            zed_tf.static_arm_ee_listener()
            zed_tf.static_arm_publisher()
            #zed_tf.debugger()
            zed_tf.rate.sleep()
        except rospy.ROSInterruptException:
            zed_tf.shutdown_hook()