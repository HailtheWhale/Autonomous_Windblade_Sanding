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

class WorldPub():

    def __init__(self, loop_rate=10000.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)
        # Node name 
        self.node_name = "world_frame_publisher/"
        # Parameters
        # Define number of measurements must be taken before the 
        # ArUco Marker frames will be set
        self.min_frames = int(rospy.get_param(self.node_name + "min_frames"))
        # Define topics 
        self.fiducial_tf_topic = str(rospy.get_param(self.node_name + "fiducial_tf_topic"))
        # Define odometry TF frame 
        self.odom_frame = str(rospy.get_param(self.node_name + "odom_frame"))

        # Transformers
        self.world_tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener=tf.TransformListener()

        # Subscribers 
        self.aruco_tf_sub= rospy.Subscriber(self.fiducial_tf_topic,FiducialTransformArray,self.aruco_tf)

        # World TF containers 
        self.world_tf_pose_x = []
        self.world_tf_pose_y = []
        self.world_tf_orient_x = 0
        self.world_tf_orient_y = 0
        self.world_tf_orient_z = []
        self.world_tf_orient_w = 0

        # To keep looping until done.
        self.world_aruco_determined = False 


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

###############################################################
# ArUco node handling
##############################################################
    def kill_qr(self):
        subprocess.call(["rosnode kill /aruco_detect"], shell=True)
        rospy.logwarn("aruco_detect node killed!")

###############################################################
# Subscriber Functions
###############################################################
## Interpreting Incoming ArUco markers.
    def aruco_tf(self,msg):
    ##### World Frame check  
        # Find Marker coordinates 
        # World Frame Check 
        world_frame = "/fiducial_0"
        try: 
            # TF From the odometry to the Waypoint 0 ArUco Marker 
            self.tf_listener.waitForTransform(world_frame,self.odom_frame,rospy.Time(0),rospy.Duration(0.5))
            rospy.loginfo_once("World Frame Found! Recording....")

            # TF From the base_link to the qr code
            (trans,rot)=self.tf_listener.lookupTransform(self.odom_frame,world_frame,rospy.Time(0))

            # Copying values.Most distortion in roll and pitch, so discarding. 
            xw,yw,zw = trans
            oxw,oyw,ozw,oww = rot
            roll,pitch,yaw = self.quat_to_euler(oxw,oyw,ozw,oww)
            yaw+=3.14
            # There seems to be some timeout involved in the tf frame of the ArUco code. Need to account for 
            # This to ensure values are not biased. 
            if len(self.world_tf_pose_x) > 1 and xw == self.world_tf_pose_x[-1]: 
                pass
            else:
                if len(self.world_tf_pose_x) == self.min_frames:
                    rospy.loginfo("Averaging recorded values...")
                    avg_world_tf_pose_x = 0
                    avg_world_tf_pose_y = 0
                    avg_world_tf_orient_z = 0
                    # Summations 
                    for i in range(0,len(self.world_tf_pose_x)):
                        avg_world_tf_pose_x+=self.world_tf_pose_x[i]
                        avg_world_tf_pose_y+=self.world_tf_pose_y[i]
                        avg_world_tf_orient_z+=self.world_tf_orient_z[i]
                    # Avgs
                    avg_world_tf_pose_x/=(len(self.world_tf_pose_x))
                    avg_world_tf_pose_y/=(len(self.world_tf_pose_x))
                    avg_world_tf_orient_z/=(len(self.world_tf_pose_x))
                    # Convert from Euler to Quat 
                    oxw,oyw,ozw,oww= self.euler_to_quat(0,0,avg_world_tf_orient_z)
                    # Save values                         
                    self.world_tf_pose_x = avg_world_tf_pose_x
                    self.world_tf_pose_y = avg_world_tf_pose_y
                    self.world_tf_orient_x = oxw
                    self.world_tf_orient_y = oyw
                    self.world_tf_orient_z = ozw
                    self.world_tf_orient_w = oww
                    # Waypoint 1 now set 
                    rospy.loginfo("World Frame ArUco position calculated!")
                    self.world_aruco_determined = True 
                else:
                    self.world_tf_pose_x.append(xw)
                    self.world_tf_pose_y.append(yw)
                    self.world_tf_orient_z.append(yaw)

        except:
            rospy.logdebug("World Frame not in View!")                    

#########################################################################
# Publishers 
#########################################################################

    def world_publisher(self):
        # Sends the QR Code TF
        parent_frame = self.odom_frame
        child_frame = "world"
        self.world_tf_broadcaster.sendTransform((self.world_tf_pose_x, self.world_tf_pose_y, 0), 
                                            (self.world_tf_orient_x,self.world_tf_orient_y,self.world_tf_orient_z,
                                            self.world_tf_orient_w), rospy.Time.now(), child_frame, parent_frame)
    
    def loop(self):
        while not self.world_aruco_determined:
            pass

##########################################################################

if __name__ == '__main__':
    rospy.init_node("world_frame_publisher",anonymous=True, log_level=rospy.INFO)
    world_tf = WorldPub()
    world_tf.loop()

    while not rospy.is_shutdown():
        try:
            world_tf.world_publisher()
	    world_tf.rate.sleep()
        except rospy.ROSInterruptException:
            world_tf.shutdown_hook()