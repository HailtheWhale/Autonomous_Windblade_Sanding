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

    def __init__(self, loop_rate=5.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)
        rospy.on_shutdown(self.shutdown_hook)
        rospy.wait_for_message('/odom',Odometry)
        # Node name
        self.node_name ="world_frame_publisher/"
        # Parameters
        # Define save directory 
        self.save_dir = str(rospy.get_param("experiment_save_directory"))
        # Define test being performed 
        self.tf_test = int(rospy.get_param(self.node_name + "tf_test"))
        # Define distance from marker for waypoint 
        # Run 6 experiments. dist = 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8
        self.waypoint_dist = float(rospy.get_param(self.node_name + "waypoint_dist"))
        # Define number of measurements must be taken before the 
        # ArUco Marker frames will be set
        self.min_frames = int(rospy.get_param(self.node_name + "min_frames"))
        # Define angle tolerance 
        self.waypoint_ang_tol = float(rospy.get_param(self.node_name + "waypoint_ang_tol"))
        # Define Waypoint distance Tolerance 
        self.waypoint_dist_tol = float(rospy.get_param(self.node_name + "waypoint_dist_tol"))
        # Define topics 
        self.odom_topic = str(rospy.get_param(self.node_name + "odom_topic"))
        self.fiducial_tf_topic = str(rospy.get_param(self.node_name + "fiducial_tf_topic"))
        self.cmd_vel_topic = str(rospy.get_param(self.node_name + "cmd_vel_topic"))

        # Transformers
        self.world_tf_broadcaster0 = tf.TransformBroadcaster()
        self.world_tf_broadcaster1 = tf.TransformBroadcaster()
        self.world_tf_broadcaster2 = tf.TransformBroadcaster()

        self.waypoint1_broadcaster = tf.TransformBroadcaster()
        self.waypoint2_broadcaster = tf.TransformBroadcaster()

        self.aruco_broadcaster = tf.TransformBroadcaster()
        self.intermediate_broadcaster = tf.TransformBroadcaster()

        self.tf_listener=tf.TransformListener()

        # Subscribers 
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.read_position)
        self.aruco_tf_sub= rospy.Subscriber(self.fiducial_tf_topic,FiducialTransformArray,self.aruco_tf)
        # Publishers 
        self.cmd_vel_pub= rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 10)

        # Orientation
        self.orient_theta = 0 
        self.pose_x_base = 0
        self.pose_y_base = 0

        # Movement 
        self.cmd_vel = Twist()
        self.cmd_vel.angular.x=0
        self.cmd_vel.angular.y=0
        self.cmd_vel.angular.z=0
        self.cmd_vel.linear.x=0
        self.cmd_vel.linear.y=0
        self.cmd_vel.linear.z=0

        # Turning and linear spds 
        self.turn_spd = 0.13
        self.linear_spd = 0.05

        # Finished turning
        self.finished_turning = False
        # Started rotation
        self.started_rot = False

        # Detected Codes?
        # Determined Markers?
        self.world_aruco_determined = False
        self.waypoint1_aruco_determined = False
        self.waypoint2_aruco_determined = False
        # Set Waypoint markers?
        self.world_aruco_saved = False
        self.world_aruco_set_0 = False
        self.world_aruco_set_1 = False
        self.world_aruco_set_2 = False

        self.waypoint1_aruco_set = False
        self.waypoint2_aruco_set = False

        # World TF containers 
        self.world_tf_pose_x = []
        self.world_tf_pose_y = []
        self.world_tf_orient_x = 0
        self.world_tf_orient_y = 0
        self.world_tf_orient_z = []
        self.world_tf_orient_w = 0

        self.world_tf_pose_x0 = 0
        self.world_tf_pose_y0 = 0
        self.world_tf_orient_x0 = 0
        self.world_tf_orient_y0 = 0
        self.world_tf_orient_z0 = 0
        self.world_tf_orient_w0 = 0

        self.world_tf_pose_x1 = 0
        self.world_tf_pose_y1 = 0
        self.world_tf_orient_x1 = 0
        self.world_tf_orient_y1 = 0
        self.world_tf_orient_z1 = 0
        self.world_tf_orient_w1 = 0

        self.world_tf_pose_x2 = 0
        self.world_tf_pose_y2 = 0
        self.world_tf_orient_x2 = 0
        self.world_tf_orient_y2 = 0
        self.world_tf_orient_z2 = 0
        self.world_tf_orient_w2 = 0

        # Waypoint TF containers 
        self.waypoint_tf_pose_x1 = []
        self.waypoint_tf_pose_y1 = []
        self.waypoint_tf_orient_x1 = 0
        self.waypoint_tf_orient_y1 = 0
        self.waypoint_tf_orient_z1 = []
        self.waypoint_tf_orient_w1 = 0

        self.waypoint_tf_pose_x2 = []
        self.waypoint_tf_pose_y2 = []
        self.waypoint_tf_orient_x2 = 0
        self.waypoint_tf_orient_y2 = 0
        self.waypoint_tf_orient_z2 = []
        self.waypoint_tf_orient_w2 = 0

        # Waypoint on 
        self.waypoint = 0
        self.waypoint_cnt = 0
        # Waypoint x,y containers 
        self.bearing_x = 0
        self.bearing_y = 0
        # Close enough to waypoint to do measurements?
        self.close_enough_dist = 0.7
        self.close_enough_1 = False
        self.close_enough_2 = False
        self.moving_close_enough = False
        # Transit Close enough Containers 
        self.transit_tf_pose_x = 0
        self.transit_tf_pose_y = 0
        self.transit_tf_orient_x = 0
        self.transit_tf_orient_y = 0
        self.transit_tf_orient_z = 0
        self.transit_tf_orient_w = 0

###############################################################
# Helper Functions 
###############################################################
    def shutdown_hook(self):
        # Rapid Shutdown
        rospy.logfatal("Shutdown!")
        # Stop robot 
        self.cmd_vel.angular.z = 0
        self.cmd_vel.linear.x = 0
        self.cmd_vel_pub.publish(self.cmd_vel)
        rospy.signal_shutdown(True)

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

## Retrieving Odometry information
    def read_position(self,msg):
        # Saving x,y positions 
        self.pose_x_base = msg.pose.pose.position.x
        self.pose_y_base = msg.pose.pose.position.y

        # Getting quaternions
        quat_x = msg.pose.pose.orientation.x
        quat_y = msg.pose.pose.orientation.y
        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w
        rpy = self.quat_to_euler(quat_x,quat_y,quat_z,quat_w)

        # Saving yaw.
        self.orient_theta = rpy[2]
        # Normalizing the retrieved angle between -pi and pi.
        self.orient_theta = self.angle_wrap(self.orient_theta)
        # Normalizing between 0 and 2pi
        if self.orient_theta < 0:
            self.orient_theta += 6.28
    
## Interpreting Incoming ArUco markers.
    def aruco_tf(self,msg):
        # TF publishers. Put here to avoid stalling the publications and breaking everything. 
        self.world_publisher()
        self.waypoint_publisher()
        # To get close enough to the marker if not done so already. 
        if abs(self.waypoint) == 1 and not self.close_enough_1:
            self.close_enough_publisher(marker_seeking="/fiducial_1")
        if abs(self.waypoint) == 2 and not self.close_enough_2:
            self.close_enough_publisher(marker_seeking="/fiducial_2")

        ## If haven't found the waypoints yet, look for them 
    ##### Waypoint 1 check  
        # Find Marker coordinates 
        if self.close_enough_1 and not self.waypoint1_aruco_determined:
            # Waypoint 1 Frame Check 
            waypoint_1_frame = "/fiducial_1"
            try: 
                # TF From the odometry to the Waypoint 1 ArUco Marker 
                self.tf_listener.waitForTransform(waypoint_1_frame,"/odom",rospy.Time(0),rospy.Duration(0.5))

                # TF From the base_link to the qr code
                (trans,rot)=self.tf_listener.lookupTransform("/odom",waypoint_1_frame,rospy.Time(0))

                # Copying values.Most distortion in roll and pitch, so discarding. 
                x1,y1,z1 = trans
                ox1,oy1,oz1,ow1 = rot
                roll,pitch,yaw = self.quat_to_euler(ox1,oy1,oz1,ow1)
                yaw+=3.14
                # There seems to be some timeout involved in the tf frame of the ArUco code. Need to account for 
                # This to ensure values are not biased. 
                if len(self.waypoint_tf_pose_x1) > 1 and x1 == self.waypoint_tf_pose_x1[-1]: 
                    pass
                else:
                    if len(self.waypoint_tf_pose_x1) >= self.min_frames:
                        rospy.loginfo("Averaging recorded values...")
                        avg_waypoint_tf_pose_x1 = 0
                        avg_waypoint_tf_pose_y1 = 0
                        avg_waypoint_tf_orient_z1 = 0
                        # Summations 
                        for i in range(0,len(self.waypoint_tf_pose_x1)):
                            avg_waypoint_tf_pose_x1+=self.waypoint_tf_pose_x1[i]
                            avg_waypoint_tf_pose_y1+=self.waypoint_tf_pose_y1[i]
                            avg_waypoint_tf_orient_z1+=self.waypoint_tf_orient_z1[i]

                        # Avgs
                        avg_waypoint_tf_pose_x1/=(len(self.waypoint_tf_pose_x1))
                        avg_waypoint_tf_pose_y1/=(len(self.waypoint_tf_pose_x1))
                        avg_waypoint_tf_orient_z1/=(len(self.waypoint_tf_pose_x1))
                        # Convert from Euler to Quat 
                        ox1,oy1,oz1,ow1= self.euler_to_quat(0,0,avg_waypoint_tf_orient_z1)
                        # Save values 
                        self.waypoint_tf_pose_x1 = avg_waypoint_tf_pose_x1
                        self.waypoint_tf_pose_y1 = avg_waypoint_tf_pose_y1
                        self.waypoint_tf_orient_x1 = ox1
                        self.waypoint_tf_orient_y1 = oy1
                        self.waypoint_tf_orient_z1 = oz1
                        self.waypoint_tf_orient_w1 = ow1
                        # Waypoint 1 now set 
                        rospy.logfatal("Waypoint 1 ArUco position calculated!")
                        self.waypoint1_aruco_determined = True 
                    else:
                        self.waypoint_tf_pose_x1.append(x1)
                        self.waypoint_tf_pose_y1.append(y1)
                        self.waypoint_tf_orient_z1.append(yaw)

            except:
                rospy.logdebug("Waypoint 1 Frame not in View!")
                rospy.loginfo("Frames found: %s", str(len(self.waypoint_tf_pose_x1)))
                    
        # Else, already have the Tfs needed to publish waypoint 1. 
        # Publish it.
        elif self.close_enough_1:
            # If Not set up the offset waypoint yet, do it. 
            if not self.waypoint1_aruco_set:
                # Recreate ArUco Marker 
                parent_frame = "/odom"
                child_frame ="/aruco_1"
                self.aruco_broadcaster.sendTransform((self.waypoint_tf_pose_x1,self.waypoint_tf_pose_y1,0), 
                (self.waypoint_tf_orient_x1,self.waypoint_tf_orient_y1,self.waypoint_tf_orient_z1,self.waypoint_tf_orient_w1), 
                rospy.Time.now(), child_frame, parent_frame)
                # Perform an offset from the marker 
                self.tf_listener.waitForTransform(child_frame,parent_frame,rospy.Time(0),rospy.Duration(0.5))
                parent_frame = "/aruco_1"
                child_frame = "/waypoint_1"
                self.intermediate_broadcaster.sendTransform((0,self.waypoint_dist,0), (0,0,0,1), rospy.Time.now(), child_frame, parent_frame)
                # Lookup the new transfrom from the odom to the waypoint. Update the waypoint info. 
                try: 
                    [trans,rot]=self.tf_listener.lookupTransform("/odom","/waypoint_1",rospy.Time(0))
                    x1,y1,z1 = trans
                    ox1,oy1,oz1,ow1 = rot
                    self.waypoint_tf_pose_x1 = x1
                    self.waypoint_tf_pose_y1 = y1
                    self.waypoint_tf_orient_x1 = ox1
                    self.waypoint_tf_orient_y1 = oy1
                    self.waypoint_tf_orient_z1 = oz1
                    self.waypoint_tf_orient_w1 = ow1
                    rospy.logfatal("Waypoint 1 set!")
                    self.waypoint1_aruco_set = True
                    rospy.loginfo("Transform to waypoint 1 successful!")
                except:
                    rospy.logwarn("Transform to waypoint 1 failed! Retrying...")

    ##### Waypoint 2 check  
        # Find Marker coordinates 
        if self.close_enough_2 and not self.waypoint2_aruco_determined:
            # Waypoint 1 Frame Check 
            waypoint_2_frame = "/fiducial_2"
            try: 
                # TF From the odometry to the Waypoint 1 ArUco Marker 
                self.tf_listener.waitForTransform(waypoint_2_frame,"/odom",rospy.Time(0),rospy.Duration(0.5))

                # TF From the base_link to the qr code
                (trans,rot)=self.tf_listener.lookupTransform("/odom",waypoint_2_frame,rospy.Time(0))

                # Copying values.Most distortion in roll and pitch, so discarding. 
                x2,y2,z2 = trans
                ox2,oy2,oz2,ow2 = rot
                roll,pitch,yaw = self.quat_to_euler(ox2,oy2,oz2,ow2)
                yaw+=3.14
                # There seems to be some timeout involved in the tf frame of the ArUco code. Need to account for 
                # This to ensure values are not biased. 
                if len(self.waypoint_tf_pose_x2) > 1 and x2 == self.waypoint_tf_pose_x2[-1]: 
                    pass
                else:
                    if len(self.waypoint_tf_pose_x2) == self.min_frames:
                        rospy.loginfo("Averaging recorded values...")
                        avg_waypoint_tf_pose_x2 = 0
                        avg_waypoint_tf_pose_y2 = 0
                        avg_waypoint_tf_orient_z2 = 0
                        # Summations 
                        for i in range(0,len(self.waypoint_tf_pose_x2)):
                            avg_waypoint_tf_pose_x2+=self.waypoint_tf_pose_x2[i]
                            avg_waypoint_tf_pose_y2+=self.waypoint_tf_pose_y2[i]
                            avg_waypoint_tf_orient_z2+=self.waypoint_tf_orient_z2[i]
                        # Avgs
                        avg_waypoint_tf_pose_x2/=(len(self.waypoint_tf_pose_x2))
                        avg_waypoint_tf_pose_y2/=(len(self.waypoint_tf_pose_x2))
                        avg_waypoint_tf_orient_z2/=(len(self.waypoint_tf_pose_x2))
                        # Convert from Euler to Quat 
                        ox2,oy2,oz2,ow2= self.euler_to_quat(0,0,avg_waypoint_tf_orient_z2)
                        # Save values 
                        self.waypoint_tf_pose_x2 = avg_waypoint_tf_pose_x2
                        self.waypoint_tf_pose_y2 = avg_waypoint_tf_pose_y2
                        self.waypoint_tf_orient_x2 = ox2
                        self.waypoint_tf_orient_y2 = oy2
                        self.waypoint_tf_orient_z2 = oz2
                        self.waypoint_tf_orient_w2 = ow2
                        # Waypoint 1 now set 
                        rospy.logfatal("Waypoint 2 ArUco position calculated!")
                        self.waypoint2_aruco_determined = True 
                    else:
                        self.waypoint_tf_pose_x2.append(x2)
                        self.waypoint_tf_pose_y2.append(y2)
                        self.waypoint_tf_orient_z2.append(yaw)

            except:
                rospy.logdebug("Waypoint 2 Frame not in View!")
                rospy.loginfo("Frames found: %s", str(len(self.waypoint_tf_pose_x2)))
                    
        # Else, already have the Tfs needed to publish waypoint 2. 
        # Publish it.
        elif self.close_enough_2:
            # If Not set up the offset waypoint yet, do it. 
            if not self.waypoint2_aruco_set:
                # Recreate ArUco Marker 
                parent_frame = "/odom"
                child_frame ="/aruco_2"
                self.aruco_broadcaster.sendTransform((self.waypoint_tf_pose_x2,self.waypoint_tf_pose_y2,0), 
                (self.waypoint_tf_orient_x2,self.waypoint_tf_orient_y2,self.waypoint_tf_orient_z2,self.waypoint_tf_orient_w2), 
                rospy.Time.now(), child_frame, parent_frame)
                # Perform an offset from the marker 
                self.tf_listener.waitForTransform(child_frame,parent_frame,rospy.Time(0),rospy.Duration(0.5))
                parent_frame = "/aruco_2"
                child_frame = "/waypoint_2"
                self.intermediate_broadcaster.sendTransform((0,self.waypoint_dist,0), (0,0,0,1), rospy.Time.now(), child_frame, parent_frame)
                # Lookup the new transfrom from the odom to the waypoint. Update the waypoint info. 
                try: 
                    [trans,rot]=self.tf_listener.lookupTransform("/odom","/waypoint_2",rospy.Time(0))
                    x2,y2,z2 = trans
                    ox2,oy2,oz2,ow2 = rot
                    self.waypoint_tf_pose_x2 = x2
                    self.waypoint_tf_pose_y2 = y2
                    self.waypoint_tf_orient_x2 = ox2
                    self.waypoint_tf_orient_y2 = oy2
                    self.waypoint_tf_orient_z2 = oz2
                    self.waypoint_tf_orient_w2 = ow2
                    rospy.logfatal("Waypoint 2 set!")
                    self.waypoint2_aruco_set = True
                    rospy.loginfo("Transform to waypoint 2 successful!")
                except:
                    rospy.logwarn("Transform to waypoint 2 failed! Retrying...")
 
    ##### World Frame check  
        # Find Marker coordinates 
        if not self.world_aruco_determined:
            # World Frame Check 
            world_frame = "/fiducial_0"
            try: 
                # TF From the odometry to the Waypoint 0 ArUco Marker 
                self.tf_listener.waitForTransform(world_frame,"/odom",rospy.Time(0),rospy.Duration(0.5))
                rospy.loginfo_once("World Frame Found! Recording....")

                # TF From the base_link to the qr code
                (trans,rot)=self.tf_listener.lookupTransform("/odom",world_frame,rospy.Time(0))

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
                        rospy.logfatal("SAVING>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                        self.data_saver(header = False, data_x=self.world_tf_pose_x,data_y=self.world_tf_pose_y,data_yaw=self.world_tf_orient_z, world_frame=abs(self.waypoint))
                        
                        self.world_tf_pose_x = avg_world_tf_pose_x
                        self.world_tf_pose_y = avg_world_tf_pose_y
                        self.avg_world_tf_orient_x = oxw
                        self.avg_world_tf_orient_y = oyw
                        self.avg_world_tf_orient_z = ozw
                        self.avg_world_tf_orient_w = oww
                        # Waypoint 1 now set 
                        rospy.loginfo("World Frame ArUco position calculated!")
                        self.world_aruco_determined = True 
                    else:
                        self.world_tf_pose_x.append(xw)
                        self.world_tf_pose_y.append(yw)
                        self.world_tf_orient_z.append(yaw)

            except:
                rospy.logdebug("World Frame not in View!")
                rospy.loginfo("Frames found: %s", str(len(self.world_tf_pose_x)))
                    
        # Else, Copy over the world TF data to the appropriate container.
        else:
            if not self.world_aruco_saved:
                    if abs(self.waypoint) == 0:
                        self.world_tf_pose_x0 = self.world_tf_pose_x 
                        self.world_tf_pose_y0 = self.world_tf_pose_y
                        self.waypoint_tf_orient_x0 = self.avg_world_tf_orient_x
                        self.waypoint_tf_orient_y0 = self.avg_world_tf_orient_y
                        self.waypoint_tf_orient_z0 = self.avg_world_tf_orient_z
                        self.waypoint_tf_orient_w0 = self.avg_world_tf_orient_w
                        self.world_aruco_set_0 = True
                    elif abs(self.waypoint) == 1:
                        self.world_tf_pose_x1 = self.world_tf_pose_x 
                        self.world_tf_pose_y1 = self.world_tf_pose_y
                        self.waypoint_tf_orient_x1 = self.avg_world_tf_orient_x
                        self.waypoint_tf_orient_y1 = self.avg_world_tf_orient_y
                        self.waypoint_tf_orient_z1 = self.avg_world_tf_orient_z
                        self.waypoint_tf_orient_w1 = self.avg_world_tf_orient_w
                        self.world_aruco_set_1 = True
                    elif abs(self.waypoint) == 2:
                        self.world_tf_pose_x2 = self.world_tf_pose_x 
                        self.world_tf_pose_y2 = self.world_tf_pose_y
                        self.waypoint_tf_orient_x2 = self.avg_world_tf_orient_x
                        self.waypoint_tf_orient_y2 = self.avg_world_tf_orient_y
                        self.waypoint_tf_orient_z2 = self.avg_world_tf_orient_z
                        self.waypoint_tf_orient_w2 = self.avg_world_tf_orient_w
                        self.world_aruco_set_2 = True
                    else:
                        rospy.logfatal("OUTSIDE OF EXPECTED WAYPOINT RANGE!")
                    # Waypoint 1 now set 
                    rospy.loginfo("World Frame ArUco position calculated!")
                    # Reset object containers. 
                    self.world_tf_pose_x = []
                    self.world_tf_pose_y = []
                    self.world_tf_orient_x = 0
                    self.world_tf_orient_y = 0
                    self.world_tf_orient_z = []
                    self.world_tf_orient_w = 0
                    # If test type 1: Go world frame 0, then 1, then 2. 
                    if self.tf_test == 1:
                        self.waypoint+=1
                    # If test type 2: Go world frame 0, then 2, then 1.
                    else:
                        if self.waypoint == 0:
                            self.waypoint = 2
                        else:
                            self.waypoint = 1
                    # Update counter
                    self.waypoint_cnt += 1
                    self.world_aruco_saved = True

#########################################################################
# Publishers 
#########################################################################

    def world_publisher(self):
        # Publishes based on if the frame is available or not. 
        if self.world_aruco_set_0:
            # Sends the QR Code TF
            parent_frame = "odom"
            child_frame = "world_0"
            self.world_tf_broadcaster0.sendTransform((self.world_tf_pose_x0, self.world_tf_pose_y0, 0), 
                                                (self.waypoint_tf_orient_x0,self.waypoint_tf_orient_y0,self.waypoint_tf_orient_z0,
                                                self.waypoint_tf_orient_w0), rospy.Time.now(), child_frame, parent_frame)
        if self.world_aruco_set_1:
            # Sends the QR Code TF
            parent_frame = "odom"
            child_frame = "world_1"
            self.world_tf_broadcaster1.sendTransform((self.world_tf_pose_x1, self.world_tf_pose_y1, 0), 
                                                (self.waypoint_tf_orient_x1,self.waypoint_tf_orient_y1,self.waypoint_tf_orient_z1,
                                                self.waypoint_tf_orient_w1), rospy.Time.now(), child_frame, parent_frame)
        if self.world_aruco_set_2:
            # Sends the QR Code TF
            parent_frame = "odom"
            child_frame = "world_2"
            self.world_tf_broadcaster2.sendTransform((self.world_tf_pose_x2, self.world_tf_pose_y2, 0), 
                                                (self.waypoint_tf_orient_x2,self.waypoint_tf_orient_y2,self.waypoint_tf_orient_z2,
                                                self.waypoint_tf_orient_w2), rospy.Time.now(), child_frame, parent_frame)
    def waypoint_publisher(self):
        # Publishes the 2 waypoints if they have been calculated. 
        if self.waypoint1_aruco_set:
                parent_frame = "/odom"
                child_frame = "/waypoint_1"
                self.waypoint1_broadcaster.sendTransform((self.waypoint_tf_pose_x1,self.waypoint_tf_pose_y1,0),
                (self.waypoint_tf_orient_x1,self.waypoint_tf_orient_y1,self.waypoint_tf_orient_z1,self.waypoint_tf_orient_w1),
                rospy.Time.now(), child_frame, parent_frame)

        if self.waypoint2_aruco_set:
                parent_frame = "/odom"
                child_frame = "/waypoint_2"
                self.waypoint1_broadcaster.sendTransform((self.waypoint_tf_pose_x2,self.waypoint_tf_pose_y2,0),
                (self.waypoint_tf_orient_x2,self.waypoint_tf_orient_y2,self.waypoint_tf_orient_z2,self.waypoint_tf_orient_w2),
                rospy.Time.now(), child_frame, parent_frame)    

    def close_enough_publisher(self,marker_seeking):
        # If not flagged close eough to the marker yet 
        if marker_seeking == "/fiducial_1" and not self.close_enough_1:
            try: 
                [trans,rot]=self.tf_listener.lookupTransform("/odom",marker_seeking,rospy.Time(0))
                x, y, z = trans
                ox,oy,oz,ow = rot
                # Pass lookup through odometry so that odometry can be considered for distance comparisons. 
                # Sends the QR Code TF
                parent_frame = "odom"
                child_frame = "aruco_seeking"
                self.aruco_broadcaster.sendTransform((x, y, 0), 
                                                    (ox,oy,oz,ow), rospy.Time.now(), child_frame, parent_frame)  
            except:
                rospy.logwarn("Lookup to 'Close Enough Marker 1' ArUco Failed! Trying again...")

        elif marker_seeking == "/fiducial_2" and not self.close_enough_2:
            try: 
                [trans,rot]=self.tf_listener.lookupTransform("/odom",marker_seeking,rospy.Time(0))
                x, y, z = trans
                ox,oy,oz,ow = rot
                # Pass lookup through odometry so that odometry can be considered for distance comparisons. 
                # Sends the QR Code TF
                parent_frame = "odom"
                child_frame = "aruco_seeking"
                self.aruco_broadcaster.sendTransform((x, y, 0), 
                                                    (ox,oy,oz,ow), rospy.Time.now(), child_frame, parent_frame)  
            except:
                rospy.logwarn("Lookup to 'Close Enough Marker 2' ArUco Failed! Trying again...")
        else:
            rospy.logwarn("No listed marker found! Marker provided was %s. Close enough to Marker 1? %s. Marker 2? %s", marker_seeking, self.close_enough_1, self.close_enough_2)

    ###############################################
    # Experimentation Movements 
    ###############################################            

    def spin(self, frame_seeking="world"):
        # Reset Spin variables 
        self.finished_turning = False
        self.started_rot = False

        turn_spd = -self.turn_spd

        while not self.started_rot:
            self.cmd_vel.angular.z = turn_spd
            self.cmd_vel_pub.publish(self.cmd_vel)
            #rospy.logdebug("Starting Rotation...")
            # rospy.logdebug(self.orient_theta)
            if (self.orient_theta >= 0.15 and self.orient_theta <= 6.18):
                self.started_rot = True

        while not self.finished_turning:
            #rospy.logdebug("NOT DONE SPINNING")

            # Rotate
            self.cmd_vel.angular.z = turn_spd
            self.cmd_vel_pub.publish(self.cmd_vel)

            # When not in transit. Usual Case. 
            if frame_seeking == "world":
                # Found code. Reorient to start position. 
                if self.world_aruco_determined:
                    rospy.loginfo_once("World frame found!")

                    # Wait till reoriented.
                    if (self.orient_theta >= 0 and self.orient_theta <= 0.1 or self.orient_theta >= 6.18 and self.orient_theta <= 6.28):
                        self.finished_turning = True
                        rospy.loginfo("World frame set!")
                        # Kill the node to free up resources. Done with it. 
                        #self.kill_qr()

            # In transit to waypoints 
            elif frame_seeking == 1:
                try:
                    self.tf_listener.waitForTransform("/fiducial_1","/odom",rospy.Time(0),rospy.Duration(0.5))
                    self.finished_turning = True
                except:
                    rospy.logwarn("Frame 1 not in view!")

            elif frame_seeking == 2:
                try:
                    self.tf_listener.waitForTransform("/fiducial_2","/odom",rospy.Time(0),rospy.Duration(0.5))
                    self.finished_turning = True
                except:
                    rospy.logwarn("Frame 2 not in view!")
            
            # Trying to calculate waypoints 
            elif frame_seeking == "waypoint_1":
                if self.waypoint1_aruco_determined:
                    rospy.loginfo_once("Waypoint 1 found!")
                    self.finished_turning = True

            elif frame_seeking == "waypoint_2":
                if self.waypoint2_aruco_determined:
                    rospy.loginfo_once("Waypoint 2 found!")
                    self.finished_turning = True

        # Stop robot 
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)

    def traveler(self, found_bearing = False, close_enough = True, close_enough_dist = 0.25):
        while not found_bearing:
            # Retrieve x,y distances based on whichever waypoint goal seeking...
            try:
                lookup = self.waypoint_bearing_finder(close_enough=close_enough)
                if lookup == True:
                    found_bearing = True
            except:
                rospy.logwarn("INITIAL Distance lookup to Waypoint Failed! Trying again...")
        
        rospy.loginfo("Found a heading! Moving to the waypoint....")

        # Getting initial linear distance to the goal 
        distance = math.sqrt(self.bearing_x**2+self.bearing_y**2)
        rospy.logdebug(distance)

        if close_enough:
            dist_thres = self.waypoint_dist_tol
        else:
            dist_thres = close_enough_dist

        # Not within the defined distance tolerance 
        while distance >= dist_thres:
                # Found through experimentation that the front of the robot is offset by pi radians.	
                ang_diff = self.angle_wrap(math.atan2(self.bearing_y, self.bearing_x))
                rospy.loginfo("Current angle difference is: %s", str(abs(ang_diff))) 

                if abs(ang_diff) >= self.waypoint_ang_tol:
                    if ang_diff >= 0 and ang_diff <= 3.14:
                        self.cmd_vel.angular.z = self.turn_spd
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel_pub.publish(self.cmd_vel)
                        #rospy.loginfo("SPIN")
                    else:
                        self.cmd_vel.angular.z = -self.turn_spd
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel_pub.publish(self.cmd_vel)
                        #rospy.loginfo("SPIN")
                else:
                        self.cmd_vel.angular.z = 0
                        self.cmd_vel.linear.x = self.linear_spd
                        self.cmd_vel_pub.publish(self.cmd_vel)
                        #rospy.loginfo("MOVE")

                # Looking up bearing again...
                # rospy.logdebug("Close enough: %s", str(close_enough))
                self.waypoint_bearing_finder(close_enough=close_enough)
                # Updating linear distance to the goal 
                distance = math.sqrt(self.bearing_x**2+self.bearing_y**2)
                rospy.loginfo("The distance remaining is %s. Need to be under %s", str(distance), str(dist_thres))

        # Within x,y tolerance. 
        # Stop the robot. 
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)  

        # If waypoint is a close_enough waypoint, look for a new world measurement. 
        if close_enough:
            rospy.loginfo("At WAYPOINT: %s", str(self.waypoint))
            # Ensure that the world_frames reset to False.
            # Spin() is meant to find them. 
            self.world_aruco_determined = False
            self.world_aruco_saved = False
        # Else used for close enough checker. Return True because close enough. 
        else: 
            return True 

######################################################
# Experiment
######################################################

    def waypoint_experiment(self):
        # Initialize header. 
        self.data_saver(header=True)
        # Find 1st World frame 
        self.spin()
#################################################
        # To go through the 2 waypoints 
        while abs(self.waypoint_cnt) <= 2:
            # To get close enough to the marker if not done so already so that the Waypoints may be calculated. 
            if not self.close_enough_1:
                rospy.logdebug("Close enough 1: %s", self.close_enough_1)
                self.transit(marker_seeking=abs(self.waypoint))
            elif not self.close_enough_2:
                rospy.logdebug("Close enough 2: %s", self.close_enough_2)
                self.transit(marker_seeking=abs(self.waypoint))
            # Spin until the waypoint is calculated 
            frame_seeking = "waypoint_" + str(abs(self.waypoint))
            self.spin(frame_seeking=frame_seeking)
            # Travel to the next waypoint 
            self.traveler()
            # Find the next world frame. Each spin increases the absolute value by 1
            self.spin()

        rospy.loginfo("Experiment complete!")

    #############################################
    # Experimentation Helpers 
    #############################################
    def transit(self,marker_seeking):
        # To get close enough to the markers so that accurate measurements 
        # may be taken. Standalone to make it easier to understand. 
        close_enough = self.traveler(close_enough=False, close_enough_dist=self.close_enough_dist)
        if marker_seeking == 1: 
            self.close_enough_1 = close_enough
        elif marker_seeking == 2: 
            self.close_enough_2 = close_enough  

    def waypoint_bearing_finder(self, close_enough = True):
            # Retrieve x,y distances based on whichever waypoint goal seeking...
            # Don't care about the orientation of the waypoint seeking...
            rospy.logdebug("Seeking waypoint %s", str(self.waypoint))
            if abs(self.waypoint) == 1 and close_enough:
                try:
                    [trans,rot]=self.tf_listener.lookupTransform("/base_link","/waypoint_1",rospy.Time(0))
                    self.bearing_x, self.bearing_y, z = trans
                    return True
                except:
                    rospy.logwarn("Distance lookup to Waypoint 1 Failed! Trying again...")
                    return False

            elif abs(self.waypoint) == 2 and close_enough:
                try:
                    [trans,rot]=self.tf_listener.lookupTransform("/base_link","/waypoint_2",rospy.Time(0))
                    self.bearing_x, self.bearing_y, z = trans
                    return True
                except:
                    rospy.logwarn("Distance lookup to Waypoint 2 Failed! Trying again...")
                    return False

            elif not close_enough:
                rospy.logdebug("NOT close enough.")
                # Need to spin until the marker seeking is within view. 
                if abs(self.waypoint) == 1:
                    try:
                        self.tf_listener.waitForTransform("/fiducial_1","/odom",rospy.Time(0),rospy.Duration(0.5))
                    except:
                        self.spin(frame_seeking=self.waypoint)                        
                elif abs(self.waypoint) == 2:
                    try:
                        self.tf_listener.waitForTransform("/fiducial_2","/odom",rospy.Time(0),rospy.Duration(0.5))
                    except:
                        self.spin(frame_seeking=self.waypoint)    
                try:
                    # Once marker seeking is within view, can calulate the distance to the marker 
                    [trans,rot]=self.tf_listener.lookupTransform("/base_link","/aruco_seeking",rospy.Time(0))
                    self.bearing_x, self.bearing_y, z = trans
                    rospy.loginfo("DISTANCE lookup to 'Close Enough' ArUco successful!")
                    rospy.loginfo("Bearing x: %s, Bearing y: %s", self.bearing_x, self.bearing_y)
                    return True
                except:
                    rospy.logwarn("DISTANCE Lookup to 'Close Enough' ArUco Failed! Trying again...")
                    return False

#############################################
# Save Data 
##############################################
    def data_saver(self, header = False, data_x = [], data_y = [], data_yaw = [], world_frame = 0):
        if header:
            header = ['X (m)', 'Y (m)', 'Yaw (rad)','World Frame', 'Frame Offset (m)']
            f = open(self.save_dir, 'w')
            writer = csv.writer(f)
            # Write data 
            writer.writerow(header)
            f.close()
        else:
            data = [[0,0,0,0,0]]
            # Setup blank save slots 
            for i in range(1,self.min_frames):
                data.append([0,0,0,0,0])

            # Copy data 
            for i in range(0,self.min_frames):
                data[i][0] = data_x[i]
                data[i][1] = data_y[i]
                data[i][2] = data_yaw[i]
                data[i][3] = world_frame
                data[i][4] = self.waypoint_dist
                    

            rospy.logdebug("Data: %s", str(data))

            # Open file in write mode 
            f = open(self.save_dir, 'a')
            writer = csv.writer(f)
            # Write data 
            writer.writerows(data)
            rospy.loginfo("DATA SAVED to: %s", str(self.save_dir))
            f.close()

if __name__ == '__main__':
    rospy.init_node("world_tf_broadcaster",anonymous=True, log_level=rospy.DEBUG)
    world_tf = WorldPub()
    world_tf.waypoint_experiment()

    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException:
            world_tf.shutdown_hook()