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

        # Parameters
        # Define save directory 
        self.save_dir = str(rospy.get_param("experiment_save_directory"))
        # Define angle tolerance 
        self.waypoint_ang_tol = float(rospy.get_param("waypoint_ang_tol"))
        # Define number of measurements must be taken before moving to next step distance.
        self.min_frames = int(rospy.get_param("min_frames"))
        # Define Distance to code min, max, step
        self.dist_meas_min = float(rospy.get_param("dist_meas_min"))
        self.dist_meas_max = float(rospy.get_param("dist_meas_max"))
        self.dist_meas_step = float(rospy.get_param("dist_meas_step"))
        # Define marker name 
        self.aruco_code = str(rospy.get_param("aruco_code"))
        # Define topics 
        self.odom_topic = str(rospy.get_param("odom_topic"))
        self.fiducial_tf_topic = str(rospy.get_param("fiducial_tf_topic"))
        self.cmd_vel_topic = str(rospy.get_param("cmd_vel_topic"))

        # Transformers
        self.aruco_broadcaster = tf.TransformBroadcaster()
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

        # Determined aruco Markers?
        self.aruco_measurements_taken = False

        # Measurement Containers 
        self.x_meas = []
        self.y_meas = []
        self.yaw_meas = []
        self.meas=[]

        # TF containers 
        self.tf_x = 0
        self.tf_y = 0
        self.tf_ox = 0
        self.tf_oy = 0
        self.tf_oz = 0
        self.tf_ow = 0

        # Which measurement on 
        self.meas_cnt = 0

        # Distance difference x,y containers 
        self.bearing_x = 0
        self.bearing_y = 0

        # 1st Distance saved?
        self.waypoint_set = False

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
        self.aruco_publisher()
    ##### ArUco check  
        # Find Marker coordinates 
        if not self.aruco_measurements_taken:
            # World Frame Check 
            try: 
                # TF From the odometry to the Waypoint 0 ArUco Marker 
                self.tf_listener.waitForTransform(self.aruco_code,"/odom",rospy.Time(0),rospy.Duration(0.5))

                if (len(self.x_meas)-self.meas_cnt*self.min_frames) < self.min_frames:
                    rospy.loginfo_once("Aruco Frame Found! Recording....")

                    # TF From the base_link to the aruco code
                    (trans,rot)=self.tf_listener.lookupTransform("/odom",self.aruco_code,rospy.Time(0))

                    # Copying values. Assuming base in planar with code, discarding roll and pitch. 
                    x,y,z = trans
                    ox,oy,oz,ow = rot
                    roll,pitch,yaw = self.quat_to_euler(ox,oy,oz,ow)
                    yaw+=3.14
                    # There seems to be some timeout involved in the tf frame of the ArUco code. Need to account for 
                    # This to ensure values are not biased. 
                    if len(self.x_meas) > 1 and x == self.x_meas[-1]: 
                        pass
                    else:
                        self.x_meas.append(x)
                        self.y_meas.append(y)
                        self.yaw_meas.append(yaw)

                # If recorded at minimum number of frames, Stop looking for 
                # more frames. 
                elif (len(self.x_meas)-self.meas_cnt*self.min_frames) == self.min_frames:
                    rospy.loginfo("Enough Values recorded!")
                    rospy.loginfo("The x measures are %s",str(len(self.x_meas)))
                    rospy.loginfo("The y measures are %s",str(len(self.y_meas)))
                    if not self.waypoint_set:
                        rospy.loginfo("Averaging recorded values for the step...")
                        avg_tf_pose_x = 0
                        avg_tf_pose_y = 0
                        avg_tf_orient_yaw = 0
                        # Summations 
                        for i in range((self.meas_cnt*self.dist_meas_step),len(self.x_meas)):
                            avg_tf_pose_x+=self.x_meas[i]
                            avg_tf_pose_y+=self.y_meas[i]
                            avg_tf_orient_yaw+=self.yaw_meas[i]

                        # Avgs
                        avg_tf_pose_x/=(self.min_frames)
                        avg_tf_pose_y/=(self.min_frames)
                        print("Avg x:", avg_tf_pose_x)
                        print("Avg y", avg_tf_pose_y)
                        avg_tf_orient_yaw/=(self.min_frames)
                        
                        # Convert from Euler to Quat 
                        ox,oy,oz,ow= self.euler_to_quat(0,0,avg_tf_orient_yaw)
                        # Save values 
                        self.tf_x = avg_tf_pose_x
                        self.tf_y = avg_tf_pose_y
                        self.tf_ox = ox
                        self.tf_oy = oy
                        self.tf_oz = oz
                        self.tf_ow = ow
                        # Aruco Position now set 
                        rospy.logfatal("ArUco position calculated!")
                        self.waypoint_set = True

                    self.aruco_measurements_taken = True 
                    self.meas_cnt+=1

            except:
                rospy.logdebug("ArUco Frame not in View!")

#########################################################################
    def aruco_publisher(self):
        # Publishes based on if the frame is available or not. 
        if self.aruco_measurements_taken:
            # Sends the QR Code TF
            parent_frame = "odom"
            child_frame = "aruco"
            self.aruco_broadcaster.sendTransform((self.tf_x, self.tf_y, 0), 
                                                (self.tf_ox,self.tf_oy,self.tf_oz,
                                                self.tf_ow), rospy.Time.now(), child_frame, parent_frame)      

#########################################################################     

    def spin(self):
        # Reset Spin variables 
        self.finished_turning = False
        self.started_rot = False

        # To alternate between left and right spins. 
        if self.meas_cnt%2 == 0:
            turn_spd = -self.turn_spd
        else:
            turn_spd = self.turn_spd

        while not self.started_rot:
            self.cmd_vel.angular.z = turn_spd
            self.cmd_vel_pub.publish(self.cmd_vel)
            # rospy.logdebug("Starting Rotation...")
            # rospy.logdebug(self.orient_theta)
            if (self.orient_theta >= 0.15 and self.orient_theta <= 6.18):
                self.started_rot = True

        while not self.finished_turning:

            # Rotate
            self.cmd_vel.angular.z = turn_spd
            self.cmd_vel_pub.publish(self.cmd_vel)

            # Found code. Reorient to start position. 
            if self.aruco_measurements_taken:
                rospy.loginfo_once("Frame measurements taken!")

                # Wait till reoriented.
                if (self.orient_theta >= 0 and self.orient_theta <= 0.1 or self.orient_theta >= 6.18 and self.orient_theta <= 6.28):
                    self.finished_turning = True
                    rospy.loginfo_once("World frame set!")
                    # Kill the node to free up resources. Done with it. 
                    #self.kill_qr()
        
        # Stop robot 
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)

    def traveler(self, meas_dist):
        found_bearing = False 
        while not found_bearing:
            # Retrieve x,y distances based on whichever waypoint goal seeking...
            try:
                self.waypoint_bearing_finder()
                found_bearing = True
            except:
                rospy.logwarn("Distance lookup to Marker Failed! Trying again...")
        
        rospy.loginfo("Found a heading! Moving to the waypoint....")

        # Getting initial linear distance to the goal 
        distance = math.sqrt(self.bearing_x**2+self.bearing_y**2)
        rospy.logwarn("The distance remaining is %s", str(distance))
        rospy.logwarn("Want < %s", meas_dist)

        # Not within the defined distance tolerance 
        while distance >= meas_dist:
                # Found through experimentation that the front of the robot is offset by pi radians.	
                ang_diff = self.angle_wrap(math.atan2(self.bearing_y, self.bearing_x))
                #rospy.loginfo("Current angle difference is: %s", str(abs(ang_diff))) 

                if abs(ang_diff) >= self.waypoint_ang_tol:
                    if ang_diff >= 0 and ang_diff <= 3.14:
                        self.cmd_vel.angular.z = self.turn_spd
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel_pub.publish(self.cmd_vel)
                    else:
                        self.cmd_vel.angular.z = -self.turn_spd
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel_pub.publish(self.cmd_vel)
                else:
                        self.cmd_vel.angular.z = 0
                        self.cmd_vel.linear.x = self.linear_spd
                        self.cmd_vel_pub.publish(self.cmd_vel)

                # Looking up bearing again...
                self.waypoint_bearing_finder()
                # Updating linear distance to the goal 
                distance = math.sqrt(self.bearing_x**2+self.bearing_y**2)
                rospy.loginfo("The distance remaining is %s", str(distance))

        # Within x,y tolerance. 
        # Stop the robot. 
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)  

        # Ensure that the aruco measurements is reset to False  
        self.aruco_measurements_taken = False

    def aruco_experiment(self):
        # Measurement 1
        self.spin()
        for j in range(0,self.min_frames):
                self.meas.append(0)
        # Making measurement matrix 
        meas_distances = [self.dist_meas_min]
        step_size = (self.dist_meas_max-self.dist_meas_min)/self.dist_meas_step
        current_step = self.dist_meas_min
        for i in range(0,self.dist_meas_step):
            current_step+=step_size
            meas_distances.append(current_step)
        # inverting order of matrix 
        meas_distances_inv = meas_distances[-1:0:-1]
        meas_distances_inv.append(meas_distances[0])
        meas_distances = meas_distances_inv
        rospy.loginfo(meas_distances)

        # To go through all measurement steps 
        for i in range(0,len(meas_distances)):
            self.traveler(meas_distances[i])
            self.spin()
            for j in range(0,self.min_frames):
                self.meas.append(meas_distances[i])

        rospy.loginfo("Experiment complete!")

    def waypoint_bearing_finder(self):
            # Retrieve x,y distances based on whichever waypoint goal seeking...
            # Don't care about the orientation of the waypoint seeking...
            try:
                [trans,rot]=self.tf_listener.lookupTransform("/base_link","/aruco",rospy.Time(0))
                self.bearing_x, self.bearing_y, z = trans
            except:
                rospy.logwarn("Distance lookup to ArUco Failed! Trying again...")
    
    def data_saver(self):
        header = ['X (m)', 'Y (m)', 'Yaw (rad)', 'Distance (m)']
        data = [[0,0,0,0]]

        # Setup blank save slots 
        for i in range(1,len(self.x_meas)):
            data.append([0,0,0,0])

        rospy.logdebug("Data length: %s", str(data))
        rospy.logdebug("Meas length: %s", str(len(self.x_meas[:])))

        # Copy data 
        for i in range(0,len(self.x_meas)):
            data[i][0] = self.x_meas[i]
            data[i][1] = self.y_meas[i]
            data[i][2] = self.yaw_meas[i]
            data[i][3] = self.meas[i]
                
        # Open file in write mode 
        f = open(self.save_dir, 'w')
        writer = csv.writer(f)

        # Write data 
        writer.writerow(header)
        writer.writerows(data)
        rospy.loginfo("DATA SAVED to: %s", str(self.save_dir))
        f.close()


if __name__ == '__main__':
    rospy.init_node("aruco_distance_distortion_tester",anonymous=True, log_level=rospy.DEBUG)
    world_tf = WorldPub()
    world_tf.aruco_experiment()
    world_tf.data_saver()

    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException:
            world_tf.shutdown_hook()