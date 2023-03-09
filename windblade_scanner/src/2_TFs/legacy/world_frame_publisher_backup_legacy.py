#!/usr/bin/python

# ROS general imports
import rospy
import tf 
import numpy as np 
import math
# For killing the Aruco detector once done with it 
import subprocess

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

        # Transformers
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.world_tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener=tf.TransformListener()
        # Subscribers 
        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.read_position)
        self.qr_tf_sub= rospy.Subscriber('/fiducial_transforms',FiducialTransformArray,self.qr_tf)
        # Publishers 
        self.cmd_vel_pub= rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

        # Movement 
        self.cmd_vel = Twist()
        self.cmd_vel.angular.x=0
        self.cmd_vel.angular.y=0
        self.cmd_vel.angular.z=0
        self.cmd_vel.linear.x=0
        self.cmd_vel.linear.y=0
        self.cmd_vel.linear.z=0

        # Orientation
        self.orient_theta = 0 

        # Detected Code?
        self.qr_found = False
        # TF set?
        self.tf_set = False
            # Finished operation
        self.finished = False
        # Started rotation
        self.started_rot = False

        # TF containers 
        self.tf_pose_x = 0
        self.tf_pose_y = 0
        self.tf_pose_z = 0

        self.tf_orient_x = 0
        self.tf_orient_y = 0
        self.tf_orient_z = 0
        self.tf_orient_w = 0

        # World frame set relative to QR code 
        self.world_trans_x = float(rospy.get_param('x_tf_trans'))
        self.world_trans_y = float(rospy.get_param('y_tf_trans'))
        self.world_trans_z = float(rospy.get_param('z_tf_trans'))

        self.world_rot_x = float(rospy.get_param('x_tf_rot'))
        self.world_rot_y = float(rospy.get_param('y_tf_rot'))
        self.world_rot_z = float(rospy.get_param('z_tf_rot'))

###############################################################
# Helper Functions 
###############################################################
    def shutdown_hook(self):
        # Rapid Shutdown
        rospy.logfatal("Shutdown!")
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
# Subsciber Functions
###############################################################
    def read_position(self,msg):

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
    
    def qr_tf(self,msg):
        qr_tf = msg.transforms
        if (len(qr_tf) > 0):

            # Wait for the transform to be present 
            self.tf_listener.waitForTransform("/fiducial_0","/odom",rospy.Time(0),rospy.Duration(0.5))

            # Sends the World frame tf RELATIVE to the qr code 
            world_tf_quat = self.euler_to_quat(1.57,-1.57,3.14)
            parent_frame = "fiducial_0"
            child_frame = "intermediate"
            self.world_tf_broadcaster.sendTransform((0, -0.7, 0), 
                                        (world_tf_quat), rospy.Time.now(), child_frame, parent_frame)

            # TF From the base_link to the qr code
            (trans,rot)=self.tf_listener.lookupTransform("/odom","/intermediate",rospy.Time(0))

            # If location of the code not saved, save it. Don't change it anymore.
            if not self.tf_set:
                self.tf_orient_x,self.tf_orient_y,self.tf_orient_z,self.tf_orient_w = rot
                self.tf_pose_x, self.tf_pose_y, self.tf_pose_z = trans
                self.tf_pose_z = 0.0
                x,y,z = self.quat_to_euler(self.tf_orient_x,self.tf_orient_y,self.tf_orient_z,self.tf_orient_w)
                print("x: ", x, "y: ", y, "z: ", z)
                self.tf_orient_x,self.tf_orient_y,self.tf_orient_z,self.tf_orient_w = self.euler_to_quat(x,0,z)
                self.tf_set = True
                self.qr_found = True

#########################################################################

    def world_publisher(self):
        
        # Sends the QR Code TF
        parent_frame = "odom"
        child_frame = "world"
        
        self.tf_broadcaster.sendTransform((self.tf_pose_x , self.tf_pose_y, self.tf_pose_z), 
                                            (self.tf_orient_x,self.tf_orient_y,self.tf_orient_z,
                                            self.tf_orient_w), rospy.Time.now(), child_frame, parent_frame)


    def spin(self):
        # While qr code not found and less than 3 rotations.
        while not self.started_rot:
            self.cmd_vel.angular.z = -0.2
            self.cmd_vel_pub.publish(self.cmd_vel)
            rospy.logdebug("Starting Rotation...")
            # rospy.logdebug(self.orient_theta)
            if (self.orient_theta >= 0.1 and self.orient_theta <= 6.18):
                self.started_rot = True

        while not self.finished:

            # Rotate
            self.cmd_vel.angular.z = -0.2
            self.cmd_vel_pub.publish(self.cmd_vel)

            # Found code. Reorient to start position. 
            if self.qr_found:
                self.cmd_vel.angular.z = -0.2
                self.cmd_vel_pub.publish(self.cmd_vel)
                rospy.logdebug("World frame found!")

                # Wait till reoriented.
                if (self.orient_theta >= 0 and self.orient_theta <= 0.1 or self.orient_theta >= 6.18 and self.orient_theta <= 6.28):
                    self.finished = True
                    rospy.loginfo("World frame set!")
                    # Kill the node to free up resources. Done with it. 
                    self.kill_qr()

            # Didn't find QR code in a rotation. Warn user. 
            elif not self.qr_found and (self.orient_theta >= 0 and self.orient_theta <= 0.1 or self.orient_theta >= 6.18 and self.orient_theta <= 6.28): 
                rospy.logwarn("World frame not found! Defaulting it to odometry.")
                self.tf_orient_x,self.tf_orient_y,self.tf_orient_z,self.tf_orient_w = 0,0,0,1
                self.tf_pose_y, self.tf_pose_z, self.tf_pose_x = 0,0,0
                self.finished = True
                # Kill the node to free up resources. Done with it. 
                self.kill_qr()
        
        # Stop robot 
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)

    def kill_qr(self):
        subprocess.call(["rosnode kill /aruco_detect"], shell=True)
        rospy.logwarn("aruco_detect node killed!")

if __name__ == '__main__':
    rospy.init_node("world_tf_broadcaster",anonymous=True, log_level=rospy.INFO)
    world_tf = WorldPub()
    # Find the world frame. Save its location. Reorient to start. 
    rospy.logwarn("Looking for 'World' Frame... ")
    rospy.logwarn("TFs in RVIZ will be broken for a bit...")
    world_tf.spin()


    while not rospy.is_shutdown():
        try:
            world_tf.world_publisher()
            #rospy.logdebug("The translation is from :")
            #print(world_tf.tf_pose_x, world_tf.tf_pose_y, world_tf.tf_pose_z)
            rospy.loginfo_once("The rotation is from:")
            rospy.loginfo_once(world_tf.tf_orient_x)
            rospy.loginfo_once(world_tf.tf_orient_y)
            rospy.loginfo_once(world_tf.tf_orient_z)
            rospy.loginfo_once(world_tf.tf_orient_w)

        except rospy.ROSInterruptException:
            world_tf.shutdown_hook()