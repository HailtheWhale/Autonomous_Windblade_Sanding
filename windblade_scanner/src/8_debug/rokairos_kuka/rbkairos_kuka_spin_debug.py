#!/usr/bin/env python

# ROS general imports
import rospy
import tf
import numpy as np

# Msgs for publishers and subscribers.
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry

class windblade_rbkairos_kuka_debugger():

    def __init__(self, loop_rate=5.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)
        rospy.on_shutdown(self.shutdown_hook)
        # Node name 
        self.node_name = "windblade_rbkairos_kuka_debugger/"
        self.namespace = "/windblade_project/"
        # Parameters
        # Define topics 
        self.odom_topic = str(rospy.get_param(self.namespace + self.node_name + "odom_topic"))
	self.cmd_vel_topic = str(rospy.get_param(self.namespace + self.node_name + "cmd_vel_topic"))

        # Subscribers 
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.read_position)
        # Publishers 
        self.cmd_vel_pub= rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 10)

        # Movement 
        self.cmd_vel = Twist()
        self.cmd_vel.angular.x=0
        self.cmd_vel.angular.y=0
        self.cmd_vel.angular.z=0
        self.cmd_vel.linear.x=0
        self.cmd_vel.linear.y=0
        self.cmd_vel.linear.z=0

        # Turning and linear spds 
        self.turn_spd = float(rospy.get_param(self.namespace + self.node_name + "turn_spd"))
        self.linear_spd = float(rospy.get_param(self.namespace + self.node_name + "linear_spd"))

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

    def twist_test(self):
        # Twists
        self.cmd_vel.angular.z = self.turn_spd
        self.cmd_vel.linear.x = 0
        # Publish
        self.cmd_vel_pub.publish(self.cmd_vel)

    def move_test(self):
        # Twists
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel.linear.x = -0.1
        # Publish
        self.cmd_vel_pub.publish(self.cmd_vel)


def main():
    	rospy.init_node("windblade_rbkairos_kuka_debugger", log_level=rospy.DEBUG)
	debugger=windblade_rbkairos_kuka_debugger()
	while not rospy.is_shutdown():
		try:
            		debugger.twist_test()
        	except rospy.ROSInterruptException:
            		debugger.shutdown_hook()

if __name__ == '__main__':
    main()