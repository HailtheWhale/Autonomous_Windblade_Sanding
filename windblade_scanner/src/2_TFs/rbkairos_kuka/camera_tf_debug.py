#!/usr/bin/python

# ROS general imports
import rospy
import tf 
import numpy as np

class StaticArm():

    def __init__(self, loop_rate=500.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)
        rospy.on_shutdown(self.shutdown_hook)
        # Node name 
        self.node_name = "arm_camera_frame_publisher/"

        # Transformers
        self.arm_tf_broadcasterbase0 = tf.TransformBroadcaster()

	self.arm_tf_listener=tf.TransformListener()

###############################################################
# Helper Functions 
###############################################################
    def shutdown_hook(self):
        # Rapid Shutdown
        rospy.logfatal("Shutdown!")
	rospy.signal_shutdown("ctrl C.")

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
        parent_frame_base0 = "ee_camera_link"
        child_frame_base0 = "camera2_link"
        trans_base0 = (0.0,0.0,0.0)
        quat_base0 = self.euler_to_quat(1.0,-1.48,0.64)

        self.arm_tf_broadcasterbase0.sendTransform(trans_base0, 
                                            quat_base0, rospy.Time.now(), child_frame_base0, parent_frame_base0)


if __name__ == '__main__':
    rospy.init_node("static_kuka_tf_publisher",anonymous=True, log_level=rospy.DEBUG)
    arm_tf = StaticArm()

    while not rospy.is_shutdown():
        try:
            arm_tf.static_arm_publisher()
        except rospy.ROSInterruptException:
            arm_tf.shutdown_hook()
