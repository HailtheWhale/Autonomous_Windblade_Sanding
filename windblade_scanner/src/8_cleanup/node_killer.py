#!/usr/bin/python

import rospy
import subprocess

subprocess.call(["rosnode kill /bumper2pointcloud"], shell=True)
subprocess.call(["rosnode kill /camera1/depth_metric"], shell=True)
subprocess.call(["rosnode kill /camera1/depth_metric_rect"], shell=True)
subprocess.call(["rosnode kill /camera2/depth_metric"], shell=True)
subprocess.call(["rosnode kill /camera2/depth_metric_rect"], shell=True)
subprocess.call(["rosnode kill /camera1/depth_registered_sw_metric_rect"], shell=True)
subprocess.call(["rosnode kill /camera2/depth_registered_sw_metric_rect"], shell=True)
subprocess.call(["rosnode kill /camera1/points_xyzrgb_sw_registered"], shell=True)
subprocess.call(["rosnode kill /camera2/points_xyzrgb_sw_registered"], shell=True)
subprocess.call(["rosnode kill /camera1/register_depth_rgb"], shell=True)
subprocess.call(["rosnode kill /camera2/register_depth_rgb"], shell=True)
subprocess.call(["rosnode kill /camera1/rgb_rectify_color"], shell=True)
subprocess.call(["rosnode kill /camera2/rgb_rectify_color"], shell=True)


rospy.signal_shutdown("Done killing unneccessary nodes.")