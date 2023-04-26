#!/usr/bin/python
'''
The octomap merge package expects the "Neighbor" Octomap to use the custom
msg type defined by the package... so, even if merging only 2 maps, need to 
convert msg types to make this possible. 
'''
# ROS general imports
import rospy
import tf

# Msg imports
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry

class Pc2Debug():

    def __init__(self, loop_rate=5.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)

        # Time stamp for both
        self.time_stamp = rospy.Time.now()

        # Subscribers, publishers 
        #self.align_pub = rospy.Publisher("/windblade_pc2_debug",PointCloud2, queue_size=1)
        #self.align_sub1=rospy.Subscriber("/robot/front_rgbd_camera/depth/points",PointCloud2, self.debugger1)
        #self.align_sub2=rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered",PointCloud2, self.debugger2)
        self.odom_sub=rospy.Subscriber("/zed2i/zed_node/odom",Odometry, self.odom_msg)


        self.align_sub=rospy.Subscriber("/occupied_cells_vis_array_rbkairos",MarkerArray, self.scrubber)

        # Msgs
        self.pc1 = PointCloud2()
        self.pc2 = PointCloud2()
        self.pc_pub = PointCloud2()

        self.loaded_map_info = MarkerArray()

        self.odom = MarkerArray()
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
################################################################

    def odom_msg(self,msg):
        ox = msg.pose.pose.orientation.x
        oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z
        ow = msg.pose.pose.orientation.w
        roll,pitch,yaw = self.quat_to_euler(ox,oy,oz,ow)
        print(roll,pitch,yaw)


    def scrubber(self,msg):
        loaded_info_array = msg.markers
        for i in range(0,len(loaded_info_array[15].points)):
            print((loaded_info_array[15].points[i].z))
        print("//////")

    def debugger1(self,msg):
        self.pc1 = msg

    def debugger2(self,msg):
        self.pc2 = msg

    def printer(self,input, name):
        print(".....", name, ".....")
        #print("HEADER", input.header)
        print("HEIGHT", input.height)
        print("WIDTH", input.width)
        print("Point_step", input.point_step)
        print("Row_step", input.row_step)
        print("is_dense", input.is_dense)
        ##print("fields", input.fields)
        if len(input.data) > 0:
        	print("data", input.data[0])


    def publisher(self):
        self.pc_pub = self.pc2
	self.pc_pub.fields=self.pc1.fields
        self.align_pub.publish(self.pc_pub) 

if __name__ == '__main__':
    rospy.init_node("pc2_debug",anonymous=True, log_level=rospy.INFO)
    debug = Pc2Debug()

    while not rospy.is_shutdown():
	try:
            #debug.printer(debug.pc1,"Front Cam")
            #debug.printer(debug.pc2,"Zed Cam")
            #debug.publisher()
            debug.rate.sleep()

	except rospy.ROSInterruptException:
		rospy.logfatal(rospy.ROSInterruptException)