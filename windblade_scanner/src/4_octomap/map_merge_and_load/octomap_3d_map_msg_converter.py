#!/usr/bin/python
'''
The octomap merge package expects the "Neighbor" Octomap to use the custom
msg type defined by the package... so, even if merging only 2 maps, need to 
convert msg types to make this possible. 
'''
# ROS general imports
import rospy

# Msg imports
from octomap_merger.msg import OctomapArray, OctomapNeighbors
from octomap_msgs.msg import Octomap

class OctomapMergeMsgConverter():

    def __init__(self, loop_rate=5.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)
        rospy.loginfo("Waiting for octomap binary and/or full topics...")
        rospy.logwarn("If nothing seems to be happening, check the script's subscribed Octomap topics.")

        rospy.wait_for_message('/octomap_full',Octomap)

        rospy.loginfo("Topic(s) found. Converting them for the Octomap merger.")
        # Time stamp for both
        self.time_stamp = rospy.Time.now()

        # Subscribers, publishers 
        self.full_pub = rospy.Publisher("/full_neighbors_topic",OctomapNeighbors, queue_size=1)
        self.full_sub=rospy.Subscriber("/octomap_full",Octomap, self.full_octomap_copier)
        # Msgs
        self.full_octo_neighbors = OctomapNeighbors()
        # Depends on 
        self.full_octo_array = OctomapArray()
        # Depends on 
        self.full_octo_copy = Octomap()
        # Header Containers
        self.full_seq_stamp = 0
        self.full_frame_id_stamp = ""

    def full_octomap_copier(self, msg):
        self.full_seq_stamp+=1
        self.full_frame_id_stamp = msg.header.frame_id
        self.full_octo_copy = msg

    def full_octo_array_converter(self):
        # Full OctomapArray Stuff 
        self.full_octo_array.header.seq = self.full_seq_stamp
        self.full_octo_array.header.stamp = self.time_stamp
        self.full_octo_array.header.frame_id = self.full_frame_id_stamp
        self.full_octo_array.octomaps = [self.full_octo_copy]
        self.full_octo_array.owner="full_octomap_merger"
        self.full_octo_array.num_octomaps = 1

    def full_octo_neighbors_converter(self):
        # Full OctomapNeighbors Stuff 
        self.full_octo_neighbors.header.seq= self.full_seq_stamp
        self.full_octo_neighbors.header.stamp = self.time_stamp
        self.full_octo_neighbors.header.frame_id = self.full_frame_id_stamp
        self.full_octo_neighbors.neighbors = [self.full_octo_array]
        self.full_octo_neighbors.num_neighbors = 1

    def debugger(self):
        print("rospy time now",type(rospy.Time.now()))
        print("header time stamp",type(self.time_stamp))

if __name__ == '__main__':
    rospy.init_node("octomap_merge_msg_converter",anonymous=True, log_level=rospy.INFO)
    converter = OctomapMergeMsgConverter()

    while not rospy.is_shutdown():
        try:
            converter.full_octo_array_converter()
            converter.full_octo_neighbors_converter()

            converter.full_pub.publish(converter.full_octo_neighbors)

            rospy.loginfo_once("Msg converter seems to be working.")
            #converter.debugger()

        except rospy.ROSInterruptException:
            rospy.logfatal(rospy.ROSInterruptException)