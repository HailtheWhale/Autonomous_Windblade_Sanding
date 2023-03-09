#!/usr/bin/python
'''
The octomap merge package expects the "Neighbor" Octomap to use the custom
msg type defined by the package... so, even if merging only 2 maps, need to 
convert msg types to make this possible. 
'''
# ROS general imports
import rospy

# Msg imports
from octomap_msgs.msg import Octomap
# Srv imports
from octomap_msgs.srv import GetOctomap, GetOctomapResponse

class MergedOctoMapServiceProvider():

    def __init__(self, loop_rate=5.0):

        # Safety 
        self.rate = rospy.Rate(loop_rate)
        rospy.loginfo("Waiting for merged octomap binary and/or full topics...")
        rospy.logwarn("If nothing seems to be happening, check the script's subscribed Octomap topics.")
        rospy.logwarn("Ensure that the octomap merger is running. It provides the topics needed.")
        self.merge_type = rospy.get_param('merge_type')
        if (self.merge_type == "both"):
            rospy.wait_for_message('/binary_merged_octomap',Octomap)
            rospy.wait_for_message('/full_merged_octomap',Octomap)
        elif (self.merge_type == "binary"):
            rospy.wait_for_message('/binary_merged_octomap',Octomap)
        else:
            rospy.wait_for_message('/full_merged_octomap',Octomap)

        rospy.loginfo("Topic(s) found. Converting them for the Octomap merger.")

        # Variables based on what's being called 
        if (self.merge_type == "both"):
            self.binary_sub=rospy.Subscriber("/binary_merged_octomap",Octomap,self.binary_octomap_copier)
            self.full_sub=rospy.Subscriber("/full_merged_octomap",Octomap,self.full_octomap_copier)
            # Msgs
            self.binary_merged_srv_msg = GetOctomapResponse()
            self.full_merged_srv_msg = GetOctomapResponse()
            # Header
            self.binary_frame_id_stamp = ""
            self.full_frame_id_stamp = ""
            # Data to copy over 
            self.binary_data = 0
            self.full_data = 0
            # Sequence stamp counters
            self.binary_seq_stamp = 0
            self.full_seq_stamp = 0

        elif (self.merge_type == "binary"):
            self.binary_sub=rospy.Subscriber("/binary_merged_octomap",Octomap,self.binary_octomap_copier)
            # Msgs
            self.binary_merged_srv_msg = GetOctomapResponse()
            # Header
            self.binary_frame_id_stamp = ""
            # Data to copy over 
            self.binary_data = 0
            # Sequence stamp counters
            self.binary_seq_stamp = 0

        else:
            self.full_sub=rospy.Subscriber("/full_merged_octomap",Octomap,self.full_octomap_copier)
            # Msgs
            self.full_merged_srv_msg = GetOctomapResponse()
            # Header
            self.full_frame_id_stamp = ""
            # Data to copy over 
            self.full_data = 0
            # Sequence stamp counters
            self.full_seq_stamp = 0

    def binary_octomap_copier(self, msg):
        # Copying all needed info from the base
        self.binary_frame_id_stamp = msg.header.frame_id
        self.binary_data = msg.data

    def full_octomap_copier(self, msg):
        # Copying all needed info from the base
        self.full_frame_id_stamp = msg.header.frame_id
        self.full_data = msg.data

    def binary_service_generator(self, request):
        self.binary_merged_srv_msg.map.header.seq = self.binary_seq_stamp
        self.binary_seq_stamp+=1
        self.binary_merged_srv_msg.map.header.stamp = rospy.Time.now()
        self.binary_merged_srv_msg.map.header.frame_id = self.binary_frame_id_stamp
        # Must change depending on if the merged msg is binary or not
        self.binary_merged_srv_msg.map.binary = True
        self.binary_merged_srv_msg.map.id = "OcTree"
        self.binary_merged_srv_msg.map.resolution = 0.01
        self.binary_merged_srv_msg.map.data = self.binary_data
        # Finally, give everything back
        return self.binary_merged_srv_msg

    def full_service_generator(self, request):
        self.full_merged_srv_msg.map.header.seq = self.full_seq_stamp
        self.full_seq_stamp+=1
        self.full_merged_srv_msg.map.header.stamp = rospy.Time.now()
        self.full_merged_srv_msg.map.header.frame_id = self.full_frame_id_stamp
        # Must change depending on if the merged msg is binary or not
        self.full_merged_srv_msg.map.binary = False
        self.full_merged_srv_msg.map.id = "OcTree"
        self.full_merged_srv_msg.map.resolution = 0.01
        self.full_merged_srv_msg.map.data = self.full_data
        # Finally, give everything back
        return self.full_merged_srv_msg

if __name__ == '__main__':
    rospy.init_node("merged_octomap_service",anonymous=True, log_level=rospy.INFO)
    converter = MergedOctoMapServiceProvider()
    if (converter.merge_type == "both"):
        binary_service_server = rospy.Service("/octomap_binary_merged", GetOctomap, converter.binary_service_generator)
        full_service_server = rospy.Service("/octomap_full_merged", GetOctomap, converter.full_service_generator)
    elif (converter.merge_type == "binary"):
        binary_service_server = rospy.Service("/octomap_binary_merged", GetOctomap, converter.binary_service_generator)
    else:
        full_service_server = rospy.Service("/octomap_full_merged", GetOctomap, converter.full_service_generator)

    rospy.loginfo_once("Merged Service Provider seems to be working.")

    rate = rospy.Rate(10)
    rate.sleep()
    rospy.spin()