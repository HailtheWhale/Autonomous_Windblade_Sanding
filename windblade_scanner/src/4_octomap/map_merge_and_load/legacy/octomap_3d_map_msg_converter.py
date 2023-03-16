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
        self.merge_type = rospy.get_param('merge_type')
        if (self.merge_type == "both"):
            rospy.wait_for_message('/octomap_binary',Octomap)
            rospy.wait_for_message('/octomap_full',Octomap)
        elif (self.merge_type == "binary"):
            rospy.wait_for_message('/octomap_binary',Octomap)
        else:
            rospy.wait_for_message('/octomap_full',Octomap)

        rospy.loginfo("Topic(s) found. Converting them for the Octomap merger.")
        # Time stamp for both
        self.time_stamp = rospy.Time.now()

        # Subscribers, publishers 
        if (self.merge_type == "both"):
            self.binary_pub = rospy.Publisher("/binary_neighbors_topic",OctomapNeighbors, queue_size=1)
            self.full_pub = rospy.Publisher("/full_neighbors_topic",OctomapNeighbors, queue_size=1)
            self.binary_sub=rospy.Subscriber("/octomap_binary",Octomap,self.binary_octomap_copier)
            self.full_sub=rospy.Subscriber("/octomap_full",Octomap, self.full_octomap_copier)
            # Msgs
            self.binary_octo_neighbors = OctomapNeighbors()
            self.full_octo_neighbors = OctomapNeighbors()
            # Depends on 
            self.binary_octo_array = OctomapArray()
            self.full_octo_array = OctomapArray()
            # Depends on 
            self.binary_octo_copy = Octomap()
            self.full_octo_copy = Octomap()
            # Header Containers
            self.binary_seq_stamp = 0
            self.full_seq_stamp = 0
            self.binary_frame_id_stamp = ""
            self.full_frame_id_stamp = ""
        elif (self.merge_type == "binary"):
            self.binary_pub = rospy.Publisher("/binary_neighbors_topic",OctomapNeighbors, queue_size=1)
            self.binary_sub=rospy.Subscriber("/octomap_binary",Octomap,self.binary_octomap_copier)
            # Msgs
            self.binary_octo_neighbors = OctomapNeighbors()
            # Depends on 
            self.binary_octo_array = OctomapArray()
            # Depends on 
            self.binary_octo_copy = Octomap()
            # Header Containers
            self.binary_seq_stamp = 0
            self.binary_frame_id_stamp = ""
        else:
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


    def binary_octomap_copier(self, msg):
        self.binary_seq_stamp+=1
        self.binary_frame_id_stamp = msg.header.frame_id
        self.binary_octo_copy = msg

    def full_octomap_copier(self, msg):
        self.full_seq_stamp+=1
        self.full_frame_id_stamp = msg.header.frame_id
        self.full_octo_copy = msg

    def binary_octo_array_converter(self):
        '''
        Left outside of a loop for readability.
        '''

        # Binary OctomapArray Stuff 
        self.binary_octo_array.header.seq = self.binary_seq_stamp
        self.binary_octo_array.header.stamp = self.time_stamp
        self.binary_octo_array.header.frame_id = self.binary_frame_id_stamp
        self.binary_octo_array.octomaps = [self.binary_octo_copy]
        self.binary_octo_array.owner="binary_octomap_merger"
        self.binary_octo_array.num_octomaps = 1

    def full_octo_array_converter(self):
        # Full OctomapArray Stuff 
        self.full_octo_array.header.seq = self.full_seq_stamp
        self.full_octo_array.header.stamp = self.time_stamp
        self.full_octo_array.header.frame_id = self.full_frame_id_stamp
        self.full_octo_array.octomaps = [self.full_octo_copy]
        self.full_octo_array.owner="full_octomap_merger"
        self.full_octo_array.num_octomaps = 1

    def binary_octo_neighbors_converter(self):
        # Binary OctomapNeighbors Stuff 
        self.binary_octo_neighbors.header.seq= self.binary_seq_stamp
        self.binary_octo_neighbors.header.stamp = self.time_stamp
        self.binary_octo_neighbors.header.frame_id = self.binary_frame_id_stamp
        self.binary_octo_neighbors.neighbors = [self.binary_octo_array]
        self.binary_octo_neighbors.num_neighbors = 1

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
            # Make msgs 
            if (converter.merge_type == "both"):
                converter.binary_octo_array_converter()
                converter.full_octo_array_converter()
                converter.binary_octo_neighbors_converter()
                converter.full_octo_neighbors_converter()
            elif (converter.merge_type == "binary"):
                converter.binary_octo_array_converter()
                converter.binary_octo_neighbors_converter()
            else:
                converter.full_octo_array_converter()
                converter.full_octo_neighbors_converter()
            # Publish msgs 
            if (converter.merge_type == "both"):
                converter.binary_pub.publish(converter.binary_octo_neighbors)
                converter.full_pub.publish(converter.full_octo_neighbors)
            elif (converter.merge_type == "binary"):
                converter.binary_pub.publish(converter.binary_octo_neighbors)
            else:
                converter.full_pub.publish(converter.full_octo_neighbors)


            rospy.loginfo_once("Msg converter seems to be working.")
            #converter.debugger()

        except rospy.ROSInterruptException:
            rospy.logfatal(rospy.ROSInterruptException)