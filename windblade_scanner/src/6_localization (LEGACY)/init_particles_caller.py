#!/usr/bin/python

import rospy

# To call the init particles service from the AMCL package 
from std_srvs.srv import Empty

rospy.init_node("init_particles_service_client")
# Wait for init particles service from AMCL
rospy.wait_for_service("/global_localization")
# Connect to the Service 
init_particles_service = rospy.ServiceProxy("/global_localization", Empty)
# Call it 
init_particles_service()