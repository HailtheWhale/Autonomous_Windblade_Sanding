#!/usr/bin/env python

import rospy

from driver import driver

if __name__ == '__main__':
    try:
        pilot = driver()
        pilot.load_goals()
        pilot.drive()
            
    except rospy.ROSInterruptException:
        pilot.shutdown_hook()
