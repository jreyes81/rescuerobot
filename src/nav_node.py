#!/usr/bin/env python
# Hercules Navigation Node that obtains information from the herc_nav.py file.
#
# Created By: Jeovanny Reyes
# Created On: March 3, 2018
#
# Raytheon Radar Guided Rescue Robot
# Cal State LA

import rospy
from herc_nav import Navigation

def Nav_Node():
	rospy.init_node("navigation_node", anonymous=True)
	navigation = Navigation() # Calls Navigation class

	rate = rospy.Rate(10) # Pubslishing at 10 hz
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
	Nav_Node()
