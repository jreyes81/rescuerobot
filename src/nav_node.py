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
    print('Navigation Node Started!')
    #navigation = Navigation() # Calls Navigation class

    rate = rospy.Rate(60) # Pubslishing at 60 hz
    while not rospy.is_shutdown():
        navigation = Navigation() # Calls Navigation class
        rate.sleep()

if __name__ == '__main__':
    Nav_Node()
