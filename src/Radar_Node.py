#!/usr/bin/env python
# Radar Display Node that obtains information from the RadarDisplay class
# 
# Created By: Jeovanny Reyes
# Created On: March 3, 2018
#
# Raytheon Radar Guided Rescue Robot
# Cal State LA

import rospy
from pygame_radar import RadarDisplay

def Radar_Node(object):
	rospy.init_node("radar_display", anonymous=True)