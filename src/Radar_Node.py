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

#stop = False # Inital boolean value for "STOP" button
#cmd_stop = ""

def Radar_Node():
    #stop,cmd_stop = RadarDisplay()
    rospy.init_node("radar_display", anonymous=True)
    radar_display = RadarDisplay() # Calls RadarDisplay class
    # stop,cmd_stop = RadarDisplay()

    #rospy.spin

if __name__ == '__main__':
    Radar_Node()
