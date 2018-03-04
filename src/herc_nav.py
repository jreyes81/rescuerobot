#!/usr/bin/env python
# Hercules Navigation Code : Takes in information from ultrasounds and process the information to
#                            determine the path the robot will navigate through.
#
# Created By: Jeovanny Reyes
# Created On: February 24, 2018
#
# Subscriber: Subscribes to message topic "/HerculesUltrasound_Range" containing "Range" data
# Publisher:  Publishes to message topic "cmd_vel" with "geometry_msgs/Twist" data
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

# Will need to use switch cases. Since python doesn't have this feature, we will have to
# use dictionary mappings.

import math
import sys
import rospy

from sensor_msgs.msg import Range # Ultrasound
from geometry_msgs.msg import Twist # Robot movements
from pygame_radar import RadarDisplay
#import pygame_radar

class Navigation(object):
    def __init__(self):
        rospy.init_node("navigation_node", anonymous=True)

        # Initializing values
        self.real_obj_dist = 0
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.angular.z = 0
	self.stopbutton = RadarDisplay.stop_button() # Importing stop value from pygame_radar.py's stop_button funcion

        # Subscriber
        self.ran_sub = rospy.Subscriber("HerculesUltrasound_Range", Range, self.distcallback) #Used to be Float32. [Float 32, callback])
        # Publisher
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100) # Rate set in Hz

        rate = rospy.Rate(10) # Pubslishing at 10 hz

        while not rospy.is_shutdown():
            rate.sleep()

    def distcallback(self,range): # Takes in message "range" as input. Data comes back in cm
        real_obj_dist = range.range
        self.real_obj_dist = real_obj_dist

    def move(self): # Function to make robot move
        self.real_obj_dist = math.fabs(self.real_obj_dist) # Returns the absolute value of distance

        if ((self.real_obj_dist < 100) & (self.real_obj_dist > 0) & (self.stopbutton == False)):
            self.twist.linear.x = 20 # Robot moves forward at 20% speed
            self.twist.angular.z = 0 # Robot does not rotate
            self.cmd_pub.publish(self.twist) # Publishes
	    print('Hercules Going Forward!')

        if ((self.real_obj_dist > 100) & (self.stopbutton == False)):
            self.twist.linear.x = 0
            self.twist.angular.z = 20
            self.cmd_pub.publish(self.twist)
	    print('Hercules Rotating Left!')

        if self.stop == True: #if statement to make robot stop
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_pub.publish(self.twist)
	    print('Hercules Has Stopped!')


if __name__ == '__main__':
	Navigation()
    #try:
    #    Navigation()
    #except rospy.ROSInterruptionException:
    #    pass
