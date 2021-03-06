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
from stop_to_nav import stop_to_nav

class Navigation(object):
    def __init__(self):
        #rospy.init_node("navigation_node", anonymous=True)

        # Initializing values
        self.real_obj_dist = 0
        #self.twist = Twist()
        #self.twist.linear.x = 0
        #self.twist.angular.z = 0
        #self.start = True
        self.stop = False

        # Subscriber
        self.ran_sub = rospy.Subscriber("HerculesUltrasound_Range", Range, self.distcallback) #Used to be Float32. [Float 32, callback])
	self.stop_cmd = rospy.Subscriber("cmd_stop",,self.stop_cb)
        # Publisher
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) # Rate set in Hz
	self.twist = Twist()
        print('Navigation Node Started')
        print(self.stop)
        self.move()

    def distcallback(self,range): # Takes in message "range" as input. Data comes back in cm
        real_obj_dist = range.range
        self.real_obj_dist = real_obj_dist

   def stop_cb(self,data): # Takes in bool values as strings. Data messages come when "STOP" button is pressed
	self.stop = data.data

    def move(self): # Function to make robot move
        #while(self.start):
        self.real_obj_dist = math.fabs(self.real_obj_dist) # Returns the absolute value of distance

        if ((self.real_obj_dist < 100) & (self.real_obj_dist > 0) & (self.stop == False)):
            self.twist.linear.x = 20 # Robot moves forward at 20% speed
            self.twist.angular.z = 0 # Robot does not rotate
            self.cmd_pub.publish(self.twist) # Publishes
            print('Hercules Going Forward!')

        if ((self.real_obj_dist > 100) & (self.stop == False)):
            self.twist.linear.x = 0
            self.twist.angular.z = 20
            self.cmd_pub.publish(self.twist)
            print('Hercules Rotating Left!')

        if self.stop == True: #if statement to make robot stop
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_pub.publish(self.twist)
            print('Hercules Has Stopped!')

        if self.stop == False:
            print('Hercules on Standby!')


if __name__ == '__main__':
	Navigation()
    #try:
    #    Navigation()
    #except rospy.ROSInterruptionException:
#    passherold
