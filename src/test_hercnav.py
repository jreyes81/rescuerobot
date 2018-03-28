#!/usr/bin/env python
# Hercules Navigation Code : Takes in information from ultrasounds and process the information to
#                            determine the path the robot will navigate through.
#
# Created By: Jeovanny Reyes
# Created On: February 24, 2018
# Modified On: March 27, 2018 by Mario
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
import opmodes

from sensor_msgs.msg import Range # Ultrasound
from geometry_msgs.msg import Twist # Robot movements
from std_msgs.msg import Bool,Int8


cases = {
 'opmode1': opmode1, 'opmode2': opmode2, 'opmode3': opmode3, 'stopmode': stopmode
 }

class Navigation(object):
    def __init__(self):

        # Initializing values
        self.real_obj_dist = 0
        self.stop = False
        self.rate = rospy.Rate(10) # Pubslishing at 60 hz
        self.scan_once_state = []

        # Subscriber
        self.ran_sub = rospy.Subscriber("HerculesUltrasound_Range", Range, self.distcallback) #Used to be Float32. [Float 32, callback])
        self.stop_cmd = rospy.Subscriber("cmd_stop",Bool,self.stop_cb)
        self.scan_once = rospy.Subscriber("scan_once_to_whls",Int8, self.scan_once_state_cb)

        # Publisher
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) # Rate set in Hz

        self.twist = Twist() # Contains linear and angular information
        self.move()

    def distcallback(self,range): # Takes in message "range" as input. Data comes back in cm
        real_obj_dist = range.range
        self.real_obj_dist = real_obj_dist

    def stop_cb(self,data): # Takes in bool values as strings. Data messages come when "STOP" button is pressed
        self.stop = data.data

    def scan_once_state_cb(self,data):
        self.scan_once_state = data.data # 0 is when radar is still sweeping. 1 is when radar has stopped sweeping
'''
    def move(self): # Function to make robot move
        while not rospy.is_shutdown():

            self.real_obj_dist = math.fabs(self.real_obj_dist) # Returns the absolute value of distance. Might not need this
            #print(self.stop)

            # Op Mode: Straight
            if ((self.real_obj_dist > 60) and (self.stop == False) and (self.scan_once_state == 1)):
                # Object is detected past 60 cm, stop button hasnt been pressed, and radar has finished sweeping
                self.twist.linear.x = 20 # Robot moves forward at 20% speed
                self.twist.angular.z = 0 # Robot does not rotate
                self.cmd_pub.publish(self.twist) # Publishes
                print('Hercules Going Forward!')

            # Op Mode: Turn Left
            if ((0 < self.real_obj_dist < 60) and (self.stop == False) and (self.scan_once_state == 1)):
                # Object is within 60 cm, stop button hasn't been pressed and radar has finished sweeping
                self.twist.linear.x = 0
                self.twist.angular.z = 20
                self.cmd_pub.publish(self.twist)
                print('Hercules Rotating Left!')

            # Op Mode: Stop
            if self.stop == True:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_pub.publish(self.twist)
                print('Hercules Has Stopped!')

            # Op Mode: Standby
            if self.stop == False and self.scan_once_state == 0: # Stop button hasn't been pressed and radar is still sweeping
                print('Hercules on Standby!')

            self.rate.sleep()
'''
    def nav_route(self, empty)
        if sector5.empty == 0:
            # opmode 1 conditions: object is only detected in sector 5
            # maneuver: robot goes forward
            if sector4.empty == 1 && sector6.empty == 1:
            return cases[opmode1]
            # opmode 2 conditions: object detected only in sector 5 and 4
            # maneuver: robot rotates RIGHT (diagonally), goes forward, rotates left (diagonally), goes forward 
            elif sector4.empty == 0 && sector6.empty == 1:
            return cases[opmode2]
            # opmode 3 conditions: object detected only in sector 5 and 6
            # maneuver: robot rotates left (diagonally), goes forward, rotates right (diagonally), goes forward
            elif sector5.empty == 0 && sector4.empty == 0 && sector6.empty == 1:
            return cases[opmode3]
        # stopmode is the default case that sets all motors to STOP --- But we should make it go forward 0.5m
        else:
            return cases[stopmode]



if __name__ == '__main__':
    Navigation()