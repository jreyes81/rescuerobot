#!/usr/bin/env python
'''
 opmodes.py is a file that has the different maneuvering procedures stored, to be called by the
 herc_nav.py file (depending on the information obtained by the ultrasonic sensor).
 Note that this file will replace the "move" Function (on line 55 of herc_nav.py)
'''

# Created By: Mario Medina
# Created On: March 14, 2018

# Publisher: "cmd_pub"; sends information about how to turn robot

# Raytheon Radar Guided Rescue Robot
# Cal State LA Senior Design

from geometry_msgs.msg import Twist

class opmodes():
    def __init__(self):
    #Initializing values
        self.hi = "hello"


	# opmode 1 conditions: object detected in sector 5 & NOT overlapping sector 4 & NOT overlapping sector 6
	# maneuver: robot goes forward
	def opmode1(self):
        self.twist.angular.z = 0 # Robot rotates left
        self.twist.linear.x  = 1 # Robot moves forward 
        self.cmd_pub.publish(self.twist) # Publishes
        #print('Op Mode 1 Complete')


	# opmode 2 conditions: object detected in sector 5 & overlapping sector 4 & NOT overlapping sector 6
	# maneuver: robot rotates RIGHT (diagonally), goes forward, rotates left (diagonally), goes forward 
	def opmode2(self):
        self.twist.angular.z = -1 # rotates robot right
        self.twist.linear.x  = 1  # robot moves forward 
        self.cmd_pub.publish(self.twist) # Publishes
        self.twist.angular.z = 1  # robot rotates left
        self.twist.linear.x  = 1  # robot moves forward 
        self.cmd_pub.publish(self.twist) # Publishes
        #print('Op Mode 2 Complete')


	# opmode 3 conditions: object detected in sector 5 & overlapping sector 4 & NOT overlapping sector 6
	# maneuver: robot rotates left (diagonally), goes forward, rotates right (diagonally), goes forward
	def opmode3(self):
        self.twist.angular.z = 1 # robot rotates left
        self.twist.linear.x  = 1 # robot moves forward 
        self.cmd_pub.publish(self.twist) # Publishes
        self.twist.angular.z = -1 # rotates robot right
        self.twist.linear.x  = 1  # robot moves forward 
        #print('Op Mode 3 Complete')

    def string(self):
        hi = "iiiii"

    # stopmode is the default case that sets all motors to STOP
	def stopmode(self):
        self.twist.angular.z = 0# Robot rotates left
        self.twist.linear.x  = 0  # Robot stops moving forward
        self.cmd_pub.publish(self.twist) # Publishes
        #print('Stopmode Complete')

