#!/usr/bin/env python

'''

 cases.py is a file that has the different maneuevering procedures stored, to be called by the
 herc_nav.py file (depending on the information obtained by the ultrasonic sensor).
 Note that this file will replace the "move" Function (on line 54 of herc_nav.py)

'''

# Created By: Mario Medina
# Created On: March 14, 2018

# Publisher: "cmd_pub"; sends information about how to turn robot

# Raytheon Radar Guided Rescue Robot
# Cal State LA Senior Design

from geometry_msgs.msg import Twist
#from time import sleep
#time.sleep(w)    w is the amount of seconds

class opmodes():
	# opmode 1 conditions: object detected in sector 5 & not overlapping sector 4 & not overlapping sector 6
	# maneuver: robot rotates LEFT (diagonally), goes forward, rotates right (diagonally), goes forward 
	def opmode1(self):
        self.twist.angular.z = 20 # Robot rotates left
        self.twist.linear.x  = 20 # Robot moves forward at 20% speed
        # delay of s seconds
        self.twist.linear.x  = 0  # Robot stops moving forward
        self.cmd_pub.publish(self.twist) # Publishes
        self.twist.angular.z = -20 # ASSUMING this rotates robot right
        self.twist.linear.x  = 20  # Robot moves forward at 20% speed
        # delay of s seconds
        self.twist.linear.x  = 0   # Robot stops moving forward
        self.cmd_pub.publish(self.twist) # Publishes
        #print('Op Mode1 Complete')


	# opmode 2 conditions: object detected in sector 5 & overlapping sector 4 & not overlapping sector 6
	# maneuver: robot rotates RIGHT (diagonally), goes forward, rotates left (diagonally), goes forward 
	def opmode2(self):
        self.twist.angular.z = -20 # ASSUMING this rotates robot right
        self.twist.linear.x  = 20  # Robot moves forward at 20% speed
        # delay of s seconds
        self.twist.linear.x  = 0   # Robot stops moving forward
        self.cmd_pub.publish(self.twist) # Publishes
        self.twist.angular.z = 20  # Robot rotates left
        self.twist.linear.x  = 20  # Robot moves forward at 20% speed
        # delay of s seconds
        self.twist.linear.x  = 0   # Robot stops moving forward
        self.cmd_pub.publish(self.twist) # Publishes
        #print('Op Mode2 Complete')


	# opmode 3 conditions: object detected in sector 5 & not overlapping sector 4 & overlapping sector 6
	# maneuver: robot rotates left (diagonally), goes forward, rotates right (diagonally), goes forward
	def opmode3(self):
        self.twist.angular.z = 20 # Robot rotates left
        self.twist.linear.x  = 20 # Robot moves forward at 20% speed
        # delay of s seconds
        self.twist.linear.x  = 0  # Robot stops moving forward
        self.cmd_pub.publish(self.twist) # Publishes
        self.twist.angular.z = -20 # ASSUMING this rotates robot right
        self.twist.linear.x  = 20  # Robot moves forward at 20% speed
        # delay of s seconds
        self.twist.linear.x  = 0   # Robot stops moving forward
        self.cmd_pub.publish(self.twist) # Publishes
        #print('Op Mode3 Complete')


    # opmode 99 is the default case that sets all motors to STOP
	def opmode99(self):
        self.twist.angular.z = 0# Robot rotates left
        self.twist.linear.x  = 0  # Robot stops moving forward
        self.cmd_pub.publish(self.twist) # Publishes
        print('Op Mode99 Complete')

