#!/usr/bin/env python
# Classes  using Shapes as an example

from geometry_msgs.msg import Twist

class opmodes():
	# opmode 1 conditions: object detected in sector 5 & NOT overlapping sector 4 & NOT overlapping sector 6
	# maneuver: robot goes forward
	def opmode1(self):
        self.twist.angular.z = 0 # Robot rotates left (diagonally)
        self.twist.linear.x  = 1 # Robot moves forward 
        self.cmd_pub.publish(self.twist) # Publishes
        #print("Op Mode 1 Complete")


	# opmode 2 conditions: object detected in sector 5 & overlapping sector 4 & NOT overlapping sector 6
	# maneuver: robot rotates RIGHT (diagonally), goes forward, rotates left (diagonally), goes forward 
	def opmode2(self):
        self.twist.angular.z = -1 # rotates robot right (diagonally)
        self.twist.linear.x  = 1  # robot moves forward 
        self.cmd_pub.publish(self.twist) # Publishes
        self.twist.angular.z = 1  # robot rotates left (diagonally)
        self.twist.linear.x  = 1  # robot moves forward 
        self.cmd_pub.publish(self.twist) # Publishes
        #print("Op Mode 2 Complete")


	# opmode 3 conditions: object detected in sector 5 & NOT overlapping sector 4 & NOT overlapping sector 6
	# maneuver: robot rotates left (diagonally), goes forward, rotates right (diagonally), goes forward
	def opmode3(self):
        self.twist.angular.z = 1 # robot rotates left (diagonally)
        self.twist.linear.x  = 1 # robot moves forward 
        self.cmd_pub.publish(self.twist) # Publishes
        self.twist.angular.z = -1 # rotates robot right (diagonally)
        self.twist.linear.x  = 1  # robot moves forward 
        #print("Op Mode 3 Complete")

    # stopmode is the default case that sets all motors to STOP
	def stopmode(self):
        self.twist.angular.z = 0# Robot does not rotate
        self.twist.linear.x  = 0  # Robot stops moving forward
        self.cmd_pub.publish(self.twist) # Publishes
        #print"Stopmode Complete")

