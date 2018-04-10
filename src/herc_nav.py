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
# Raytheon Radar Guided Rescue Robot
# Cal State LA

# Will need to use switch cases. Since python doesn't have this feature, we will have to
# use dictionary mappings.

import math
import sys
import rospy
import time
import numpy

from sensor_msgs.msg import Range # Ultrasound
from geometry_msgs.msg import Twist # Robot movements
from std_msgs.msg import Bool,Float32

class Navigation(object):
    def __init__(self):

        # Initializing values
        self.real_obj_dist = 0
        self.stop = False
        self.rate = rospy.Rate(10) # Pubslishing at 60 hz
        self.scan_once_state = []
        self.count = 0
        self.robot_on_standby = True
        self.done_processing_data = False # Used to tell when robot will move when data is done being processed!
        self.go_to_sect4 = False # Used to tell if robot will navigate through sector 4
        self.go_to_sect5 = False # Used to tell if robot will navigate through sector 5
        self.go_t0_sect6 = False # Used to tell if robot will navigate through sector 6
        self.state_of_radar = True
        self.real_obj_dist_array = [0] *4101 # Will store data from ultrasound. Ued to be 4096
        self.sect1 = [0]*454 # Each sector is incremented by 20 degrees. 227 data points from first half sweep
        self.sect2 = [0]*454
        self.sect3 = [0]*454
        self.sect4 = [0]*454
        self.sect5 = [0]*454
        self.sect6 = [0]*454
        self.sect7 = [0]*454
        self.sect8 = [0]*454
        self.sect9 = [0]*454


        # Subscriber
        self.ran_sub = rospy.Subscriber("HerculesUltrasound_Range", Range, self.distcallback) #Used to be Float32. [Float 32, callback])
        self.stop_cmd = rospy.Subscriber("cmd_stop",Bool,self.stop_cb)
        self.scan_once = rospy.Subscriber("scan_once_to_whls",Float32, self.scan_once_state_cb) # Recieves 0 when radar is still sweeping, 1 when radar is done

        # Publisher
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) # Rate set in Hz

        self.twist = Twist() # Contains linear and angular information
        self.robot_state()
        #self.move() # Used processed data to make navigation decision

    def scan_once_state_cb(self,data):
        self.scan_once_state = data.data # 0 is when radar is still sweeping. 1 is when radar has stopped sweeping
        self.state_of_radar = True

    def robot_state(self):
        while not rospy.is_shutdown():
        # while self.state_of_radar:
            if self.scan_once_state == 0:
                self.robot_on_standby = True
                print("Robot Scanning and Storing Data!")
                self.move()
                #self.store_data()
            if self.scan_once_state == 1:
                print("Robot Stopped Scanning and Processing Data!")
                self.process_data()
                self.move()

    #def store_data(self):
    #    rospy.Subscriber("HerculesUltrasound_Range", Range, self.distcallback) #Used to be Float32. [Float 32, callback])

    def distcallback(self,data): # Takes in message "range" as input. Data comes back in cm
        real_obj_dist = data.range # Stores object distance (raw data) info into local variable
        for i in range(4096): # Find out how many data points we get from one sweep
            self.real_obj_dist_array[i] = real_obj_dist # Stores local variable into an array to process data

    def stop_cb(self,data): # Takes in bool values. Data messages come in as True when "STOP" button is pressed
        self.stop = data.data

    def process_data(self):
        # Storing data from 0 to 180 degree sweep!
        for i in range(227):
            self.sect1[i] = self.real_obj_dist_array[i] # From 0 to 226
            self.sect2[i] = self.real_obj_dist_array[227+i] # From 227 to 454
            self.sect3[i] = self.real_obj_dist_array[455+i] # From 455 to 682
            self.sect4[i] = self.real_obj_dist_array[683+i] # From 683 to 910
            self.sect5[i] = self.real_obj_dist_array[911+i] # From 911 to 1138
            self.sect6[i] = self.real_obj_dist_array[1139+i] # From 1139 to 1366
            self.sect7[i] = self.real_obj_dist_array[1367+i] # From 1367 to 1594
            self.sect8[i] = self.real_obj_dist_array[1595+i] # From 1595 to 1822
            self.sect9[i] = self.real_obj_dist_array[1823+i] # From 1823 to 2050

        # Storing data from 180 to 0 degree sweep!
        for j in range(227):
            self.sect1[j] = self.real_obj_dist_array[3874+i] # From 3874 to 4101
            self.sect2[j] = self.real_obj_dist_array[3646+i] # From 3646 to 3873
            self.sect3[j] = self.real_obj_dist_array[3418+i] # From 3418 to 3645
            self.sect4[j] = self.real_obj_dist_array[3190+i] # From 3190 to 3417
            self.sect5[j] = self.real_obj_dist_array[2962+i] # From 2962 to 3189
            self.sect6[j] = self.real_obj_dist_array[2734+i] # From 2734 to 2961
            self.sect7[j] = self.real_obj_dist_array[2506+i] # From 2506 to 2733
            self.sect8[j] = self.real_obj_dist_array[2278+i] # From 2278 to 2505
            self.sect9[j] = self.real_obj_dist_array[2050+j] # From 2050 to 2277

        # We process and analyze the information stored in sectors 4,5 and 6
        # Based on stored distance information, we exectue following op modes!
        for i in range(454):
            if self.sect5[i] > 58: # Checking to see if there is an object in sector 5 within 60 cm
                self.go_to_sect5 = True # There is no object detected within the 60 cm threshold of sector 5
            if self.sect4[i] > 58: # Checking to see if there is an object in sector 4 within 60 cm
                self.go_to_sect4 = True # There is no object detected within the 60 cm threshold of sector 4
            if self.sect6[i] > 58: # There is no object detected within the 60 cm threshold of sector 6
                self.go_to_sect6 = True # There is no object detected within the 60 cm threshold of sector 5
        self.robot_on_standby= False
        self.done_processing_data = True # Data has been processed, and robot is now ready to move!

    def move(self): # Function to make robot move
        # while not rospy.is_shutdown():

            if self.robot_on_standby == True and self.done_processing_data == False: # Robot is scanning and collectin data
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_pub.publish(self.twist)
                print('Hercules On Standby!')

            # if self.stop == True: # Stop button has been pressed
            #     self.twist.linear.x = 0
            #     self.twist.angular.z = 0
            #     self.cmd_pub.publish(self.twist)
            #     print('Hercules Has Stopped!')

            if self.robot_on_standby == False: # Robot has finished collecting data and is processing it
                if self.done_processing_data == True: # Robot has finished processing data and ready to move!
                    # Op Mode: Straight : Object is detected past 60 cm and stop button hasnt been pressed
                    if self.go_to_sect5 == True and self.stop == False:
                        #if ((self.real_obj_dist > 60) and (self.stop == False)):
                        self.twist.linear.x = 20 # Robot moves forward
                        self.twist.angular.z = 20 # Robot does not rotate
                        self.twist.linear.z = -1
                        self.cmd_pub.publish(self.twist) # Publishes
                        #time.sleep(3) # Forward for threee seconds
                        #rospy.sleep(6)
                        #self.twist.linear.x = 0
                        #self.twist.angular.z = 20
                        #self.cmd_pub.publish(self.twist)
                        print('Hercules Going Forward!')

                    # Op Mode: Turn Left : Object is within 60 cm and stop button hasn't been pressed
                    #if ((0 < self.real_obj_dist < 60) and (self.stop == False)):
                    if self.go_to_sect5 == False and self.go_to_sect4 == True and self.stop == False:
                        self.twist.linear.x = 0
                        self.twist.angular.z = 20
                        self.cmd_pub.publish(self.twist)
                        print('Hercules Rotating Right!')

                # Op Mode: Turn Right : Object is within 60 cm and stop button hasn't been pressed
                    #if ((0 < self.real_obj_dist < 60) and (self.stop == False)):
                #if self.go_to_sect5 == False and self.go_to_sect4 == False self.stop == False:
                #    self.twist.linear.x = 0
                #    self.twist.angular.z = -20
                #    self.cmd_pub.publish(self.twist)
                #    print('Hercules Rotating Right!')

                    # Op Mode: Stop : When Emergency stop button is pressed
                    if self.stop == True:
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                        self.cmd_pub.publish(self.twist)
                        print('Hercules Has Stopped!')

            self.rate.sleep()


if __name__ == '__main__':
    Navigation()
