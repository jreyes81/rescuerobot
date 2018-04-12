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
        self.rate = rospy.Rate(60) # Pubslishing at 60 hz
        self.scan_once_state = []
        self.scan_once_return = 2 # Used to send state information back to radar display to scan again
        self.count = 0
        self.store_count_obj = 0
        self.robot_on_standby = True
        self.done_processing_data = False # Used to tell when robot will move when data is done being processed!
        self.go_to_sect4 = False # Used to tell if robot will navigate through sector 4
        self.go_to_sect5 = False # Used to tell if robot will navigate through sector 5
        self.go_to_sect6 = False # Used to tell if robot will navigate through sector 6
        self.state_of_radar = True
        self.real_obj_dist_array = [0] *4101 # Will store data from ultrasound. Ued to be 4096. We will process this data
        self.fake_obj_dist = 0 # Used to store data when we re done sweeping. It is a holder so we don't process it.
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
        self.scan_once = rospy.Subscriber("scan_once_to_whls",Float32, self.scan_once_state_cb) # Recieves 0 when radar is still sweeping, 1 when radar is done
        self.ran_sub = rospy.Subscriber("HerculesUltrasound_Range", Range, self.distcallback) #Used to be Float32. [Float 32, callback])
        self.stop_cmd = rospy.Subscriber("cmd_stop",Bool,self.stop_cb)
        #self.scan_once = rospy.Subscriber("scan_once_to_whls",Float32, self.scan_once_state_cb) # Recieves 0 when radar is still sweeping, 1 when radar is done

        # Publisher
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) # Rate set in Hz
        self.scan_return = rospy.Publisher("scan_once_return", Float32, queue_size=10) # Returns 2 to have robot scan again as soon as robot is finished moving

        self.twist = Twist() # Contains linear and angular information
        self.robot_state()

    def scan_once_state_cb(self,data):
        self.scan_once_state = data.data # 0 is when radar is still sweeping. 1 is when radar has stopped sweeping
        if self.scan_once_state == 0: # For 2nd iterations and above when robot sweeps again
            self.count = 0 # Reseting count when radar scans again and we need to move again!

    def robot_state(self):
        while not rospy.is_shutdown():
            if self.scan_once_state == 0 and self.robot_on_standby == True: # Robot scanning and awaitng orders!
                print("Robot Scanning and Storing Data. Hercules on Standby!")
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_pub.publish(self.twist)

            if self.scan_once_state == 1: # Robot dones scanning and ready to process data
                print("Robot Stopped Scanning and Processing Data!")
                self.store_data_in_sectors() # Seperates the self.real_obj_dist_array into the 9 sectors to process data
                print("Finished seperating data in sectors")
                self.process_data()
                print("Finished processing data!")
                print(self.done_processing_data) # Should be "True"
                print(self.robot_on_standby) # Should be "False"
                print(self.go_to_sect5) # If "True", we can move straight
                print(self.go_to_sect4) # If "True" there is no object in this sector
                print(self.go_to_sect6) # If "True" there is no object in this sector
                if self.done_processing_data == True: # Robot has finished processing data and ready to send commands to motor controller to move!
                #print("Done Processing Data!")
                    self.move()
                    self.count = self.count + 1 # It takes about 5 seconds to get to a 300 count!

    def distcallback(self,data): # Takes in message "range" as input. Data comes back in cm. This part of code works well!
        if self.scan_once_state == 0: # As long as radar is sweeping, store data
            print('Useful Data!')
            #print(self.store_count_obj) # To make sure count gets incremented
            real_obj_dist = data.range # Stores object distance (raw data) info into local variable
            self.real_obj_dist_array[self.store_count_obj] = real_obj_dist # Stores local variable into an array to process data
            print(self.real_obj_dist_array[1:self.store_count_obj]) # Should print the range ov values according to the count
            #print(self.real_obj_dist_array[self.store_count_obj])
            self.store_count_obj = self.store_count_obj + 1 # Increments to store next data we obtain as sensor sweeps
        if self.scan_once_state == 1: # Radar has stopped sweeping, we dont care about storing data in our processing array
            print('Not useful Data!')
            fake_obj_dist = data.range # Don't care about this data!

    def stop_cb(self,data): # Takes in bool values. Data messages come in as True when "STOP" button is pressed
        self.stop = data.data

    def store_data_in_sectors(self):
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

    def process_data(self):
        # We process and analyze the information stored in sectors 4,5 and 6
        # Based on stored distance information, we set information about the sectors and ir robot can navigate through them
        for i in range(454):
            if self.sect5[i] > 58: # Checking to see if there is an object in sector 5 within 60 cm
                self.go_to_sect5 = True # There is no object detected within the 60 cm threshold of sector 5
            if self.sect4[i] > 58: # Checking to see if there is an object in sector 4 within 60 cm
                self.go_to_sect4 = True # There is no object detected within the 60 cm threshold of sector 4
            if self.sect6[i] > 58: # There is no object detected within the 60 cm threshold of sector 6
                self.go_to_sect6 = True # There is no object detected within the 60 cm threshold of sector 5
        self.robot_on_standby= False
        self.done_processing_data = True # Data has been processed, and robot is now ready to move!
        #print('Done processing data!')

    def move(self): # Function to make robot move

            if self.robot_on_standby == False: # Robot has finished collecting data and is processing it
                if self.done_processing_data == True: # Robot has finished processing data and ready to move!

                    # Op Mode: Stop : When Emergency stop button is pressed
                    if self.stop == True: # StopButton was pressed
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                        self.cmd_pub.publish(self.twist)
                        print('Hercules Has Stopped!')

                    # Op Mode 1: Go Straight : Object is detected past 60 cm and stop button hasnt been pressed
                    if self.stop == False and self.go_to_sect5 == True: # Stopp button has not been pressed and there is no object in front of robot
                        print('Hercules Going Forward!')
                        if self.count < 150: # 300 counts is about 5 seconds. 150 counts is about 2.5 seconds
                            self.twist.linear.x = 20 # Robot moves forward
                            self.twist.angular.z = 0 # Robot does not rotate
                            self.twist.linear.z = 0
                            self.cmd_pub.publish(self.twist) # Publishes
                        if self.count >= 150:
                            self.twist.linear.x = 0 # Robot stops
                            self.twist.angular.z = 0
                            self.cmd_pub.publish(self.twist)
                            self.count = 0 # Count variable reset so it can be reused
                            self.pub_scan_once_return() # Returns a value of 2 to have radar sweep again

                    # Op Mode 2: Navigate Right Side Around Object : Object is within 60 cm and stop button hasn't been pressed
                    if self.stop == False and self.go_to_sect5 == False and self.go_to_sect4 == False:
                        print('Hercules Going Right!')
                        if self.count < 150: # It takes about 5 seconds to get to a 300 count!
                            self.twist.linear.x = 0 # Robot steers right first
                            self.twist.angular.z = 20
                            self.cmd_pub.publish(self.twist)
                        if 150 < self.count < 300: # From 150 to 300 is about 2.5 seconds
                            self.twist.linear.x = 0
                            self.twist.angular.z = -20 # Robot steers left second
                            self.cmd_pub.publish(self.twist)
                        if 300 <= self.count < 450: # From 300 to 450 is about 2.5 seconds
                            self.twist.linear.x = 20
                            self.twist.angular.z = 0 # Robot then goes straight
                            self.cmd_pub.publish(self.twist)
                        if self.count >= 450:
                            self.twist.linear.x = 0
                            self.twist.angular.z = 0 # Robot then stops after a few seconds
                            self.cmd_pub.publish(self.twist)
                            #self.count = 0 # Reseting count variable to be reused
                            self.pub_scan_once_return() # Returns a value of 2 to have radar sweep again

                    # Op Mode 3: Navigate Left Side Around Object : Object is within 60 cm and stop button hasn't been pressed
                    if self.stop == False and self.go_to_sect5 == False and self.go_to_sect4 == True:
                        print('Hercules Going Left!')
                        if self.count < 150: # About 2.5 second travel time
                            self.twist.linear.x = 0
                            self.twist.angular.z = -20 # Robot goes left first!
                            self.cmd_pub.publish(self.twist)
                        if 150 <= self.count < 300: # From 150 to 300 is about 2.5 seconds
                            self.twist.linear.x = 0
                            self.twist.angular.z = 20 # Robot steers right second
                            self.cmd_pub.publish(self.twist)
                        if 300 <= self.count < 450: # From 300 to 450 is about 2.5 seconds
                            self.twist.linear.x = 20
                            self.twist.angular.z = 0 # Robot then goes straight
                            self.cmd_pub.publish(self.twist)
                        if self.count >= 450:
                            self.twist.linear.x = 0
                            self.twist.angular.z = 0 # Robot then stops after a few seconds
                            self.cmd_pub.publish(self.twist)
                        self.count = 0 # Reseting count variable to be reused

            self.rate.sleep()

    def pub_scan_once_return(self):
        self.scan_return.publish(self.scan_once_return)

if __name__ == '__main__':
    Navigation()
