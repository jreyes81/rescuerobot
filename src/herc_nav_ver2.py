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
        self.count_size_array = 0
        self.scan_once_return = 0 # Used to send state information back to radar display to scan again
        self.count = 0
        self.store_count_obj = 0
        self.robot_on_standby = True
        self.done_processing_data = False # Used to tell when robot will move when data is done being processed!
        self.go_to_sect4 = False # Used to tell if robot will navigate through sector 4
        self.go_to_sect5 = False # Used to tell if robot will navigate through sector 5
        self.go_to_sect6 = False # Used to tell if robot will navigate through sector 6
        self.state_of_radar = True
        # Based on micro sweep speed. We collect 460 points worth of data. Based on radar display sweep, we collecct 606 points of data, but will use 612
        self.real_obj_dist_array = [0] *1500 # Will store data from ultrasound. Takes about 459-465 points worth of raw data for one complete sweep. Used to be 4101
        self.fake_obj_dist = 0 # Used to store data when we re done sweeping. It is a holder so we don't process it.
        self.sect1 = [0]*66 # Each sector is incremented by 20 degrees. Used to be 52
        self.sect2 = [0]*66 # Used to be 51
        self.sect3 = [0]*66 # Used to be 51
        self.sect4 = [0]*70 # Used to be 51
        self.sect5 = [0]*70 # Used to be 51
        self.sect6 = [0]*70 # Used to be 51
        self.sect7 = [0]*66 # Used to be 51
        self.sect8 = [0]*66 # Used to be 51
        self.sect9 = [0]*66 # Used to be 51.Will contain an additional point of data


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
        #if self.scan_once_state == 1: # For 2nd iterations and above when robot sweeps again
        #    self.count = 0 # Reseting count when radar scans again and we need to move again!

    def robot_state(self):
        #if self.scan_once_state == 2:
        #    self.count = 0
        while not rospy.is_shutdown():
            while self.scan_once_state == 0: #and self.robot_on_standby == True: # Robot scanning and awaitng orders!
                print("Robot Scanning and Storing Data. Hercules on Standby!")
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_pub.publish(self.twist)
                self.scan_once_return = 1
                self.pub_scan_once_return() # Returns a value of 1 to have radar know we received a 0 and we won't move

            while self.scan_once_state == 1: #or self.scan_once_state == 2: # Robot dones scanning and ready to process data
                self.done_processing_data = False
                #self.scan_once_return = 2
                #self.pub_scan_once_return() # Returns a value of 2 to let radar node we know it is done sweeping and we are going to move
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
            self.count_size_array = self.count_size_array + 1
            self.new_array = [0]*self.count_size_array
            #print(self.store_count_obj) # To make sure count gets incremented
            real_obj_dist = data.range # Stores object distance (raw data) info into local variable
            self.real_obj_dist_array[self.store_count_obj] = real_obj_dist # Stores local variable into an array to process data
            #print(self.real_obj_dist_array[1:self.store_count_obj]) # Should print the range of values according to the count
            #print(len(self.real_obj_dist_array)) # Prints the number of elements in the array. Useful to see how we can process raw data
            #print(self.real_obj_dist_array[self.store_count_obj])
            self.store_count_obj = self.store_count_obj + 1 # Increments to store next data we obtain as sensor sweeps
        if self.scan_once_state == 1: # Radar has stopped sweeping, we dont care about storing data in our processing array
            print('Not useful Data!')
            fake_obj_dist = data.range # Don't care about this data!

    def stop_cb(self,data): # Takes in bool values. Data messages come in as True when "STOP" button is pressed
        self.stop = data.data

    def store_data_in_sectors(self):
        for i in range(self.store_count_obj):
            self.new_array[i] = self.real_obj_dist_array[i] # Transferring data from the pre created array to a new size array.
            # This new array will contain all of the raw data

        # 459 points is the sweep from 5 to 175 degrees and back [micro servo speed]
        # 606 points is the sweep from 5 to 175 degrees and back [radar display speed]
        # 230 points is from 5 to 175 degrees! [micro servo speed]
        # 303 points is from 5 to 175 degrees same thing back! [radar display speed]
        # 229 points is from 175 degrees to 5!
        # The index value at 230 of the array will be zero and willbe stores in sector 9 since we won't process this

        # Storing data from 0 to 180 degree sweep!

        # for i in range(35): # Used to be 227.Then it was 26
        # # 1st attempt with micro servo: Sectors 3,4,5,6 and 7 will receive 26 data points. Sectors 1,2,8 and 9 will receive 25
        # # 2nd attempt with radar display speed: Sectors 4,5 and 6 will recieve 35 points. Sectors 1,2,3,7,8 and 9 will receive 33
        #     #self.sect3[i] = self.real_obj_dist_array[66+i] # From 66 to 100.Used to be [50+i] From 50 to 75
        #     self.sect4[i] = self.real_obj_dist_array[99+i] # From 99 to 133. Used to be [76+i] From 76 to 101
        #     self.sect5[i] = self.real_obj_dist_array[134+i] # From 134 to 168. Used to be [102+i] From 102 to 127
        #     self.sect6[i] = self.real_obj_dist_array[169+i] # From 169 to 203. Used to be [128+i] From 128 to 153
        #     #self.sect7[i] = self.real_obj_dist_array[154+i] # Used to be [154+i] From 154 to 179

        # for i in range(33): # Used to be 25
        #     self.sect1[i] = self.real_obj_dist_array[i] # From 0 to 32.Used to be [i] From 0 to 24
        #     self.sect2[i] = self.real_obj_dist_array[33+i] # From 33 to 65.Used to be [25+i] From 25 to 49
        #     self.sect3[i] = self.real_obj_dist_array[66+i] # From 66 to 98.Used to be [50+i] From 50 to 75
        #     #self.sect6[i] = self.real_obj_dist_array[171+i] # From 171 to 203
        #     self.sect7[i] = self.real_obj_dist_array[204+i] # From 204 to 236
        #     self.sect8[i] = self.real_obj_dist_array[237+i] # From 237 to 269.Used to be[180+i] From 180 to 204
        #     self.sect9[i] = self.real_obj_dist_array[270+i] # From 270 to 302. Used to be [205+i] From 205 to 229

        # Storing data from 180 to 0 degree sweep!
        # for j in range(33): # Used to be 227. Then it was 25
        # # 1st attempt with micro servo: Sectors 3,4,5,6 and 7 will have 26 data points. Sectors 1,2,8, and 9 will receive 25
        # # 2nd attempt with radar display speed: Sectors 4,5,and 6 will receive 35 points. Sectors 1,2,3,7,8 and 9 will receive 33
        #     self.sect9[j] = self.real_obj_dist_array[303+j] # From 303 to 335. Used to be [230+j] From 230 to 254
        #     self.sect8[j] = self.real_obj_dist_array[336+j] # From 336 to 368. Used to be [255+j] From 255 to 279
        #     self.sect7[i] = self.real_obj_dist_array[369+i] # From 369 to 401.
        #     self.sect3[i] = self.real_obj_dist_array[507+i] # From 507 to 539. Used to be [50+i] From 50 to 75
        #     self.sect2[j] = self.real_obj_dist_array[540+j] # From 540 to 572. Used to be [410+j] From 410 to 434
        #     self.sect1[j] = self.real_obj_dist_array[573+j] # From 573 to 605. Used to be [435+j] From 435 to 459

        # for j in range(35): # Used to be 26
        #     #self.sect7[j] = self.real_obj_dist_array[280+j] # Used to be [280+j] From 280 to 305
        #     self.sect6[j] = self.real_obj_dist_array[402+j] # From 402 to 436.Used to be [306+j] From 306 to 331
        #     self.sect5[j] = self.real_obj_dist_array[437+j] # From 437 to 471. Used to be [332+j] From 332 to 359
        #     self.sect4[j] = self.real_obj_dist_array[472+j] # From 472 to 506. Used to be [360+j]From 360 to 383
        #     #self.sect3[j] = self.real_obj_dist_array[384+j] # Used to be [384+j] From 384 to 409

    def process_data(self):
        # We process and analyze the information stored in sectors 4,5 and 6
        # Based on stored distance information, we set information about the sectors and ir robot can navigate through them
        # for i in range(35): # Used to be 51. Then it was 26
        #     if abs(self.sect5[i]) >= 1: # Checking to see if there is an object in sector 5 within 60 cm
        #         self.go_to_sect5 = True # There is no object detected within the 60 cm threshold of sector 5
        #     if abs(self.sect4[i]) >= 28: # Checking to see if there is an object in sector 4 within 60 cm
        #         self.go_to_sect4 = True # There is no object detected within the 60 cm threshold of sector 4
        #     if abs(self.sect6[i]) >= 28: # There is no object detected within the 60 cm threshold of sector 6
        #         self.go_to_sect6 = True # There is no object detected within the 60 cm threshold of sector 5

        for i in range(self.store_count_obj):
            if abs(self.new_array[i]) >= 0:
                self.go_to_sect5 = True
        self.robot_on_standby= False
        self.done_processing_data = True # Data has been processed, and robot is now ready to move!
        #print('Done processing data!')


    def move(self): # Function to make robot move

            if self.robot_on_standby == False: # Robot has finished collecting data and is processing it
                #if self.done_processing_data == True: # Robot has finished processing data and ready to move!
                    # self.scan_once_return = 1 # We will publish this value to have radar sweep again after we are done moving

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
                            self.scan_once_return = 2 # We will publish this value to have radar know we are still moving
                            self.cmd_pub.publish(self.twist) # Publishes
                            self.pub_scan_once_return()
                        if self.count >= 250:
                            self.twist.linear.x = 0 # Robot stops
                            self.twist.angular.z = 0
                            self.scan_once_return = 1
                            self.cmd_pub.publish(self.twist)
                            #self.count = 0 # Count variable reset so it can be reused
                            self.pub_scan_once_return() # Returns a value of 2 to have radar sweep again

                    # Op Mode 2: Navigate Right Side Around Object : Object is within 60 cm and stop button hasn't been pressed
                    if self.stop == False and self.go_to_sect6 == True:#self.go_to_sect5 == False and self.go_to_sect6 == True:
                        print('Hercules Going Right!')
                        if self.count < 150: # It takes about 5 seconds to get to a 300 count!
                            self.twist.linear.x = 0 # Robot steers right first
                            self.twist.angular.z = 20
                            self.scan_once_return = 2 # We will publish this value to have radar know we are still moving
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                        if 150 < self.count < 300: # From 150 to 300 is about 2.5 seconds
                            self.twist.linear.x = 0
                            self.twist.angular.z = -20 # Robot steers left second
                            self.scan_once_return = 2 # We will publish this value to have radar know we are still moving
                            self.pub_scan_once_return()
                            self.cmd_pub.publish(self.twist)
                        if 300 <= self.count < 450: # From 300 to 450 is about 2.5 seconds
                            self.twist.linear.x = 20
                            self.twist.angular.z = 0 # Robot then goes straight
                            self.scan_once_return = 2 # Robot is still moving
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                        if self.count >= 450:
                            self.twist.linear.x = 0
                            self.twist.angular.z = 0 # Robot then stops after a few seconds
                            self.scan_once_return = 1 # We will publish this value to have radar sweep again after we are done moving
                            self.cmd_pub.publish(self.twist)
                            #self.count = 0 # Reseting count variable to be reused
                            self.pub_scan_once_return() # Returns a value of 1 to have radar sweep again

                    # Op Mode 3: Navigate Left Side Around Object : Object is within 60 cm and stop button hasn't been pressed
                    if self.stop == False and self.go_to_sect4 == True:#self.go_to_sect5 == False and self.go_to_sect4 == True:
                        print('Hercules Going Left!')
                        if self.count < 150: # About 2.5 second travel time
                            self.twist.linear.x = 0
                            self.twist.angular.z = -20 # Robot goes left first!
                            self.scan_once_return = 2 # Robot is still moving
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                        if 150 <= self.count < 300: # From 150 to 300 is about 2.5 seconds
                            self.twist.linear.x = 0
                            self.twist.angular.z = 20 # Robot steers right second
                            self.scan_once_return = 2 # Robot is still moving
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                        if 300 <= self.count < 450: # From 300 to 450 is about 2.5 seconds
                            self.twist.linear.x = 20
                            self.twist.angular.z = 0 # Robot then goes straight
                            self.scan_once_return = 2 # Robot is still moving
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                        if self.count >= 450:
                            self.twist.linear.x = 0
                            self.twist.angular.z = 0 # Robot then stops after a few seconds
                            self.scan_once_return = 1 # Robot stops
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                        #self.count = 0 # Reseting count variable to be reused

            self.rate.sleep()

    def pub_scan_once_return(self):
        self.scan_return.publish(self.scan_once_return)

if __name__ == '__main__':
    Navigation()
