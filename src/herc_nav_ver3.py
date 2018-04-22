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
        self.state_count = 0
        self.sector_points = 0 # Number of sector points that sectors 4,5,and 6 will receive
        self.scan_once_return = 0 # Used to send state information back to radar display to scan again
        self.count = 0
        self.store_count_obj = 0
        self.robot_on_standby = True
        self.done_processing_data = False # Used to tell when robot will move when data is done being processed!
        self.go_to_sect4 = False # Used to tell if robot will navigate through sector 4
        self.go_to_sect5 = True # Used to tell if robot will navigate through sector 5
        self.go_to_sect6 = False # Used to tell if robot will navigate through sector 6
        self.state_of_radar = True
        # Based on micro sweep speed. We collect 460 points worth of data. Based on radar display sweep, we collecct 606 points of data, but will use 612
        self.new_array = 0
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

        # Publisher
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) # Rate set in Hz
        self.scan_return = rospy.Publisher("scan_once_return", Float32, queue_size=10) # Returns 2 to have robot scan again as soon as robot is finished moving

        self.twist = Twist() # Contains linear and angular information
        self.robot_state()

    def scan_once_state_cb(self,data):
        self.scan_once_state = data.data # 0 is when radar is still sweeping. 1 is when radar has stopped sweeping
    #
    # def count_for_move(self):
    #     if self.scan_once_state == 0:
    #         self.count = 0
    #     self.count = self.count+1

    def robot_state(self):
        #if self.scan_once_state == 2:
        #    self.count = 0
        while not rospy.is_shutdown():
            while self.scan_once_state == 0: #and self.robot_on_standby == True: # Robot scanning and awaitng orders!
                print("Robot Scanning and Storing Data. Hercules on Standby!")
                self.go_to_sect5 = True # Reseting the boolean values for robot to travel through
                self.go_to_sect4 = False
                self.go_to_sect6 = False
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.scan_once_return = 1
                self.count = 0
                self.cmd_pub.publish(self.twist)
                # self.scan_once_return = 1
                self.pub_scan_once_return() # Returns a value of 1 to have radar know we received a 0 and we won't move

            while self.scan_once_state == 1 : #or self.scan_once_state == 2: # Robot dones scanning and ready to process data
                self.done_processing_data = False
                # self.go_to_sect5 =  True
                self.state_count = self.state_count +1 # for first movement, it is 1
                # self.go_to_sect4 = True
                # self.go_to_sect6 = True
                #self.scan_once_return = 2
                #self.pub_scan_once_return() # Returns a value of 2 to let radar node we know it is done sweeping and we are going to move
                print("Robot Stopped Scanning and Processing Data!")
                rospy.loginfo(self.store_count_obj)
                self.count = self.count+1
                print(self.count)
                self.store_data_in_sectors() # Seperates the self.real_obj_dist_array into the 9 sectors to process data
                print("Finished seperating data in sectors")
                self.process_data()
                print("Finished processing data!")
                print(self.done_processing_data) # Should be "True"
                print(self.robot_on_standby) # Should be "False"
                print("State for sector 5:")
                print(self.go_to_sect5) # If "True", we can move straight
                print("State for sector 4:")
                print(self.go_to_sect4) # If "True" there is no object in this sector
                print("State for sector 6:")
                print(self.go_to_sect6) # If "True" there is no object in this sector
                if self.done_processing_data == True: # Robot has finished processing data and ready to send commands to motor controller to move!
                #print("Done Processing Data!")
                    self.move()
                    # print(self.count)
                    # self.count = self.count + 1 # It takes about 5 seconds to get to a 300 count!
                    # print(self.count)

    def distcallback(self,data): # Takes in message "range" as input. Data comes back in cm. This part of code works well!
        if self.scan_once_state == 0: # As long as radar is sweeping, store data
            print('Useful Data!')
            self.count_size_array = self.count_size_array + 1
            self.new_array = [0]*self.count_size_array # Array gets resized until robot is done scanning
            #print(self.store_count_obj) # To make sure count gets incremented
            real_obj_dist = data.range # Stores object distance (raw data) info into local variable
            self.real_obj_dist_array[self.store_count_obj] = real_obj_dist # Stores local variable into an array to process data
            # print(self.real_obj_dist_array[1:self.store_count_obj]) # Should print the range of values according to the count
            #print(len(self.real_obj_dist_array)) # Prints the number of elements in the array. Useful to see how we can process raw data
            #print(self.real_obj_dist_array[self.store_count_obj])
            self.store_count_obj = self.store_count_obj + 1 # Increments to store next data we obtain as sensor sweeps
        if self.scan_once_state == 1: # Radar has stopped sweeping, we dont care about storing data in our processing array
            print('Not useful Data!')

            self.fake_obj_dist = data.range # Don't care about this data!

    def stop_cb(self,data): # Takes in bool values. Data messages come in as True when "STOP" button is pressed
        self.stop = data.data

    def store_data_in_sectors(self):

        self.new_array = [0]*self.store_count_obj
        for i in range(self.store_count_obj):
            self.new_array[i] = self.real_obj_dist_array[i] # Transferring data from the pre created array to a new size array.
            # This new array will contain all of the raw data
        print(self.new_array)  # Should print out the array containing all the raw data collected
        # example: Say self.new_array has 1368 data points. Split these int0 9 sectors
        numel_of_new_array = len(self.new_array) # Contains the number of elements in the array

        if numel_of_new_array%2==0: # If the size of our array is perfectly divisble by half
            half_data = numel_of_new_array/2 # Splitting data points into two parts first: From 5 to 175 degrees and from 175 to 5!
            # 1368/2 = 684. 2nd example is 1000/2 = 500
            if half_data%9 == 0: # If the number of data points for each of the 9 sectors is even
                half_sweep_sector_points = half_data/9 # 76 data points
                self.sector_points = (half_data/9)*2 # An even amount of data points for each sector. Say we have 152 points for each sector
                self.sect1 = [0] * self.sector_points # 152 points. 76 for one sweep and the 76 for the other
                self.sect2 = [0] * self.sector_points
                self.sect3 = [0] * self.sector_points
                self.sect4 = [0] * self.sector_points
                self.sect5 = [0] * self.sector_points
                self.sect6 = [0] * self.sector_points
                self.sect7 = [0] * self.sector_points
                self.sect8 = [0] * self.sector_points
                self.sect9 = [0] * self.sector_points

                for j in range(half_sweep_sector_points): # Storing 76 data points into each sector from the new array. 5 to 175
                    self.sect1[j] = self.new_array[j] # From 0 to 75
                    self.sect2[j] = self.new_array[half_sweep_sector_points+j] # From 76 to 151
                    self.sect3[j] = self.new_array[(half_sweep_sector_points*2)+j] # From 152 to 227
                    self.sect4[j] = self.new_array[(half_sweep_sector_points*3)+j] # From 228 to 303
                    self.sect5[j] = self.new_array[(half_sweep_sector_points*4)+j] # From 304 to 379
                    self.sect6[j] = self.new_array[(half_sweep_sector_points*5)+j] # From 380 to 455
                    self.sect7[j] = self.new_array[(half_sweep_sector_points*6)+j] # From 456 to 531
                    self.sect8[j] = self.new_array[(half_sweep_sector_points*7)+j] # From 532 to 607
                    self.sect9[j] = self.new_array[(half_sweep_sector_points*8)+j] # From 608 to 683

                for k in range(half_sweep_sector_points): # Storing 76 data points into each sector from the new array. 175 to 5
                    self.sect9[k] = self.new_array[(half_sweep_sector_points*9)+k] # From 684 to 759
                    self.sect8[k] = self.new_array[(half_sweep_sector_points*10)+k] # From 760 to 835
                    self.sect7[k] = self.new_array[(half_sweep_sector_points*11)+k] # From 836 to 911
                    self.sect6[k] = self.new_array[(half_sweep_sector_points*12)+k] # From 912 to 987
                    self.sect5[k] = self.new_array[(half_sweep_sector_points*13)+k] # From 988 to 1063
                    self.sect4[k] = self.new_array[(half_sweep_sector_points*14)+k] # From 1064 to 1139
                    self.sect3[k] = self.new_array[(half_sweep_sector_points*15)+k] # From 1140 to 1215
                    self.sect2[k] = self.new_array[(half_sweep_sector_points*16)+k] # From 1216 to 1291
                    self.sect1[k] = self.new_array[(half_sweep_sector_points*17)+k] # From 1292 to 1367


            else:                # If the number of data points for each sector is NOT even
                half_sweep_sector_points = half_data/9 # 500/9=55
                if ((half_sweep_sector_points*9) != half_data): # 55*9 != 500 -> 495!=500
                    extra_points = half_data - (half_sweep_sector_points*9) # 500-495 = 5. Sector 9 will receive the extra points
                self.sector_points = (half_data/9)*2 # (500/9)*2 = 110 points for each sector. Sector 9
                self.sect1 = [0] * self.sector_points # 110 points. 55 for one sweep and the 5 for the other
                self.sect2 = [0] * self.sector_points
                self.sect3 = [0] * self.sector_points
                self.sect4 = [0] * self.sector_points
                self.sect5 = [0] * self.sector_points
                self.sect6 = [0] * self.sector_points
                self.sect7 = [0] * self.sector_points
                self.sect8 = [0] * self.sector_points
                self.sect9 = [0] * (self.sector_points+(extra_points*2)) # This will receive the extra points. So 110 + 5*2=120

                for j in range(half_sweep_sector_points): # Storing 55 data points into each sector form the new array. 5 to 175 degrees
                    self.sect1[j] = self.new_array[j] # From 0 to 54
                    self.sect2[j] = self.new_array[half_sweep_sector_points+j] # From 55 to 109
                    self.sect3[j] = self.new_array[(half_sweep_sector_points*2)+j] # From 110 to 164
                    self.sect4[j] = self.new_array[(half_sweep_sector_points*3)+j] # From 165 to 219
                    self.sect5[j] = self.new_array[(half_sweep_sector_points*4)+j] # From 220 to 274
                    self.sect6[j] = self.new_array[(half_sweep_sector_points*5)+j] # From 275 to 329
                    self.sect7[j] = self.new_array[(half_sweep_sector_points*6)+j] # From 330 to 384
                    self.sect8[j] = self.new_array[(half_sweep_sector_points*7)+j] # From 385 to 439
                    self.sect9[j] = self.new_array[(half_sweep_sector_points*8)+j] # From 440 to 494
                for k in range((extra_points*2)): # For 5*2 = 10
                    self.sect9[k] = self.new_array[(half_sweep_sector_points*9)+k] # From 495 to 504
                for l in range(half_sweep_sector_points): # Storing 55 data points into each sector from the new array. 175 to 5 degrees.
                    self.sect9[l] = self.new_array[(half_sweep_sector_points*9)+(extra_points*2)+l] # From 505 to 559
                    self.sect8[l] = self.new_array[(half_sweep_sector_points*10)+(extra_points*2)+l] # From 560 to 614
                    self.sect7[l] = self.new_array[(half_sweep_sector_points*11)+(extra_points*2)+l] # From 615 to 669
                    self.sect6[l] = self.new_array[(half_sweep_sector_points*12)+(extra_points*2)+l] # From 670 to 724
                    self.sect5[l] = self.new_array[(half_sweep_sector_points*13)+(extra_points*2)+l] # From 725 to 779
                    self.sect4[l] = self.new_array[(half_sweep_sector_points*14)+(extra_points*2)+l] # From 780 to 834
                    self.sect3[l] = self.new_array[(half_sweep_sector_points*15)+(extra_points*2)+l] # From 835 to 889
                    self.sect2[l] = self.new_array[(half_sweep_sector_points*16)+(extra_points*2)+l] # From 890 to 944
                    self.sect1[l] = self.new_array[(half_sweep_sector_points*17)+(extra_points*2)+l] # From 945 to 999

            # For data points that were collected from 5 to 175 degrees! [500 points | From 0 to 499]
        #    for j in range (60):
        #        self.sect1[j] =
            # For data points that were collected from 175 to 5 degrees! [500 points | From 500 to 999]
        #else:                       # If the size of our array is NOT perfectly divisible by half
        #    for k in range(60):
        #           self.sect1[k]

    def process_data(self):
        # We process and analyze the information stored in sectors 4,5 and 6
        # Based on stored distance information, we set information about the sectors and ir robot can navigate through them
        print(self.sect5)
        print(self.sect4)
        print(self.sect6)
        sect5data = rospy.loginfo(self.sect5)
        # sect4data = rospy.loginfo(self.sect4)
        # sect6data = rospy.loginfo(self.sect6)
        for i in range(self.sector_points): # 152 points
            if 0 < abs(self.sect5[i]) <= 68: # Checking to see if there is an object in sector 5 within 60 cm
                self.go_to_sect5 = False # There is an object detected within the 60 cm threshold of sector 5
                self.go_to_sect6 = True # Possibility of going through sector 6
                self.go_to_sect4 = True # Possibility of going through sector 4
            if 0 < abs(self.sect4[i]) <= 38: # Checking to see if there is an object in sector 4 within 60 cm
                self.go_to_sect4 = False # There is an object detected within the 60 cm threshold of sector 4. We will go
            if 0 < abs(self.sect6[i]) <= 38: # There is no object detected within the 60 cm threshold of sector 6
                self.go_to_sect6 = False # There is no object detected within the 60 cm threshold of sector 5

        if self.go_to_sect6 == True:
            self.go_to_sect4 =  False # We only want to navigate through one sector

        # rospy.loginfo(self.go_to_sect4)
        rospy.loginfo(self.go_to_sect5)
        # for i in range(self.store_count_obj):
        #     if abs(self.new_array[i]) >= 0:
        #         self.go_to_sect5 = True
        self.robot_on_standby= False
        self.done_processing_data = True # Data has been processed, and robot is now ready to move!
        #print('Done processing data!')


    def move(self): # Function to make robot move

            if self.robot_on_standby == False: # Robot has finished collecting data and is processing it
                if self.done_processing_data == True: # Robot has finished processing data and ready to move!
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
                        if self.count < 50: # 300 counts is about 5 seconds. 150 counts is about 2.5 seconds
                            self.twist.linear.x = 20 # Robot moves forward
                            self.twist.angular.z = 0 # Robot does not rotate
                            self.twist.linear.z = 0
                            self.scan_once_return = 2 # We will publish this value to have radar know we are still moving
                            # rospy.loginfo(self.count)
                            print(self.count)
                            # self.count = self.count+1
                            self.cmd_pub.publish(self.twist) # Publishes
                            self.pub_scan_once_return()
                        if self.count >= 50:
                            print("Hercules has stopped")
                            self.twist.linear.x = 0 # Robot stops
                            self.twist.angular.z = 0
                            self.scan_once_return = 1
                            self.cmd_pub.publish(self.twist)
                            self.go_to_sect4 =  False
                            self.go_to_sect6 =  False
                            # self.store_count_obj = 0
                            #self.count = 0 # Count variable reset so it can be reused
                            self.pub_scan_once_return() # Returns a value of 2 to have radar sweep again

                    # Op Mode 2: Navigate Right Side Around Object : Object is within 60 cm and stop button hasn't been pressed
                    if self.stop == False and self.go_to_sect6 == True:#self.go_to_sect5 == False and self.go_to_sect6 == True:
                        print('Hercules Going Right!')
                        if self.count < 50: # It takes about 5 seconds to get to a 300 count!
                            self.twist.linear.x = 0 # Robot steers right first
                            self.twist.angular.z = 20
                            self.scan_once_return = 2 # We will publish this value to have radar know we are still moving
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                      #  if 150 < self.count < 300: # From 150 to 300 is about 2.5 seconds
                      #      self.twist.linear.x = 0
                      #      self.twist.angular.z = -20 # Robot steers left second
                      #      self.scan_once_return = 2 # We will publish this value to have radar know we are still moving
                      #      self.pub_scan_once_return()
                      #      self.cmd_pub.publish(self.twist)  
                        if  50 <= self.count < 100: # From 300 to 450 is about 2.5 seconds
                            self.twist.linear.x = 20
                            self.twist.angular.z = 0 # Robot then goes straight
                            self.scan_once_return = 2 # Robot is still moving
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                        if self.count >= 100:
                            self.twist.linear.x = 0
                            self.twist.angular.z = 0 # Robot then stops after a few seconds
                            self.scan_once_return = 1 # We will publish this value to have radar sweep again after we are done moving
                            self.cmd_pub.publish(self.twist)
                            #self.count = 0 # Reseting count variable to be reused
                            self.pub_scan_once_return() # Returns a value of 1 to have radar sweep again

                    # Op Mode 3: Navigate Left Side Around Object : Object is within 60 cm and stop button hasn't been pressed
                    if self.stop == False and self.go_to_sect4 == True:#self.go_to_sect5 == False and self.go_to_sect4 == True:
                        print('Hercules Going Left!')
                        if self.count < 50: # About 2.5 second travel time
                            self.twist.linear.x = 0
                            self.twist.angular.z = -20 # Robot goes left first!
                            self.scan_once_return = 2 # Robot is still moving
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                          # if 150 <= self.count < 300: # From 150 to 300 is about 2.5 seconds
                          # self.twist.linear.x = 0
                          # self.twist.angular.z = 20 # Robot steers right second
                          # self.scan_once_return = 2 # Robot is still moving
                          # self.cmd_pub.publish(self.twist)
                          # self.pub_scan_once_return() 
                        if 50 <= self.count < 100: # From 300 to 450 is about 2.5 seconds
                            self.twist.linear.x = 20
                            self.twist.angular.z = 0 # Robot then goes straight
                            self.scan_once_return = 2 # Robot is still moving
                            self.cmd_pub.publish(self.twist)
                            self.pub_scan_once_return()
                        if self.count >= 100:
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
