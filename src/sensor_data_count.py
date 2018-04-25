#!/usr/bin/env python
# sensor_data_count.py counts how many points we get from a sweep

# Modified By: Jeovanny Reyes
# Modified On: February 10, 2018

# Subscriber: "scan_once_to_whls" message topic of float 32 data type
# Pubslisher: None

# Raytheon Radar Guided Rescue Robot
# Cal State LA Senior Design

import rospy
from std_msgs.msg import Float32

class Count(object):
    def __init__(self):
        rospy.init_node("navigation_node", anonymous=True)
        print('Navigation Node Started!')
        self.count = 0
        self.scan_once_state = 3
        self.rate = rospy.Rate(60) # Pubslishing at 60 hz
        # rospy.init_node("navigation_node", anonymous=True)
        # print('Navigation Node Started!')

        self.scan_once = rospy.Subscriber("scan_once_to_whls",Float32, self.scan_once_state_cb) # Recieves 0 when radar is still sweeping, 1 when radar is done
        #self.ran_sub = rospy.Subscriber("HerculesUltrasound_Range", Range, self.distcallback) #Used to be Float32. [Float 32, callback])

        while not rospy.is_shutdown():
            if self.scan_once_state == 0:
                self.count = self.count + 1
                print(self.count)
            else:
                print("Awaiting orders")
            self.rate.sleep()

    def scan_once_state_cb(self,data):
        self.scan_once_state = data.data # 0 is when radar is still sweeping. 1 is when radar has stopped sweeping
        #self.count = self.count + 1
        #if self.scan_once_state == 1: # For 2nd iterations and above when robot sweeps again
        #    self.count = 0 # Reseting count when radar scans again and we need to move again!

    #def distcallback(self,data): # Takes in message "range" as input. Data comes back in cm. This part of code works well!
    #    if self.scan_once_state == 0: # As long as radar is sweeping, store data
    #        print('Useful Data!')
            #print(self.store_count_obj) # To make sure count gets incremented
    #        real_obj_dist = data.range # Stores object distance (raw data) info into local variable
    #        self.real_obj_dist_array[self.store_count_obj] = real_obj_dist # Stores local variable into an array to process data
    #        print(self.real_obj_dist_array[1:self.store_count_obj]) # Should print the range of values according to the count
    #        print(len(self.real_obj_dist_array)) # Prints the number of elements in the array. Useful to see how we can process raw data
            #print(self.real_obj_dist_array[self.store_count_obj])
    #        self.store_count_obj = self.store_count_obj + 1 # Increments to store next data we obtain as sensor sweeps
    #    if self.scan_once_state == 1: # Radar has stopped sweeping, we dont care about storing data in our processing array
    #        print('Not useful Data!')
    #        fake_obj_dist = data.range # Don't care about this data!

if __name__ == '__main__':
    Count()
