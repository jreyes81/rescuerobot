#!/usr/bin/env python
# tf_samp.py: Sets up the "world" frame (origin) and "Hercules" frame.
# This is a tf broadcaster

# Created By: Jeovanny Reyes
# Created On: February 12, 2018
# Modified On: February 13, 2018

import roslib
import rospy
import tf

def main():
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2.0, 0.0), # Takes in five paramters
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "hercules", # Child frame
                         "world")  # Parent Frame

    rate.sleep()

if __name__ == "__main__":
    main()
