#!/usr/bin/env python
# Hercules Navigation Code
#
# Created By: Jeovanny Reyes
# Created On: February 24, 2018
#
# Subscriber: Subscribes to message topic "/HerculesUltrasound_Range" containing "Range" data
# Publisher: Publishes to message topic "cmd_vel" with "geometry_msgs/Twist" data
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

import math
import sys
