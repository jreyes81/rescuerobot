#!/usr/bin/env python
# learningdictionaries.py is a file that has a function called f. The return 'value' will be a function
# that is a maneuvering procedure


# Created By: Mario Medina
# Created On: March 14, 2018

# Publisher: "

# Raytheon Radar Guided Rescue Robot
# Cal State LA Senior Design

# case 1: object is in front, avoids it by going through the left
# case 2: object is on the left side, robot avoids it by going through its right
# case 3: object is on the right side, robot avoids it by going through its left
from geometry_msgs.msg import Twist
from opmodes import opmodes

cases = {
 'opmode1': opmode1,
 'opmode2': opmode2,
 'opmode3': opmode3

 }

print cases['opmode99']




# cases['opmode1']  # returns only first element 

'''
# using the dictionary  get(key[, default])  method:
def f(x):
    return {
        'opmode1': opmode1(),
        'opmode2': opmode2(),
        'opmode3': opmode3()
    }.get(x, 9)    # 9 is default if x not found --- should be replaced with opmode99()

'''
