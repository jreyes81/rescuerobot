#!/usr/bin/env python
# Integer to String code: This contains the function to make convert the range
#                         integers to strings as well as the degree integers to strings
#
# Created By: Jeovanny Reyes
# Created On: March 6, 2018
#
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

import sys
import pygame

from text_object_green import text_object_green

def range_to_string(xpos,ypos,real_obj_dist,servo_pos):
    x_pos = xpos * 0.800
    y_pos = ypos * 0.005
    x_length = xpos * 0.150
    y_width = ypos * 0.100
    str_dist = str(real_obj_dist) # converting range integer to string
    str_pos = str(servo_pos) # Converting angle to string

    #str_pos = str(new_servo_pos) # Converts angle integers to string. position is calculated instead of taking data from micro servo
    fontsize = int(xpos * 0.030)

    smalltext = pygame.font.Font("freesansbold.ttf",fontsize)

    textSurfstop, textRectstop = text_object_green(str_dist + "cm", smalltext)
    textRectstop.center = ( (x_pos +(x_length/2)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurfstop, textRectstop)

    textSurfstop1, textRectstop1 = text_object_green(str_pos + "degrees", smalltext)
    textRectstop1.center = ( (x_pos +(x_length/2) - (xpos*0.540)), y_pos+(y_width/2) + (ypos*0.850))
    pygame.display.get_surface().blit(textSurfstop1, textRectstop1)
