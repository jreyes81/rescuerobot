#!/usr/bin/env python
# Labels code: This contains the function to create the labels "Object Distance"
#              and "Servo Angle:"
#
# Created By: Jeovanny Reyes
# Created On: March 5, 2018
#
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

import sys
import pygame

from text_object_green import text_object_green
from text_object_red import text_object_red

def labels(xpos,ypos,real_obj_dist):
    x_pos = xpos * 0.625
    y_pos = ypos * 0.005
    x_length = xpos * 0.150
    y_width = ypos * 0.100
    fontsize = int(xpos * 0.03)
    servo_ang_ypos = ypos * 0.85

    smalltext = pygame.font.Font("freesansbold.ttf",fontsize)

    textSurfstop, textRectstop = text_object_green("Object Distance:", smalltext)
    textRectstop.center = ( (x_pos +(x_length/2)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurfstop, textRectstop)

    if real_obj_dist < 100:
        textSurfstop1, textRectstop1 = text_object_green("Object Detected", smalltext)
        textRectstop1.center = ( (x_pos +(x_length/2) - (xpos*0.300)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurfstop1, textRectstop1)
    else:
        textSurfstop1, textRectstop1 = text_object_red("Object Out of Range", smalltext)
        textRectstop1.center = ( (x_pos +(x_length/2) - (xpos*0.300)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurfstop1, textRectstop1)

    textSurfstop2, textRectstop2 = text_object_green("Servo Angle: ", smalltext)
    textRectstop2.center = ( (x_pos +(x_length/2) - xpos * 0.550), y_pos+(y_width/2) + servo_ang_ypos) # Used to be 850
    pygame.display.get_surface().blit(textSurfstop2, textRectstop2)
