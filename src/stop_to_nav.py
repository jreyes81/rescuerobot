#!/usr/bin/env python
# Stop To Navigation code: This contains function passes the stop information from
#                          the robot_stop.py file to herc_nav.py
#
# Created By: Jeovanny Reyes
# Created On: March 3, 2018
#
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

import pygame

def stop_to_nav():
    while(True):
        pygame.init()
        mouse = pygame.mouse.get_pos()	 # Obtains position of mouse
        click = pygame.mouse.get_pressed()
        xpos = 600
        ypos = 600
        x_pos = xpos * 0.050 # X Coordinate for button position
        y_pos = ypos * 0.035 # Y Coordinate for button position
        x_length = xpos * 0.150 # Length of "STOP" button
        y_width  = ypos * 0.050 # Width of "STOP" button

        stop = False # Initially contains False boolean

        xposlength = x_pos + x_length
        yposwidth = y_pos + y_width

        if ((xposlength > mouse[0] > x_pos) and (yposwidth > mouse[1] > y_width)):
            if click[0] == 1:
                stop = True
                return stop
        print(stop)
        print(click)
        return stop
