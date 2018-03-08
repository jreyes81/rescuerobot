#!/usr/bin/env python
# Robot Stop Button code: This contains code for the "STOP" button in the radar display
#
# Created By: Jeovanny Reyes
# Created On: March 3, 2018
#
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

import math
import time
import pygame

from text_object_black import text_object_black

def stop_button(xpos,ypos,stop_val,brightred,red):
	mouse = pygame.mouse.get_pos()	 # Obtains position of mouse
	click = pygame.mouse.get_pressed() # Obtains information when mouse gets clicked
	fontsize = int(xpos * 0.030)
	smalltext = pygame.font.Font("freesansbold.ttf",30)
	x_pos = xpos * 0.050 # X Coordinate for button position
	y_pos = ypos * 0.035 # Y Coordinate for button position
	x_length = xpos * 0.150 # Length of "STOP" button
	y_width = ypos * 0.050 # Width of "STOP" button

	stop = stop_val # Initialy contains False boolean

	xposlength = x_pos + x_length
	yposwidth = y_pos + y_width

	# return xposlength, yposwidth, mouse, x_pos, y_pos, x_length, y_width

	if ((xposlength > mouse[0] > x_pos) and (yposwidth > mouse[1] > y_width)): # Hovering over box
		pygame.draw.rect(pygame.display.get_surface(),brightred,(x_pos,y_pos,x_length,y_width))
		if click [0] == 1: # When we left click on the "STOP" button
			print('Robot has stopped moving!')
			stop = True # Value is then set to Stop
	else:
		pygame.draw.rect(pygame.display.get_surface(),red,(x_pos,y_pos,x_length,y_width))

	textSurf,textRect = text_object_black("STOP",smalltext)
	textRect.center = ((x_pos +(x_length/2)), y_pos+(y_width/2))
	pygame.display.get_surface().blit(textSurf, textRect)

def tranf_stop
