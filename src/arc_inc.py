#!/usr/bin/env python
# Markers for Arc Increments code: This contains the function to create markers (e.g. 25 cm, 50 cm) for the arcs
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

def arc_inc(xpos,ypos):
    x_pos = xpos * 0.328
    y_pos = ypos * 0.460
    x_length = xpos * 0.150
    y_width = ypos * 0.100
    fontsize = int(xpos * 0.015)
    smalltext = pygame.font.Font("freesansbold.ttf",fontsize)

    textSurf1, textRect1 = text_object_green("25 cm", smalltext)
    textRect1.center = ( (x_pos +(x_length/2)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurf1, textRect1)

    textSurf2, textRect2 = text_object_green("50 cm", smalltext)
    textRect2.center = ( (x_pos +(x_length/2) - (xpos/10)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurf2, textRect2)

    textSurf3, textRect3 = text_object_green("75 cm", smalltext)
    textRect3.center = ( (x_pos +(x_length/2) - (xpos/5)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurf3, textRect3)

    textSurf4, textRect4 = text_object_green("100 cm", smalltext)
    textRect4.center = ( (x_pos +(x_length/2) - (xpos*0.300)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurf4, textRect4)

    textSurf5, textRect5 = text_object_green("25 cm", smalltext)
    textRect5.center = ( (x_pos +(x_length/2) + (xpos/5)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurf5, textRect5)

    textSurf6, textRect6 = text_object_green("50 cm", smalltext)
    textRect6.center = ( (x_pos +(x_length/2) + (xpos*0.300)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurf6, textRect6)

    textSurf7, textRect7 = text_object_green("75 cm", smalltext)
    textRect7.center = ( (x_pos +(x_length/2) + (xpos/2.5)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurf7, textRect7)

    textSurf8, textRect8 = text_object_green("100 cm", smalltext)
    textRect8.center = ( (x_pos +(x_length/2) + (xpos/2)), y_pos+(y_width/2))
    pygame.display.get_surface().blit(textSurf8, textRect8) # cm and deg markers # cm markers ,e.g. 20 cm
