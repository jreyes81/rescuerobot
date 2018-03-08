#!/usr/bin/env python
# Line Section code: This contains the function to create the cross sectional lines
#
# Created By: Jeovanny Reyes
# Created On: March 5, 2018
#
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

import sys
import math
import pygame

from text_object_green import text_object_green

def linesections(xpos,ypos,green):
    center_of_circle = (xpos/2,ypos/2)
    left_edge = ((xpos/10),ypos/2)
    cent_to_lefedge = (center_of_circle[0]-left_edge[0],center_of_circle[1]-left_edge[1])
    smalltext = pygame.font.Font("freesansbold.ttf",(xpos/50) - (xpos/200))
    newpoint_x = [0] * 17
    newpoint_y = [0] * 17
    newpoint = [0] * 17
    n = range(20,360,20) # Creates 17 integers. Used for degree markers

    for i in range(17):
            newpoint_x[i] = ( math.cos(math.radians(n[i])) * cent_to_lefedge[0] ) + ( math.sin(math.radians(n[i])) * cent_to_lefedge[1] )
            newpoint_y[i] = ( -1 *math.sin(math.radians(n[i])) * cent_to_lefedge[0] ) + ( math.cos(math.radians(n[i])) * cent_to_lefedge[1] )

    newpoint1 = (center_of_circle[0] - newpoint_x[0], newpoint_y[0] + center_of_circle[1])
    newpoint2 = (center_of_circle[0] - newpoint_x[1], newpoint_y[1] + center_of_circle[1])
    newpoint3 = (center_of_circle[0] - newpoint_x[2], newpoint_y[2] + center_of_circle[1])
    newpoint4 = (center_of_circle[0] - newpoint_x[3], newpoint_y[3] + center_of_circle[1])
    newpoint5 = (center_of_circle[0] - newpoint_x[4], newpoint_y[4] + center_of_circle[1])
    newpoint6 = (center_of_circle[0] - newpoint_x[5], newpoint_y[5] + center_of_circle[1])
    newpoint7 = (center_of_circle[0] - newpoint_x[6], newpoint_y[6] + center_of_circle[1])
    newpoint8 = (center_of_circle[0] - newpoint_x[7], newpoint_y[7] + center_of_circle[1])
    newpoint9 = (center_of_circle[0] - newpoint_x[8], newpoint_y[8] + center_of_circle[1])
    newpoint10 = (center_of_circle[0] - newpoint_x[9], newpoint_y[9] + center_of_circle[1])
    newpoint11 = (center_of_circle[0] - newpoint_x[10], newpoint_y[10] + center_of_circle[1])
    newpoint12 = (center_of_circle[0] - newpoint_x[11], newpoint_y[11] + center_of_circle[1])
    newpoint13 = (center_of_circle[0] - newpoint_x[12], newpoint_y[12] + center_of_circle[1])
    newpoint14 = (center_of_circle[0] - newpoint_x[13], newpoint_y[13] + center_of_circle[1])
    newpoint15 = (center_of_circle[0] - newpoint_x[14], newpoint_y[14] + center_of_circle[1])
    newpoint16 = (center_of_circle[0] - newpoint_x[15], newpoint_y[15] + center_of_circle[1])
    newpoint17 = (center_of_circle[0] - newpoint_x[16], newpoint_y[16] + center_of_circle[1])

    textSurf1, textRect1 = text_object_green("20 deg", smalltext)
    textRect1.center = ( newpoint1[0] - (xpos * 0.025), newpoint1[1])
    pygame.display.get_surface().blit(textSurf1, textRect1)


    textSurf2, textRect2 = text_object_green("40 deg", smalltext)
    textRect2.center = ( newpoint2[0] - (xpos*0.030), newpoint2[1])
    pygame.display.get_surface().blit(textSurf2, textRect2)

    textSurf3, textRect3 = text_object_green("60 deg", smalltext)
    textRect3.center = ( newpoint3[0] - (xpos*0.025), newpoint3[1] - (ypos* 0.005))
    pygame.display.get_surface().blit(textSurf3, textRect3)

    textSurf4, textRect4 = text_object_green("80 deg", smalltext)
    textRect4.center = ( newpoint4[0] - (xpos*0.025), newpoint4[1] - (ypos * 0.010))
    pygame.display.get_surface().blit(textSurf4, textRect4)

    textSurf5, textRect5 = text_object_green("100 deg", smalltext)
    textRect5.center = ( newpoint5[0] + (xpos*0.025), newpoint5[1] - (ypos*0.010))
    pygame.display.get_surface().blit(textSurf5, textRect5)

    textSurf6, textRect6 = text_object_green("120 deg", smalltext)
    textRect6.center = ( newpoint6[0] + (xpos*0.025), newpoint6[1] - (ypos*0.005))
    pygame.display.get_surface().blit(textSurf6, textRect6)

    textSurf7, textRect7 = text_object_green("140 deg", smalltext)
    textRect7.center = ( newpoint7[0] + (xpos*0.030), newpoint7[1] - (ypos*0.010))
    pygame.display.get_surface().blit(textSurf7, textRect7)

    textSurf8, textRect8 = text_object_green("160 deg", smalltext)
    textRect8.center = ( newpoint8[0] + (xpos*0.025), newpoint8[1] )
    pygame.display.get_surface().blit(textSurf8, textRect8)

    textSurf9, textRect9 = text_object_green("90 deg", smalltext)
    textRect9.center = ( xpos/2,(xpos*0.085))
    pygame.display.get_surface().blit(textSurf9, textRect9)

    screen = pygame.display.get_surface()

    pygame.draw.aaline(screen, green, center_of_circle, newpoint1,5) # Left Hand Side
    pygame.draw.aaline(screen, green, center_of_circle, newpoint2,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint3,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint4,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint5,5) # Right Hand Side
    pygame.draw.aaline(screen, green, center_of_circle, newpoint6,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint7,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint8,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint9,5) # Bottom Right Side
    pygame.draw.aaline(screen, green, center_of_circle, newpoint10,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint11,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint12,5) # Bottom Left Side
    pygame.draw.aaline(screen, green, center_of_circle, newpoint13,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint14,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint15,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint16,5)
    pygame.draw.aaline(screen, green, center_of_circle, newpoint17,5) # Creates 9 sectors above x axis
