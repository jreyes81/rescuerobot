#!/usr/bin/env python
# Text Color code: This contains the function to make text black
#
# Created By: Jeovanny Reyes
# Created On: March 5, 2018
#
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

import pygame
import sys

def text_object_black(text,font):
    color_black = (0,0,0)
    textSurface = font.render(text,True,color_black)

    return textSurface, textSurface.get_rect()
